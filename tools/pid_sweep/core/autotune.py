"""Autotuning de (kP_t, kP_r) para el PPHolonomicDriveController.

Metodo de entropia cruzada (CEM) en escala logaritmica:
  1. Muestrea `population` candidatos de una normal en (log kP_t, log kP_r).
  2. Cada candidato corre con `robots_per_candidate` robots aleatorizados (los mismos para todos
     los candidatos de la generacion) en todas las trayectorias + la prueba mantener_pose, con
     sus kP y con kP / margin_fraction.
  3. Candidato valido = cero inestables en ambas corridas. Costo = percentil 95 del costo por
     corrida (error RMS + error final, posicion y heading). Invalidos van al final.
  4. La distribucion se mueve hacia la elite. Al final, los mejores candidatos se re-evaluan
     juntos con mas robots, se elige el mejor valido y se valida con miles de robots nuevos y
     escenarios fuera de rango.

Uso (desde tools/pid_sweep):
  python -m core.autotune --device cuda                 # todas las variantes
  python -m core.autotune --variant actual_openloop     # solo una
  python -m core.autotune --quick                       # prueba rapida
"""

from __future__ import annotations

import argparse
import copy
import csv
import math
import time
from pathlib import Path

import torch

from .params import PACKAGE_DIR, EnvParams, load_config, sample_env_params, variant
from .sweep import all_trajectories, evaluate
from .trajectory import HOLD_NAME


def run_cost(cfg: dict, res: dict) -> torch.Tensor:
    c = cfg["autotune"]["cost"]
    w, d2m = float(c["w_final"]), float(c["deg_to_m"])
    cost = (res["pos_rms_m"] + w * res["final_pos_err_m"]
            + d2m * (res["yaw_rms_deg"] + w * res["final_yaw_err_deg"]))
    cost = cost + float(c["abort_penalty_m"]) * res["abort"].float()
    return torch.nan_to_num(cost, nan=1e3, posinf=1e3)


def score_candidates(cfg: dict, var: dict, kps: torch.Tensor, n_robots: int, seed: int, device="cpu",
                     trajs=None, margin: bool = True, max_batch: int = 200_000) -> dict:
    """Evalua cada fila de kps (P,2) con los mismos n_robots robots. Devuelve tensores (P,)."""
    trajs = trajs or all_trajectories(cfg)
    p = kps.shape[0]
    gen = torch.Generator().manual_seed(seed)
    base = sample_env_params(cfg, torch.zeros(n_robots), torch.zeros(n_robots), gen)

    def batch(scale: float) -> EnvParams:
        envs = [base.replace(kp_t=torch.full((n_robots,), float(kps[i, 0]) * scale),
                             kp_r=torch.full((n_robots,), float(kps[i, 1]) * scale)) for i in range(p)]
        return EnvParams.cat(envs)

    is_path = torch.tensor([t.name != HOLD_NAME for t in trajs])
    out = {}
    scales = [("nom", 1.0)] + ([("margin", 1.0 / float(cfg["autotune"]["margin_fraction"]))] if margin else [])
    for tag, scale in scales:
        res, rid, tid = evaluate(cfg, var, batch(scale), trajs, device, max_batch)
        cand = rid // n_robots  # robot i del candidato c esta en la fila c*n_robots + i
        out[f"unstable_{tag}"] = torch.zeros(p).index_add_(0, cand, res["unstable"].float())
        if tag == "nom":
            cost = run_cost(cfg, res)
            path = is_path[tid]
            q = float(cfg["autotune"]["cost"]["quantile"])
            out["cost_q"] = torch.stack([torch.quantile(cost[(cand == c) & path], q) for c in range(p)])
            out["cost_mean"] = torch.stack([cost[(cand == c) & path].mean() for c in range(p)])
            out["abort"] = torch.zeros(p).index_add_(0, cand, res["abort"].float())
            out["_res"], out["_cand"], out["_tid"] = res, cand, tid
    out["feasible"] = (out["unstable_nom"] == 0) & (out.get("unstable_margin", torch.zeros(p)) == 0)
    return out


def _rank_key(s: dict) -> torch.Tensor:
    """Validos primero (por costo); invalidos despues (por cantidad de inestables)."""
    bad = s["unstable_nom"] + s.get("unstable_margin", torch.zeros_like(s["unstable_nom"]))
    return torch.where(s["feasible"], s["cost_q"], 1e6 + bad)


def cem(cfg: dict, var: dict, device="cpu", log=print, max_batch: int = 200_000) -> dict:
    a = cfg["autotune"]
    trajs = all_trajectories(cfg)
    g = torch.Generator().manual_seed(int(a["seed"]))
    lo, hi = (math.log(float(b)) for b in a["bounds_kp"])
    mu = torch.log(torch.tensor(a["init_kp"], dtype=torch.float64))
    sd = torch.full((2,), float(a["init_log_std"]), dtype=torch.float64)
    n_elite = max(2, int(round(float(a["elite_fraction"]) * int(a["population"]))))
    history = []
    for it in range(int(a["generations"])):
        t0 = time.time()
        z = mu + sd * torch.randn(int(a["population"]), 2, generator=g, dtype=torch.float64)
        z[0] = mu  # siempre evalua la media actual
        kps = torch.exp(z.clamp(lo, hi)).float()
        s = score_candidates(cfg, var, kps, int(a["robots_per_candidate"]), int(a["seed"]) + 1000 + it,
                             device, trajs, True, max_batch)
        order = torch.argsort(_rank_key(s))
        elite = torch.log(kps[order[:n_elite]].double())
        alpha = float(a["smoothing"])
        mu = alpha * elite.mean(0) + (1 - alpha) * mu
        new_sd = torch.clamp(alpha * elite.std(0) + (1 - alpha) * sd, min=float(a["min_log_std"]))
        # Sin suficientes candidatos validos la elite no es confiable: no reducir la dispersion
        # (si no, la busqueda se congela dentro de la zona inestable).
        sd = new_sd if int(s["feasible"].sum()) >= n_elite else torch.maximum(new_sd, sd)
        for i in range(kps.shape[0]):
            history.append({"generation": it, "kp_t": float(kps[i, 0]), "kp_r": float(kps[i, 1]),
                            "feasible": int(s["feasible"][i]), "cost_q": float(s["cost_q"][i]),
                            "cost_mean": float(s["cost_mean"][i]), "unstable_nom": int(s["unstable_nom"][i]),
                            "unstable_margin": int(s["unstable_margin"][i])})
        b = int(order[0])
        log(f"[autotune] {var['name']} gen {it:2d} | media kP=({math.exp(mu[0]):5.2f}, {math.exp(mu[1]):5.2f}) "
            f"sd_log=({sd[0]:.2f}, {sd[1]:.2f}) | mejor ({kps[b, 0]:.2f}, {kps[b, 1]:.2f}) "
            f"p95={s['cost_q'][b]:.3f} m valido={bool(s['feasible'][b])} | validos {int(s['feasible'].sum())}/"
            f"{kps.shape[0]} | {time.time() - t0:.1f} s")
    return {"mu": torch.exp(mu).float(), "history": history, "trajs": trajs}


def finalize(cfg: dict, var: dict, search: dict, device="cpu", log=print, max_batch: int = 200_000) -> dict:
    """Re-evalua finalistas juntos, elige el mejor valido y lo valida a fondo."""
    a, v = cfg["autotune"], cfg["autotune"]["validation"]
    trajs = search["trajs"]
    feas = [h for h in search["history"] if h["feasible"]]
    feas.sort(key=lambda h: h["cost_q"])
    finals = [search["mu"].tolist()]
    for h in feas:
        cand = [round(h["kp_t"], 2), round(h["kp_r"], 2)]
        if all(abs(cand[0] - f[0]) > 0.05 or abs(cand[1] - f[1]) > 0.05 for f in finals):
            finals.append(cand)
        if len(finals) >= int(v["finalists"]):
            break
    kps = torch.tensor(finals, dtype=torch.float32)
    s = score_candidates(cfg, var, kps, int(v["finalist_robots"]), int(a["seed"]) + 50_000, device, trajs,
                         True, max_batch)
    order = torch.argsort(_rank_key(s))
    best = kps[int(order[0])]
    if not bool(s["feasible"][int(order[0])]):
        log("[autotune] AVISO: ningun finalista fue valido con todos los robots; se reporta el menos malo")
    finalists = [{"kp_t": float(kps[i, 0]), "kp_r": float(kps[i, 1]), "cost_q": float(s["cost_q"][i]),
                  "feasible": bool(s["feasible"][i]), "unstable_nom": int(s["unstable_nom"][i]),
                  "unstable_margin": int(s["unstable_margin"][i])} for i in order.tolist()]
    log(f"[autotune] {var['name']} elegido por la busqueda kP=({best[0]:.2f}, {best[1]:.2f})")

    # Retroceso robusto: con los robots de validacion, bajar 5% mientras haya ALGUN inestable
    # (con kP o con kP/margin). La busqueda usa pocos robots y tiende a quedar en la frontera.
    searched = best.clone()
    backoffs = 0
    for _ in range(int(a.get("max_backoffs", 15))):
        chk = score_candidates(cfg, var, best.unsqueeze(0), int(v["robots"]), int(a["seed"]) + 90_000, device,
                               trajs, True, max_batch)
        bad = int(chk["unstable_nom"][0]) + int(chk["unstable_margin"][0])
        if bad == 0:
            break
        log(f"[autotune] retroceso: kP=({best[0]:.2f}, {best[1]:.2f}) tiene {bad} corridas inestables "
            f"con {v['robots']} robots -> -5%")
        best = best * 0.95
        backoffs += 1
    log(f"[autotune] {var['name']} final kP=({best[0]:.2f}, {best[1]:.2f}) tras {backoffs} retrocesos")

    # Validacion: el elegido (redondeado a 0.5 hacia abajo para el codigo) + comparaciones.
    rounded = torch.floor(best * 2) / 2
    compare = [("autotune", best), ("autotune redondeado", rounded)] + [
        (f"comparacion ({c[0]:g}, {c[1]:g})", torch.tensor(c, dtype=torch.float32)) for c in v["compare"]]
    kv = torch.stack([c[1] for c in compare])
    t0 = time.time()
    # Robots distintos a los del retroceso: la validacion es fuera de muestra.
    sv = score_candidates(cfg, var, kv, int(v["robots"]), int(a["seed"]) + 95_000, device, trajs, True, max_batch)
    log(f"[autotune] validacion: {len(compare)} candidatos x {v['robots']} robots x {len(trajs)} pruebas x 2 "
        f"en {time.time() - t0:.1f} s")
    validation = []
    res, cand, tid = sv["_res"], sv["_cand"], sv["_tid"]
    is_path = torch.tensor([t.name != HOLD_NAME for t in trajs])[tid]
    for i, (name, k) in enumerate(compare):
        m = (cand == i) & is_path
        row = {"nombre": name, "kp_t": float(k[0]), "kp_r": float(k[1]), "cost_q": float(sv["cost_q"][i]),
               "cost_mean": float(sv["cost_mean"][i]), "unstable_nom": int(sv["unstable_nom"][i]),
               "unstable_margin": int(sv["unstable_margin"][i]), "abort": int(sv["abort"][i]),
               "n_runs": int(((cand == i)).sum())}
        for key in ("lateral_rms_m", "final_pos_err_m", "final_yaw_err_deg", "overshoot_m", "slip_frac"):
            x = res[key][m]
            row[key] = (float(x.median()), float(torch.quantile(x, 0.95)), float(x.max()))
        per = {}
        for j, tr in enumerate(trajs):
            mm = (cand == i) & (tid == j)
            per[tr.name] = {k2: float(torch.quantile(res[k2][mm], 0.95)) for k2 in
                            ("pos_rms_m", "final_pos_err_m", "final_yaw_err_deg")}
            per[tr.name]["unstable"] = int(res["unstable"][mm].sum())
        row["per_traj"] = per
        validation.append(row)

    # Estres: fuera del rango aleatorizado, con el elegido redondeado.
    stress = []
    for name, over in a.get("stress", {}).items():
        c2 = copy.deepcopy(cfg)
        c2["randomization"].update(over)
        ss = score_candidates(c2, var, rounded.unsqueeze(0), int(v["robots"]) // 4, int(a["seed"]) + 70_000,
                              device, trajs, False, max_batch)
        stress.append({"escenario": name, "cost_q": float(ss["cost_q"][0]),
                       "unstable": int(ss["unstable_nom"][0]), "abort": int(ss["abort"][0]),
                       "n_runs": (int(v["robots"]) // 4) * len(trajs)})
        log(f"[autotune] estres {name}: p95={stress[-1]['cost_q']:.3f} m inestables {stress[-1]['unstable']} "
            f"abortos {stress[-1]['abort']}")
    return {"best": best, "searched": searched, "backoffs": backoffs, "rounded": rounded, "finalists": finalists,
            "validation": validation, "stress": stress}


def _plots(name: str, history: list[dict], best, out_dir: Path) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from .report import GRID, INK, INK_2, SERIES, SURFACE, _style

    files = []
    fig, ax = plt.subplots(figsize=(7.5, 4.8), dpi=130)
    _style(ax, fig)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ok = [h for h in history if h["feasible"]]
    bad = [h for h in history if not h["feasible"]]
    ax.scatter([h["kp_t"] for h in bad], [h["kp_r"] for h in bad], s=14, color=SERIES[1], alpha=0.55,
               label="invalido (inestable con kP o kP/0.65)", edgecolors="none")
    sc = ax.scatter([h["kp_t"] for h in ok], [h["kp_r"] for h in ok], s=18, c=[h["cost_q"] for h in ok],
                    cmap="Blues_r", label="valido (color = costo p95)", edgecolors="none")
    if ok:
        cb = fig.colorbar(sc, ax=ax, pad=0.02)
        cb.set_label("costo p95 [m] (menor es mejor)", color=INK_2, fontsize=9)
        cb.ax.tick_params(colors=INK_2, labelsize=8)
        cb.outline.set_visible(False)
    ax.plot([float(best[0])], [float(best[1])], marker="*", markersize=17, color=SERIES[2],
            markeredgecolor=SURFACE, markeredgewidth=1.5, linestyle="none", label="elegido")
    ax.annotate(f"({float(best[0]):.2f}, {float(best[1]):.2f})", (float(best[0]), float(best[1])),
                textcoords="offset points", xytext=(10, 8), color=INK, fontsize=9)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("kP traslacion")
    ax.set_ylabel("kP rotacion")
    ax.set_title(f"Candidatos evaluados por el autotuner - {name}", fontsize=11, loc="left")
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, loc="lower left")
    fig.tight_layout()
    f = out_dir / f"autotune_candidatos_{name}.png"
    fig.savefig(f, facecolor=SURFACE)
    plt.close(fig)
    files.append(f.name)

    gens = sorted({h["generation"] for h in history})
    fig, ax = plt.subplots(figsize=(7.5, 4.2), dpi=130)
    _style(ax, fig)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    best_q = []
    for g_ in gens:
        q = [h["cost_q"] for h in history if h["generation"] == g_ and h["feasible"]]
        best_q.append(min(q) if q else float("nan"))
    ax.plot(gens, best_q, color=SERIES[0], linewidth=2, marker="o", markersize=4)
    ax.set_xlabel("generacion")
    ax.set_ylabel("mejor costo p95 valido [m]")
    ax.set_title(f"Convergencia del autotuner - {name}", fontsize=11, loc="left")
    fig.tight_layout()
    f = out_dir / f"autotune_convergencia_{name}.png"
    fig.savefig(f, facecolor=SURFACE)
    plt.close(fig)
    files.append(f.name)
    return files


def write_autotune_report(cfg: dict, results: dict, out_dir: Path, device: str, elapsed: float) -> Path:
    a = cfg["autotune"]
    L = ["# Autotuning de kP de PathPlanner\n"]
    L.append(f"Device `{device}`, {elapsed / 60:.1f} min en total. CEM: {a['generations']} generaciones x "
             f"{a['population']} candidatos x {a['robots_per_candidate']} robots, todas las trayectorias + "
             f"mantener_pose, cada una con kP y con kP/{a['margin_fraction']}.\n")
    L.append("> Punto de partida, no valor final: confirmar en el robot real.\n")
    L.append("## Resultado\n")
    L.append("| variante | kP_t | kP_r | para el codigo (redondeado) | costo p95 [m] | inestables (kP) | "
             "inestables (kP/0.65) | corridas |")
    L.append("|---|---|---|---|---|---|---|---|")
    for name, r in results.items():
        v0 = r["validation"][1]
        L.append(f"| {name} | {r['best'][0]:.2f} | {r['best'][1]:.2f} | **({r['rounded'][0]:g}, "
                 f"{r['rounded'][1]:g})** | {v0['cost_q']:.3f} | {v0['unstable_nom']} | {v0['unstable_margin']} | "
                 f"{v0['n_runs']} |")
    L.append("")
    for name, r in results.items():
        if r["backoffs"]:
            L.append(f"- {name}: la busqueda llego a ({r['searched'][0]:.2f}, {r['searched'][1]:.2f}), pero con "
                     f"{a['validation']['robots']} robots nuevos aparecieron casos inestables; se bajo 5% "
                     f"{r['backoffs']} vez/veces hasta que ninguno fallo.")
    L.append("")
    L.append("Costo por corrida = error RMS de posicion + error final + (lo mismo en heading, 1 grado = 1 cm) "
             f"+ {a['cost']['abort_penalty_m']} m si aborta; se reporta el percentil "
             f"{int(float(a['cost']['quantile']) * 100)} sobre robots y trayectorias.\n")
    for name, r in results.items():
        L.append(f"## {name}\n")
        L.append("### Validacion con robots nuevos\n")
        L.append("| candidato | kP | costo p95 / medio [m] | lateral RMS med / p95 / max [m] | "
                 "error final pos med / p95 / max [m] | heading final p95 [deg] | inest. kP | inest. kP/0.65 | abortos |")
        L.append("|---|---|---|---|---|---|---|---|---|")
        for v in r["validation"]:
            lr, fp, fy = v["lateral_rms_m"], v["final_pos_err_m"], v["final_yaw_err_deg"]
            L.append(f"| {v['nombre']} | ({v['kp_t']:.2f}, {v['kp_r']:.2f}) | {v['cost_q']:.3f} / {v['cost_mean']:.3f} | "
                     f"{lr[0]:.3f} / {lr[1]:.3f} / {lr[2]:.3f} | {fp[0]:.3f} / {fp[1]:.3f} / {fp[2]:.3f} | "
                     f"{fy[1]:.1f} | {v['unstable_nom']} | {v['unstable_margin']} | {v['abort']} |")
        L.append("")
        L.append(f"### Por trayectoria (p95) con el valor redondeado ({r['rounded'][0]:g}, {r['rounded'][1]:g})\n")
        L.append("| trayectoria | error RMS pos [m] | error final pos [m] | error final heading [deg] | inestables |")
        L.append("|---|---|---|---|---|")
        for tn, p in r["validation"][1]["per_traj"].items():
            L.append(f"| {tn} | {p['pos_rms_m']:.3f} | {p['final_pos_err_m']:.3f} | {p['final_yaw_err_deg']:.1f} | "
                     f"{p['unstable']} |")
        L.append("")
        L.append("### Estres (fuera del rango aleatorizado, valor redondeado)\n")
        L.append("| escenario | costo p95 [m] | inestables | abortos | corridas |")
        L.append("|---|---|---|---|---|")
        for s in r["stress"]:
            L.append(f"| {s['escenario']} | {s['cost_q']:.3f} | {s['unstable']} | {s['abort']} | {s['n_runs']} |")
        L.append("")
        L.append("### Finalistas re-evaluados juntos\n")
        L.append("| kP | costo p95 [m] | valido | inest. kP | inest. kP/0.65 |")
        L.append("|---|---|---|---|---|")
        for f in r["finalists"]:
            L.append(f"| ({f['kp_t']:.2f}, {f['kp_r']:.2f}) | {f['cost_q']:.3f} | {'si' if f['feasible'] else 'no'} | "
                     f"{f['unstable_nom']} | {f['unstable_margin']} |")
        L.append("")
        for p in r.get("plots", []):
            L.append(f"![{p}]({p})")
        L.append("")
    path = out_dir / "autotune.md"
    path.write_text("\n".join(L), encoding="utf-8")
    return path


def quick_autotune_config(cfg: dict) -> dict:
    q = copy.deepcopy(cfg)
    a = q["autotune"]
    a.update(population=8, robots_per_candidate=6, generations=3)
    a["validation"].update(robots=64, finalists=3, finalist_robots=16)
    a["stress"] = dict(list(a["stress"].items())[:1])
    q["sweep"]["trajectories"] = ["recta_4m", "giro_sitio_180"]
    return q


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=str(PACKAGE_DIR / "results_autotune"))
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--variant", action="append", help="nombre de variante (repetible); default todas")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--max-batch", type=int, default=200_000)
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    if args.quick:
        cfg = quick_autotune_config(cfg)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    names = args.variant or [v["name"] for v in cfg["variants"]]
    t0 = time.time()
    results = {}
    for name in names:
        var = variant(cfg, name)
        search = cem(cfg, var, args.device, max_batch=args.max_batch)
        r = finalize(cfg, var, search, args.device, max_batch=args.max_batch)
        with open(out / f"autotune_history_{name}.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(search["history"][0].keys()))
            w.writeheader()
            w.writerows(search["history"])
        try:
            r["plots"] = _plots(name, search["history"], r["best"], out)
        except ImportError:
            r["plots"] = []
        results[name] = r
    path = write_autotune_report(cfg, results, out, args.device, time.time() - t0)
    print(f"[autotune] {path}")


if __name__ == "__main__":
    main()

"""Analisis de sweep_results: limites de estabilidad, recomendacion, summary.md y graficas PNG."""

from __future__ import annotations

import datetime as _dt
import math
from pathlib import Path

import numpy as np

from .trajectory import HOLD_NAME

# Paleta (validada con el script de dataviz): series categoricas en orden fijo, rampa azul secuencial.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SEQ_BLUE = ["#f0efec", "#cde2fb", "#86b6ef", "#2a78d6", "#1c5cab", "#0d366b"]


def _arr(rows, key, dtype=float):
    return np.array([r[key] for r in rows], dtype=dtype)


def _round_half(x: float) -> float:
    return math.floor(x * 2) / 2


def analyze_variant(cfg: dict, rows: list[dict]) -> dict:
    frac = float(cfg["recommendation"]["fraction_of_limit"])
    kt, kr = _arr(rows, "kp_t"), _arr(rows, "kp_r")
    env_id = _arr(rows, "env_id", int)
    traj = np.array([r["trajectory"] for r in rows])
    ut, ur = _arr(rows, "unstable_t", bool), _arr(rows, "unstable_r", bool)

    n_env = env_id.max() + 1
    env_kt, env_kr = np.zeros(n_env), np.zeros(n_env)
    env_kt[env_id], env_kr[env_id] = kt, kr
    env_ut, env_ur = np.zeros(n_env, bool), np.zeros(n_env, bool)
    np.logical_or.at(env_ut, env_id, ut)
    np.logical_or.at(env_ur, env_id, ur)
    both = env_ut & env_ur

    def first(mask, kvals, flag):
        if not mask.any():
            return None, None
        lim = float(kvals[mask].min())
        cand = np.nonzero(mask & (kvals == lim))[0]
        # Detalle del primer caso: el robot y la trayectoria que lo provocaron.
        i = int(cand[0])
        r0 = next(r for r in rows if r["env_id"] == i and r[flag])
        detail = {k: r0[k] for k in ("trajectory", "latency_s", "cof", "speed_factor", "moi", "cg_height", "kp_t", "kp_r")}
        detail["n_casos_en_el_limite"] = int(len(cand))
        return lim, detail

    # Limites por eje. Una inestabilidad cuenta para un eje solo si la ganancia del OTRO eje esta
    # por debajo de su propio limite (si no, la causa puede ser el otro eje). Se itera hasta que
    # los dos limites dejan de cambiar; el punto de partida usa solo casos "puros" de un eje.
    lim_t, det_t = first(env_ut & ~env_ur, env_kt, "unstable_t")
    lim_r, det_r = first(env_ur & ~env_ut, env_kr, "unstable_r")
    for _ in range(10):
        below_r = env_kr < (lim_r if lim_r is not None else np.inf)
        new_t, new_det_t = first(env_ut & below_r, env_kt, "unstable_t")
        below_t = env_kt < (new_t if new_t is not None else np.inf)
        new_r, new_det_r = first(env_ur & below_t, env_kr, "unstable_r")
        if (new_t, new_r) == (lim_t, lim_r):
            break
        lim_t, det_t, lim_r, det_r = new_t, new_det_t, new_r, new_det_r

    grid_t, grid_r = np.unique(env_kt), np.unique(env_kr)
    rec_t = _round_half(frac * lim_t) if lim_t else None
    rec_r = _round_half(frac * lim_r) if lim_r else None

    def cell_value(grid, rec):
        if rec is None:
            return float(grid.max())
        below = grid[grid <= rec + 1e-9]
        return float(below.max()) if below.size else float(grid.min())

    cell_t, cell_r = cell_value(grid_t, rec_t), cell_value(grid_r, rec_r)

    path = traj != HOLD_NAME
    def stats(mask):
        out = {}
        for k in ("lateral_rms_m", "pos_rms_m", "max_pos_err_m", "final_pos_err_m", "final_yaw_err_deg",
                  "max_yaw_err_deg", "overshoot_m", "overshoot_deg", "total_time_s", "slip_frac"):
            v = _arr([r for r, m in zip(rows, mask) if m], k)
            v = v[np.isfinite(v)]
            out[k] = (float(np.median(v)), float(v.max())) if v.size else (float("nan"), float("nan"))
        sub = [r for r, m in zip(rows, mask) if m]
        out["n"] = len(sub)
        out["abort"] = int(sum(r["abort"] for r in sub))
        out["unstable"] = int(sum(r["unstable"] for r in sub))
        out["sin_detenerse"] = int(sum(1 for r in sub if not math.isfinite(r["total_time_s"])))
        return out

    at_cell = (kt == cell_t) & (kr == cell_r)
    cell_stats = stats(at_cell & path)
    per_traj = {name: stats(at_cell & (traj == name)) for name in sorted(set(traj[path]))}

    # Curvas: peor caso y mediana contra kP_t (con kP_r de la celda) y contra kP_r (con kP_t de la celda).
    pos_rms, fyaw, myaw = _arr(rows, "pos_rms_m"), _arr(rows, "final_yaw_err_deg"), _arr(rows, "max_yaw_err_deg")
    curve_t = []
    for g in grid_t:
        m = (kt == g) & (kr == cell_r) & path
        curve_t.append((float(g), float(np.median(pos_rms[m])), float(np.nanmax(pos_rms[m]))))
    curve_r = []
    for g in grid_r:
        m = (kr == g) & (kt == cell_t) & path
        curve_r.append((float(g), float(np.median(myaw[m])), float(np.nanmax(myaw[m]))))

    # Mapa: fraccion de robots inestables por celda.
    heat = np.zeros((grid_r.size, grid_t.size))
    for j, gr in enumerate(grid_r):
        for i, gt in enumerate(grid_t):
            m = (env_kt == gt) & (env_kr == gr)
            heat[j, i] = (env_ut[m] | env_ur[m]).mean()

    # Celda robusta con mejor peor-caso de pos_rms (informativo), solo por debajo de ambos limites.
    best = None
    for gt in grid_t:
        for gr in grid_r:
            if (lim_t is not None and gt >= lim_t) or (lim_r is not None and gr >= lim_r):
                continue
            em = (env_kt == gt) & (env_kr == gr)
            if (env_ut[em] | env_ur[em]).any():
                continue
            m = (kt == gt) & (kr == gr) & path
            w = float(np.nanmax(pos_rms[m]))
            if best is None or w < best[2]:
                best = (float(gt), float(gr), w)

    return {
        "lim_t": lim_t, "lim_r": lim_r, "det_t": det_t, "det_r": det_r,
        "rec_t": rec_t, "rec_r": rec_r, "cell_t": cell_t, "cell_r": cell_r,
        "n_both": int(both.sum()), "n_env": int(n_env),
        "cell_stats": cell_stats, "per_traj": per_traj,
        "curve_t": curve_t, "curve_r": curve_r,
        "heat": heat, "grid_t": grid_t, "grid_r": grid_r, "best": best,
        "max_t": float(grid_t.max()), "max_r": float(grid_r.max()),
    }


def _style(ax, fig):
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)
    ax.title.set_color(INK)


def _plots(results: dict, out_dir: Path) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    files = []
    cmap = LinearSegmentedColormap.from_list("seq_blue", SEQ_BLUE)
    for name, a in results.items():
        fig, ax = plt.subplots(figsize=(7.5, 4.8), dpi=130)
        _style(ax, fig)
        gt, gr = a["grid_t"], a["grid_r"]
        dt = (gt[1] - gt[0]) / 2 if gt.size > 1 else 0.5
        dr = (gr[1] - gr[0]) / 2 if gr.size > 1 else 0.5
        im = ax.imshow(a["heat"], origin="lower", aspect="auto", cmap=cmap, vmin=0, vmax=1,
                       extent=(gt[0] - dt, gt[-1] + dt, gr[0] - dr, gr[-1] + dr), interpolation="nearest")
        cb = fig.colorbar(im, ax=ax, pad=0.02)
        cb.set_label("fraccion de robots inestables", color=INK_2, fontsize=9)
        cb.ax.tick_params(colors=INK_2, labelsize=8)
        cb.outline.set_visible(False)
        if a["rec_t"] is not None and a["rec_r"] is not None:
            ax.plot([a["rec_t"]], [a["rec_r"]], marker="*", markersize=16, color=SERIES[1],
                    markeredgecolor=SURFACE, markeredgewidth=1.5, linestyle="none")
            ax.annotate(f"recomendado ({a['rec_t']:g}, {a['rec_r']:g})", (a["rec_t"], a["rec_r"]),
                        textcoords="offset points", xytext=(10, 8), color=INK, fontsize=9)
        ax.set_xlabel("kP traslacion")
        ax.set_ylabel("kP rotacion")
        ax.set_title(f"Estabilidad en todo el rango aleatorizado - {name}", fontsize=11, loc="left")
        f = out_dir / f"estabilidad_{name}.png"
        fig.tight_layout()
        fig.savefig(f, facecolor=SURFACE)
        plt.close(fig)
        files.append(f.name)

    for key, xlabel, ylabel, title, fname in (
        ("curve_t", "kP traslacion (kP rotacion fijo en su celda recomendada)", "error de posicion RMS [m]",
         "Peor caso del error de posicion contra kP de traslacion", "error_vs_kp_traslacion.png"),
        ("curve_r", "kP rotacion (kP traslacion fijo en su celda recomendada)", "error maximo de heading [grados]",
         "Peor caso del error de heading contra kP de rotacion", "error_vs_kp_rotacion.png"),
    ):
        fig, ax = plt.subplots(figsize=(7.5, 4.5), dpi=130)
        _style(ax, fig)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for idx, (name, a) in enumerate(results.items()):
            c = SERIES[idx % len(SERIES)]
            xs = [p[0] for p in a[key]]
            worst = [p[2] for p in a[key]]
            med = [p[1] for p in a[key]]
            ax.plot(xs, worst, color=c, linewidth=2, marker="o", markersize=4, label=f"{name} (peor caso)")
            ax.plot(xs, med, color=c, linewidth=1.2, linestyle="--", label=f"{name} (mediana)")
            ax.annotate(name, (xs[-1], worst[-1]), textcoords="offset points", xytext=(4, 0),
                        color=INK_2, fontsize=8, va="center")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11, loc="left")
        ax.legend(frameon=False, fontsize=8, labelcolor=INK_2)
        fig.tight_layout()
        f = out_dir / fname
        fig.savefig(f, facecolor=SURFACE)
        plt.close(fig)
        files.append(f.name)
    return files


def _fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, float) and not math.isfinite(v):
        return "nan"
    return f"{v:.{nd}f}"


def write_report(cfg: dict, rows: list[dict], out_dir: Path, backend: str = "torch", log=print) -> None:
    variants = [v["name"] for v in cfg["variants"]]
    results = {v: analyze_variant(cfg, [r for r in rows if r["variant"] == v]) for v in variants}
    try:
        pngs = _plots(results, out_dir)
    except ImportError as e:  # matplotlib no instalado
        log(f"[report] sin graficas: {e}")
        pngs = []

    frac = cfg["recommendation"]["fraction_of_limit"]
    rnd = cfg["randomization"]
    L = []
    L.append("# Sweep de PID para PathPlanner - resumen\n")
    L.append(f"Generado {_dt.datetime.now():%Y-%m-%d %H:%M} con backend `{backend}`. "
             f"{results[variants[0]]['n_env']} robots aleatorizados por variante, "
             f"trayectorias: {', '.join(sorted(set(r['trajectory'] for r in rows)))}.\n")
    L.append("> Estos kP son un **punto de partida**. El modelo es una simplificacion: hay que confirmarlos "
             "en el robot real, empezando por el valor recomendado y subiendo con cuidado.\n")

    L.append("## Recomendacion por variante\n")
    L.append(f"Regla: kP = {frac:.0%} del limite de inestabilidad **robusto** (el kP mas bajo que se volvio "
             "inestable en CUALQUIER robot del rango aleatorizado), redondeado hacia abajo a 0.5.\n")
    L.append("| variante | limite kP_t | kP_t recomendado | limite kP_r | kP_r recomendado | celda evaluada (t, r) |")
    L.append("|---|---|---|---|---|---|")
    for v, a in results.items():
        lt = _fmt(a["lim_t"], 1) if a["lim_t"] else f"> {a['max_t']:g} (no se encontro)"
        lr = _fmt(a["lim_r"], 1) if a["lim_r"] else f"> {a['max_r']:g} (no se encontro)"
        L.append(f"| {v} | {lt} | {_fmt(a['rec_t'], 1)} | {lr} | {_fmt(a['rec_r'], 1)} | "
                 f"({a['cell_t']:g}, {a['cell_r']:g}) |")
    L.append("")
    for v, a in results.items():
        if a["lim_t"] is None or a["lim_r"] is None:
            L.append(f"- **{v}**: no aparecio inestabilidad dentro del rango barrido en algun eje. Amplia "
                     "`sweep.kp_translation` / `sweep.kp_rotation` en params.yaml y vuelve a correr; "
                     "no extrapoles.")
        if a["n_both"]:
            L.append(f"- **{v}**: {a['n_both']} robots fueron inestables en traslacion Y rotacion a la vez "
                     "(acoplamiento). Cuentan para el limite de un eje solo si la ganancia del otro eje "
                     "esta por debajo de su propio limite.")
    L.append("")

    L.append("## Metricas en la celda recomendada (mediana / peor caso, sobre robots y trayectorias)\n")
    L.append("| variante | error lateral RMS [m] | error final pos [m] | error final heading [deg] | "
             "sobreimpulso [m] | tiempo total [s] | derrape | abortos | inestables |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for v, a in results.items():
        s = a["cell_stats"]
        L.append(f"| {v} | {_fmt(s['lateral_rms_m'][0])} / {_fmt(s['lateral_rms_m'][1])} | "
                 f"{_fmt(s['final_pos_err_m'][0])} / {_fmt(s['final_pos_err_m'][1])} | "
                 f"{_fmt(s['final_yaw_err_deg'][0], 1)} / {_fmt(s['final_yaw_err_deg'][1], 1)} | "
                 f"{_fmt(s['overshoot_m'][0])} / {_fmt(s['overshoot_m'][1])} | "
                 f"{_fmt(s['total_time_s'][0], 2)} / {_fmt(s['total_time_s'][1], 2)} | "
                 f"{s['slip_frac'][0]:.0%} / {s['slip_frac'][1]:.0%} | {s['abort']}/{s['n']} | {s['unstable']}/{s['n']} |")
    L.append("")
    a0 = results[variants[0]]
    L.append(f"### Por trayectoria - {variants[0]} en ({a0['cell_t']:g}, {a0['cell_r']:g})\n")
    L.append("| trayectoria | lateral RMS med/peor [m] | final pos med/peor [m] | final heading peor [deg] | abortos |")
    L.append("|---|---|---|---|---|")
    for name, s in a0["per_traj"].items():
        L.append(f"| {name} | {_fmt(s['lateral_rms_m'][0])} / {_fmt(s['lateral_rms_m'][1])} | "
                 f"{_fmt(s['final_pos_err_m'][0])} / {_fmt(s['final_pos_err_m'][1])} | "
                 f"{_fmt(s['final_yaw_err_deg'][1], 1)} | {s['abort']}/{s['n']} |")
    L.append("")

    L.append("## Primer caso inestable (que robot fijo el limite)\n")
    for v, a in results.items():
        for eje, d in (("traslacion", a["det_t"]), ("rotacion", a["det_r"])):
            if d:
                L.append(f"- {v}, {eje}: kP=({d['kp_t']:g}, {d['kp_r']:g}) en `{d['trajectory']}`, latencia "
                         f"{d['latency_s'] * 1000:.0f} ms, COF {d['cof']:.2f}, velocidad {d['speed_factor']:.0%}, "
                         f"MOI {d['moi']:.2f} ({d['n_casos_en_el_limite']} robots en ese kP)")
    L.append("")
    L.append("Si el limite lo fija casi siempre la latencia alta, medir la latencia real del robot es lo que mas "
             "mueve la recomendacion.\n")

    L.append("## Informativo: celda sin inestabilidad con mejor peor caso de error RMS\n")
    for v, a in results.items():
        if a["best"]:
            L.append(f"- {v}: kP=({a['best'][0]:g}, {a['best'][1]:g}), peor RMS de posicion {a['best'][2]:.3f} m")
    L.append("\nSi esta celda queda muy por debajo de la recomendada, kP mas alto no esta ayudando (la latencia "
             "o la traccion limitan antes que la ganancia).\n")

    if pngs:
        L.append("## Graficas\n")
        for p in pngs:
            L.append(f"![{p}]({p})")
        L.append("")

    L.append("## Como leer las metricas\n")
    L.append("- **error lateral RMS**: distancia perpendicular a la direccion de avance del path, durante el path.\n"
             "- **error final**: pose del centro del robot contra la pose final del path, despues de detenerse "
             "(PathPlanner manda velocidad 0 al terminar y ya no corrige).\n"
             "- **sobreimpulso**: cuanto se pasa del punto final en la direccion de llegada (m) o del heading final (deg).\n"
             "- **tiempo total**: hasta que el robot queda quieto (< 0.05 m/s); `nan` si no se detuvo en el tiempo simulado.\n"
             "- **derrape**: fraccion del tiempo del path con algun modulo en el limite de friccion.\n"
             "- **aborto**: error > 0.75 m o > 30 grados.\n"
             "- **inestable**: aborto, o en la prueba `mantener_pose` mas de "
             f"{cfg['stability']['hold_max_reversals']} inversiones del error o falta de convergencia.\n")

    L.append("## Rangos aleatorizados\n")
    L.append(f"masa extra {rnd['extra_mass_kg']} kg, MOI {rnd['moi_yaw_kgm2']} kg m2, altura CM {rnd['cg_height_m']} m, "
             f"COF {rnd['cof']}, velocidad real {rnd['speed_factor']} x {cfg['code']['speed_at_12v_mps']} m/s, "
             f"latencia {rnd['latency_s']} s, error inicial <= {rnd['init_pos_offset_m']} m / "
             f"{rnd['init_yaw_offset_deg']} deg.\n")

    L.append("## Supuestos y limitaciones del modelo\n")
    L.append(
        "- Cuerpo rigido plano; sin suspension, sin cabeceo/balanceo (la transferencia de carga es cuasi-estatica).\n"
        "- La pose medida es la real (sin ruido, sin deriva de odometria por derrape); la latencia es un retraso puro del comando.\n"
        "- Sin modelo de bateria: 12 V fijos. La caida de voltaje se representa solo via `speed_factor`.\n"
        "- `speed_factor` se modela bajando Kv del motor; en la realidad parte es friccion (kS) y bateria.\n"
        "- Steer: limite de velocidad + primer orden con valores supuestos (TODO medir).\n"
        "- Llanta: modelo lineal de angulo de deslizamiento + circulo de friccion; sin datos del Spike Grip.\n"
        "- Los feedforwards de fuerza de PathPlanner se ignoran (igual que el codigo actual del robot).\n"
        "- Trayectorias sinteticas, no las de PathPlanner (no hay paths en el proyecto). PathPlanner limita "
        "con un modelo de fuerzas por modulo; aqui se limita aceleracion tangencial y centripeta por separado.\n"
        "- kI = kD = 0, como pediste.\n")
    (out_dir / "summary.md").write_text("\n".join(L), encoding="utf-8")
    log(f"[report] {out_dir / 'summary.md'}" + (f" + {len(pngs)} PNG" if pngs else ""))


def main(argv=None):
    """Regenera summary.md y las graficas desde un sweep_results.csv existente (sin simular)."""
    import argparse
    import csv

    from .params import load_config

    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("csv", help="ruta a sweep_results.csv")
    ap.add_argument("--config", default=None, help="params.yaml (para fraccion y umbrales)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    text = {"variant", "trajectory"}
    ints = {"env_id", "abort", "unstable_t", "unstable_r", "unstable", "wheel_lift"}
    rows = []
    with open(args.csv, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k in text else int(v) if k in ints else float(v)) for k, v in r.items()})
    present = {r["variant"] for r in rows}
    cfg["variants"] = [v for v in cfg["variants"] if v["name"] in present]
    write_report(cfg, rows, Path(args.csv).parent, backend="csv")


if __name__ == "__main__":
    main()

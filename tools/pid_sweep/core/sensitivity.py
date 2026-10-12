"""Sensibilidad: cuanto margen de ganancia le queda a un par de kP cuando cambia cada parametro.

Para cada parametro y cada valor: se fija ese valor (el resto sigue aleatorizado), se corren
`robots` robots en todas las trayectorias + mantener_pose con los kP multiplicados por cada
factor de `multipliers`, y se reporta:
  - margen de ganancia = mayor multiplicador estable en TODOS los robots
    (>= 1/0.65 = 1.54 cumple la regla del 65%),
  - costo p95 con los kP sin multiplicar.
Dice que dato del robot conviene medir primero: el que mas mueve el margen.

Uso (desde tools/pid_sweep):
  python -m core.sensitivity --kp 8.0,10.8 --device cuda
"""

from __future__ import annotations

import argparse
import copy
import time
from pathlib import Path

import torch

from .autotune import run_cost
from .params import PACKAGE_DIR, EnvParams, load_config, sample_env_params, variant
from .sweep import all_trajectories, evaluate
from .trajectory import HOLD_NAME

MULTIPLIERS = [1.0, 1.25, 1.54, 1.75, 2.0, 2.5, 3.0]

# (nombre, como aplicarlo, valores). "rand" fija un rango de randomization; "cfg" cambia una constante.
PARAMS = [
    ("Latencia [s]", ("rand", "latency_s"), [0.02, 0.04, 0.06, 0.08]),
    ("COF", ("rand", "cof"), [0.7, 0.9, 1.2, 1.6]),
    ("Velocidad real [x 5.85 m/s]", ("rand", "speed_factor"), [0.75, 0.85, 0.95, 1.0]),
    ("Altura del CM [m]", ("rand", "cg_height_m"), [0.15, 0.20, 0.25, 0.30]),
    ("MOI yaw [kg m2]", ("rand", "moi_yaw_kgm2"), [6.0, 7.0, 8.0]),
    ("Masa extra [kg]", ("rand", "extra_mass_kg"), [0.0, 4.0, 8.0]),
    ("Friccion real del modulo [V] (kS del codigo = 0.408)", ("cfg", ("drive_motor", "static_friction_v")), [0.1, 0.25, 0.408, 0.6]),
    ("Velocidad max. de steer [rad/s]", ("cfg", ("steer", "max_rate_rad_s")), [10.0, 20.0, 30.0]),
    ("Constante de tiempo de steer [s]", ("cfg", ("steer", "time_constant_s")), [0.005, 0.015, 0.03]),
    ("Rigidez lateral de llanta [1/rad]", ("cfg", ("tire", "cornering_stiffness_per_load")), [8.0, 15.0, 30.0]),
]


def apply(cfg: dict, how, value) -> dict:
    c = copy.deepcopy(cfg)
    kind, key = how
    if kind == "rand":
        c["randomization"][key] = [value, value]
    else:
        c[key[0]][key[1]] = value
    return c


def margin_for(cfg: dict, var: dict, kp, n_robots: int, seed: int, device: str, trajs) -> dict:
    g = torch.Generator().manual_seed(seed)
    base = sample_env_params(cfg, torch.zeros(n_robots), torch.zeros(n_robots), g)
    envs = [base.replace(kp_t=torch.full((n_robots,), kp[0] * m), kp_r=torch.full((n_robots,), kp[1] * m))
            for m in MULTIPLIERS]
    res, rid, tid = evaluate(cfg, var, EnvParams.cat(envs), trajs, device)
    which = rid // n_robots
    unstable = [int(res["unstable"][which == i].sum()) for i in range(len(MULTIPLIERS))]
    ok = 0.0
    for m, u in zip(MULTIPLIERS, unstable):
        if u:
            break
        ok = m
    path = torch.tensor([t.name != HOLD_NAME for t in trajs])[tid]
    cost = run_cost(cfg, res)[(which == 0) & path]
    aborts = int(res["abort"][(which == 0) & path].sum())
    return {"margin": ok, "unstable": unstable, "cost_q": float(torch.quantile(cost, 0.95)), "aborts": aborts,
            "n_runs": int(((which == 0) & path).sum())}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--kp", required=True, help="kP_t,kP_r a evaluar")
    ap.add_argument("--variant", default="actual_openloop")
    ap.add_argument("--robots", type=int, default=512)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", default=str(PACKAGE_DIR / "results_autotune"))
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    kp = [float(v) for v in args.kp.split(",")]
    var = variant(cfg, args.variant)
    trajs = all_trajectories(cfg)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    rows = []
    base = margin_for(cfg, var, kp, args.robots, 4242, args.device, trajs)
    print(f"[sens] nominal: margen x{base['margin']:g} p95 {base['cost_q']:.3f} m")
    for name, how, values in PARAMS:
        for v in values:
            r = margin_for(apply(cfg, how, v), var, kp, args.robots, 4242, args.device, trajs)
            rows.append({"param": name, "value": v, **r})
            print(f"[sens] {name} = {v:g}: margen x{r['margin']:g}  p95 {r['cost_q']:.3f} m  abortos {r['aborts']}/{r['n_runs']}"
                  f"  inestables por multiplicador {r['unstable']}")
    _write(cfg, args, kp, base, rows, out, time.time() - t0)


def _write(cfg, args, kp, base, rows, out: Path, elapsed: float):
    need = 1 / float(cfg["autotune"]["margin_fraction"])
    L = [f"# Sensibilidad de kP = ({kp[0]:g}, {kp[1]:g}) - {args.variant}\n",
         f"{args.robots} robots por punto x {len(MULTIPLIERS)} multiplicadores x 9 pruebas; {elapsed / 60:.1f} min "
         f"en `{args.device}`. Margen = mayor multiplicador de kP estable en TODOS los robots "
         f"(probados: {', '.join(f'{m:g}' for m in MULTIPLIERS)}). La regla del 65% pide >= {need:.2f}.\n",
         f"Nominal (todo aleatorizado en el rango normal): margen x{base['margin']:g}, costo p95 {base['cost_q']:.3f} m.\n",
         "| parametro | valor | margen de ganancia | cumple 65% | costo p95 [m] | abortos |", "|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['param']} | {r['value']:g} | x{r['margin']:g} | {'si' if r['margin'] >= need - 1e-6 else '**no**'} | "
                 f"{r['cost_q']:.3f} | {r['aborts']}/{r['n_runs']} |")
    L.append("")
    spans = {}
    for r in rows:
        spans.setdefault(r["param"], []).append(r["margin"])
    L.append("## Que medir primero (lo que mas mueve el margen)\n")
    for p, ms in sorted(spans.items(), key=lambda kv: -(max(kv[1]) - min(kv[1]))):
        L.append(f"- {p}: margen entre x{min(ms):g} y x{max(ms):g}")
    try:
        L.append("\n" + _plot(rows, need, out))
    except ImportError:
        pass
    path = out / f"sensibilidad_{args.variant}.md"
    path.write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"[sens] {path}")


def _plot(rows, need, out: Path) -> str:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from .report import GRID, INK, INK_2, SERIES, SURFACE, _style

    params = list(dict.fromkeys(r["param"] for r in rows))
    fig, ax = plt.subplots(figsize=(8.5, 0.55 * len(params) + 1.6), dpi=130)
    _style(ax, fig)
    ax.grid(True, axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for i, p in enumerate(params):
        pts = [(r["value"], r["margin"]) for r in rows if r["param"] == p]
        ms = [m for _, m in pts]
        y = len(params) - 1 - i
        ax.plot([min(ms), max(ms)], [y, y], color=GRID, linewidth=6, solid_capstyle="round")
        for v, m in pts:
            ax.plot([m], [y], marker="o", markersize=7, color=SERIES[0] if m >= need - 1e-6 else SERIES[1],
                    markeredgecolor=SURFACE, linestyle="none")
            ax.annotate(f"{v:g}", (m, y), textcoords="offset points", xytext=(0, 7), ha="center", fontsize=7, color=INK_2)
    ax.axvline(need, color=INK, linewidth=1, linestyle="--")
    ax.annotate("regla 65%", (need, len(params) - 0.4), textcoords="offset points", xytext=(4, 0), fontsize=8, color=INK)
    ax.set_yticks(range(len(params)))
    ax.set_yticklabels(list(reversed(params)), fontsize=8, color=INK_2)
    ax.set_xlabel("margen de ganancia (multiplicador de kP estable en todos los robots)")
    ax.set_title("Margen de ganancia segun cada parametro (etiqueta = valor del parametro)", fontsize=11, loc="left")
    fig.tight_layout()
    f = out / "sensibilidad.png"
    fig.savefig(f, facecolor=SURFACE)
    plt.close(fig)
    return f"![sensibilidad]({f.name})"


if __name__ == "__main__":
    main()

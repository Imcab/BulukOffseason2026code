"""Repeticion visual: muchos robots aleatorizados siguiendo las trayectorias, en un HTML autocontenido.

Cada escenario = un par de kP. Todos los escenarios usan los MISMOS robots aleatorizados, asi se
ve directamente que cambia al subir o bajar la ganancia. El HTML no necesita internet (las
fuentes caen a las del sistema si no hay conexion).

Uso (desde tools/pid_sweep):
  python -m core.replay                                   # escenarios por defecto
  python -m core.replay --scenario "Autotune=7.5,11.5" --scenario "Gazebo=10,5"
  python -m core.replay --robots 144 --variant velocity --out results/replay.html
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import torch

from .params import PACKAGE_DIR, load_config, sample_env_params, variant
from .sim import run_batch
from .sweep import all_trajectories, hold_env
from .trajectory import HOLD_NAME

TEMPLATE = Path(__file__).with_name("replay_template.html")
DEFAULT_SCENARIOS = ["Autotune=6,7.5", "Antes=5,5", "Gazebo=10,5", "Inestable=25,25"]


def _r(x, nd=3):
    return [round(float(v), nd) for v in x]


def build_replay(cfg: dict, scenarios: list[tuple[str, float, float]], n_robots: int, var_name: str,
                 device="cpu", seed: int = 3472, frame_every: int = 2) -> dict:
    var = variant(cfg, var_name)
    trajs = all_trajectories(cfg)
    nj = len(trajs)
    g = torch.Generator().manual_seed(seed)
    base = sample_env_params(cfg, torch.zeros(n_robots), torch.zeros(n_robots), g)
    tid = torch.arange(n_robots) % nj  # reparte robots entre trayectorias (incluye mantener_pose)
    tid, order = torch.sort(tid, stable=True)  # agrupados por trayectoria, como filas de entornos
    base = base.index(order)
    hold = hold_env(cfg, base)
    is_hold = torch.tensor([trajs[int(j)].name == HOLD_NAME for j in tid])
    base = base.replace(**{k: torch.where(is_hold, getattr(hold, k), getattr(base, k))
                           for k in ("init_dx", "init_dy", "init_dyaw")})

    loop_dt = float(cfg["code"]["loop_period_s"])
    frame_dt = loop_dt * frame_every
    out_traj = []
    for tr in trajs:
        k = int(math.ceil(tr.total_time / frame_dt))
        pts = [tr.sample(i * frame_dt) for i in range(k + 1)]
        xs, ys = [p["x"] for p in pts], [p["y"] for p in pts]
        out_traj.append({"name": tr.name, "description": tr.description, "total_time": tr.total_time,
                         "x": _r(xs), "y": _r(ys), "theta": _r([p["theta"] for p in pts]),
                         "bounds": [min(xs), min(ys), max(xs), max(ys)]})

    robots = []
    for i in range(n_robots):
        robots.append({"traj": int(tid[i]), "latency_ms": round(float(base.latency_s[i]) * 1000),
                       "cof": round(float(base.cof[i]), 2), "speed_pct": round(float(base.speed_factor[i]) * 100),
                       "moi": round(float(base.moi[i]), 2), "mass": round(float(base.mass[i]), 1),
                       "cg_h": round(float(base.cg_height[i]), 3),
                       "init_cm": round(100 * math.hypot(float(base.init_dx[i]), float(base.init_dy[i]))),
                       "init_deg": round(math.degrees(float(base.init_dyaw[i])), 1)})

    out_sc = []
    for label, kt, kr in scenarios:
        env = base.replace(kp_t=torch.full((n_robots,), kt), kp_r=torch.full((n_robots,), kr))
        res = run_batch(cfg, var, env, trajs, tid, device, record_poses=True)
        poses = res["poses"][::frame_every]  # (F, N, 3)
        runs = []
        for i in range(n_robots):
            p = poses[:, i]
            runs.append({
                "x": _r(p[:, 0]), "y": _r(p[:, 1]), "th": _r(p[:, 2]),
                "state": "inestable" if bool(res["unstable"][i]) else ("aborto" if bool(res["abort"][i]) else "ok"),
                "final_cm": round(100 * float(res["final_pos_err_m"][i]), 1),
                "final_deg": round(float(res["final_yaw_err_deg"][i]), 1),
                "max_cm": round(100 * min(float(res["max_pos_err_m"][i]), 99.0), 1),
                "slip_pct": round(100 * float(res["slip_frac"][i])),
            })
        n_un = sum(r["state"] == "inestable" for r in runs)
        n_ab = sum(r["state"] == "aborto" for r in runs)
        print(f"[replay] {label} kP=({kt:g}, {kr:g}): inestables {n_un}, abortos {n_ab} de {n_robots}")
        out_sc.append({"label": label, "kp_t": kt, "kp_r": kr, "runs": runs})

    return {"variant": var_name, "frame_dt": frame_dt, "bumper_m": cfg["robot"]["bumper_size_m"][0],
            "settle_s": float(cfg["sim"]["settle_time_s"]), "trajectories": out_traj, "robots": robots,
            "scenarios": out_sc,
            "ranges": {k: cfg["randomization"][k] for k in ("cof", "speed_factor", "latency_s", "moi_yaw_kgm2")}}


def write_html(data: dict, out: Path, fragment: bool = False) -> Path:
    html = TEMPLATE.read_text(encoding="utf-8")
    html = html.replace("/*__REPLAY_DATA__*/null", json.dumps(data, separators=(",", ":")))
    if not fragment:
        html = ("<!doctype html>\n<html lang=\"es\">\n<head>\n<meta charset=\"utf-8\">\n"
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
                "</head>\n<body>\n" + html + "\n</body>\n</html>\n")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8")
    return out


def _parse_scenario(s: str) -> tuple[str, float, float]:
    label, _, kps = s.partition("=")
    kt, kr = (float(v) for v in kps.split(","))
    return label.strip(), kt, kr


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--scenario", action="append", help='"Nombre=kP_t,kP_r" (repetible)')
    ap.add_argument("--robots", type=int, default=144)
    ap.add_argument("--variant", default="velocity_limites_propuestos")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--seed", type=int, default=3472)
    ap.add_argument("--out", default=str(PACKAGE_DIR / "results" / "replay.html"))
    ap.add_argument("--fragment", action="store_true", help="sin <html>/<head> (para publicar como artifact)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    scenarios = [_parse_scenario(s) for s in (args.scenario or DEFAULT_SCENARIOS)]
    data = build_replay(cfg, scenarios, args.robots, args.variant, args.device, args.seed)
    path = write_html(data, Path(args.out), args.fragment)
    print(f"[replay] {path} ({path.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

"""Barrido de kP: comun a sweep_torch.py y a isaac/run_sweep.py.

Cada entorno = un par (kP_t, kP_r) + un robot aleatorizado. Los MISMOS robots aleatorizados
se usan en todas las variantes y trayectorias (numeros aleatorios comunes), asi las
comparaciones entre variantes no dependen de la suerte del muestreo.
"""

from __future__ import annotations

import copy
import csv
import math
import time
from pathlib import Path
from typing import Callable

import torch

from .params import EnvParams, build_sweep_grid, sample_env_params
from .sim import run_trajectory
from .trajectory import HOLD_NAME, hold_trajectory, list_trajectories

METRICS = [
    "lateral_rms_m", "pos_rms_m", "max_pos_err_m", "max_yaw_err_deg", "final_pos_err_m", "final_yaw_err_deg",
    "overshoot_m", "overshoot_deg", "path_time_s", "total_time_s", "slip_frac", "reversals_t", "reversals_r",
]
FLAGS = ["abort", "unstable_t", "unstable_r", "unstable", "wheel_lift"]


def quick_config(cfg: dict) -> dict:
    """Configuracion reducida para una primera corrida (pocos entornos)."""
    q = copy.deepcopy(cfg)
    q["sweep"]["kp_translation"] = {"start": 2.0, "stop": 20.0, "num": 4}
    q["sweep"]["kp_rotation"] = {"start": 2.0, "stop": 12.0, "num": 3}
    q["sweep"]["samples_per_cell"] = 2
    q["sweep"]["trajectories"] = ["recta_4m", "curva_s_lateral", "giro_sitio_180"]
    return q


def hold_env(cfg: dict, env: EnvParams) -> EnvParams:
    """Mismo robot, pero con el error inicial al maximo del rango (direccion conservada)."""
    rnd = cfg["randomization"]
    ang = torch.atan2(env.init_dy, env.init_dx)
    r = float(rnd["init_pos_offset_m"])
    yaw = math.radians(float(rnd["init_yaw_offset_deg"]))
    cols = env.as_columns()
    cols["init_dx"] = r * torch.cos(ang)
    cols["init_dy"] = r * torch.sin(ang)
    cols["init_dyaw"] = torch.where(env.init_dyaw >= 0, yaw, -yaw).to(env.init_dyaw.dtype)
    return EnvParams(**cols)


def run_sweep(cfg: dict, body_factory: Callable[[EnvParams], object], device="cpu",
              out_dir: str | Path = "results", log: Callable[[str], None] = print,
              backend: str = "torch") -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    kp_t, kp_r = build_sweep_grid(cfg)
    gen = torch.Generator().manual_seed(int(cfg["sweep"]["seed"]))
    env = sample_env_params(cfg, kp_t, kp_r, gen)
    trajs = [hold_trajectory(float(cfg["stability"]["hold_test_time_s"]))]
    trajs += list_trajectories(cfg["sweep"]["trajectories"])
    log(f"[sweep] backend={backend} entornos={env.n} variantes={len(cfg['variants'])} "
        f"trayectorias={len(trajs)} ({', '.join(t.name for t in trajs)})")

    rows = []
    t_start = time.time()
    for var in cfg["variants"]:
        for traj in trajs:
            e = hold_env(cfg, env) if traj.name == HOLD_NAME else env
            t0 = time.time()
            body = body_factory(e)
            res = run_trajectory(cfg, var, e, traj, body, device)
            n_bad = int(res["unstable"].sum())
            log(f"[sweep] {var['name']:>28} | {traj.name:>18} | {time.time() - t0:6.1f} s | inestables {n_bad}/{e.n}")
            cols = e.as_columns()
            for i in range(e.n):
                row = {"variant": var["name"], "trajectory": traj.name, "env_id": i}
                for k, v in cols.items():
                    row[k] = float(v[i])
                row["init_dyaw_deg"] = math.degrees(row.pop("init_dyaw"))
                for k in METRICS:
                    row[k] = float(res[k][i])
                for k in FLAGS:
                    row[k] = int(bool(res[k][i]))
                rows.append(row)
    log(f"[sweep] simulacion total: {time.time() - t_start:.1f} s")

    csv_path = out_dir / "sweep_results.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    log(f"[sweep] {csv_path} ({len(rows)} filas)")

    from .report import write_report  # import tardio: matplotlib solo hace falta aqui

    write_report(cfg, rows, out_dir, backend=backend, log=log)
    return out_dir

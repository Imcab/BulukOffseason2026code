"""Barrido de kP en grid (lo usa core/sweep_torch.py) y utilidades de evaluacion en lote.

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
from .sim import FLAGS, METRICS, run_batch
from .trajectory import HOLD_NAME, Trajectory, hold_trajectory, list_trajectories


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
    return env.replace(
        init_dx=r * torch.cos(ang),
        init_dy=r * torch.sin(ang),
        init_dyaw=torch.where(env.init_dyaw >= 0, yaw, -yaw).to(env.init_dyaw.dtype),
    )


def all_trajectories(cfg: dict) -> list[Trajectory]:
    """La prueba de estabilidad mantener_pose + las trayectorias de cfg['sweep']['trajectories']."""
    return [hold_trajectory(float(cfg["stability"]["hold_test_time_s"]))] + list_trajectories(
        cfg["sweep"]["trajectories"])


def evaluate(cfg: dict, variant: dict, env: EnvParams, trajs: list[Trajectory], device="cpu",
             max_batch: int = 200_000) -> tuple[dict, torch.Tensor, torch.Tensor]:
    """Corre cada robot de env en CADA trayectoria (hold con el error inicial maximo).

    Devuelve (resultados dict (N*J,), indice de robot (N*J,), indice de trayectoria (N*J,)).
    El orden es trayectoria-mayor: fila = j * N + i.
    """
    n, nj = env.n, len(trajs)
    envs = [hold_env(cfg, env) if t.name == HOLD_NAME else env for t in trajs]
    big = EnvParams.cat(envs)
    tid = torch.arange(nj).repeat_interleave(n)
    rid = torch.arange(n).repeat(nj)
    chunks = []
    for a in range(0, big.n, max_batch):
        sl = slice(a, min(a + max_batch, big.n))
        chunks.append(run_batch(cfg, variant, big.index(sl), trajs, tid[sl], device))
    res = {k: torch.cat([c[k] for c in chunks]) for k in chunks[0] if k != "trace"}
    return res, rid, tid


def run_sweep(cfg: dict, device="cpu", out_dir: str | Path = "results", log: Callable[[str], None] = print,
              max_batch: int = 200_000) -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    kp_t, kp_r = build_sweep_grid(cfg)
    gen = torch.Generator().manual_seed(int(cfg["sweep"]["seed"]))
    env = sample_env_params(cfg, kp_t, kp_r, gen)
    trajs = all_trajectories(cfg)
    log(f"[sweep] device={device} robots={env.n} variantes={len(cfg['variants'])} "
        f"trayectorias={len(trajs)} ({', '.join(t.name for t in trajs)})")

    rows = []
    t_start = time.time()
    for var in cfg["variants"]:
        t0 = time.time()
        res, rid, tid = evaluate(cfg, var, env, trajs, device, max_batch)
        log(f"[sweep] {var['name']:>28} | {time.time() - t0:6.1f} s | {rid.numel()} corridas | "
            f"inestables {int(res['unstable'].sum())}")
        hold = hold_env(cfg, env)
        cols = {k: v for k, v in env.as_columns().items()}
        hcols = hold.as_columns()
        for row_i in range(rid.numel()):
            i, j = int(rid[row_i]), int(tid[row_i])
            src = hcols if trajs[j].name == HOLD_NAME else cols
            row = {"variant": var["name"], "trajectory": trajs[j].name, "env_id": i}
            for k, v in src.items():
                row[k] = float(v[i])
            row["init_dyaw_deg"] = math.degrees(row.pop("init_dyaw"))
            for k in METRICS:
                row[k] = float(res[k][row_i])
            for k in FLAGS:
                row[k] = int(bool(res[k][row_i]))
            rows.append(row)
    log(f"[sweep] simulacion total: {time.time() - t_start:.1f} s")

    csv_path = out_dir / "sweep_results.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    log(f"[sweep] {csv_path} ({len(rows)} filas)")

    from .report import write_report  # import tardio: matplotlib solo hace falta aqui

    write_report(cfg, rows, out_dir, backend=f"torch-{device}", log=log)
    return out_dir

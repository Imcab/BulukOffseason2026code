"""Trayectorias muestreadas (JSON) y su interpolacion.

Formato de trajectories/*.json (columnas, marco de campo, unidades SI, theta en rad):
  {"name", "description", "dt", "total_time", "constraints",
   "samples": {"t": [...], "x": [...], "y": [...], "theta": [...],
               "vx": [...], "vy": [...], "omega": [...]}}
vx, vy, omega son velocidades de CAMPO (como PathPlannerTrajectoryState.fieldSpeeds).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import torch

from .params import DTYPE, PACKAGE_DIR

TRAJ_DIR = PACKAGE_DIR / "trajectories"
KEYS = ("t", "x", "y", "theta", "vx", "vy", "omega")


@dataclass
class Trajectory:
    name: str
    description: str
    total_time: float
    data: dict[str, torch.Tensor]  # columnas de KEYS, forma (M,)

    @staticmethod
    def from_json(path: str | Path) -> "Trajectory":
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        data = {k: torch.tensor(d["samples"][k], dtype=DTYPE) for k in KEYS}
        return Trajectory(d["name"], d.get("description", ""), float(d["total_time"]), data)

    def sample(self, t: float) -> dict[str, float]:
        """Estado objetivo en el tiempo t (interpolacion lineal; theta por el camino corto).

        Igual que PathPlanner: antes de 0 devuelve el inicio y despues de total_time el final.
        """
        ts = self.data["t"]
        if t <= float(ts[0]):
            return {k: float(self.data[k][0]) for k in KEYS[1:]}
        if t >= float(ts[-1]):
            return {k: float(self.data[k][-1]) for k in KEYS[1:]}
        i = int(torch.searchsorted(ts, torch.tensor(t, dtype=DTYPE), right=True)) - 1
        i = min(max(i, 0), ts.numel() - 2)  # t puede redondear justo al ultimo punto en float32
        t0, t1 = float(ts[i]), float(ts[i + 1])
        a = min(max((t - t0) / (t1 - t0), 0.0), 1.0)
        out = {}
        for k in KEYS[1:]:
            v0, v1 = float(self.data[k][i]), float(self.data[k][i + 1])
            if k == "theta":
                d = math.atan2(math.sin(v1 - v0), math.cos(v1 - v0))
                out[k] = v0 + a * d
            else:
                out[k] = v0 + a * (v1 - v0)
        return out

    @property
    def start(self) -> dict[str, float]:
        return self.sample(0.0)

    @property
    def end(self) -> dict[str, float]:
        return self.sample(self.total_time)


def list_trajectories(names="all", directory: Path = TRAJ_DIR) -> list[Trajectory]:
    paths = sorted(Path(directory).glob("*.json"))
    if names != "all":
        wanted = set(names)
        paths = [p for p in paths if p.stem in wanted]
        missing = wanted - {p.stem for p in paths}
        if missing:
            raise FileNotFoundError(f"no existen trayectorias: {sorted(missing)}")
    if not paths:
        raise FileNotFoundError(f"no hay trayectorias en {directory}")
    return [Trajectory.from_json(p) for p in paths]


HOLD_NAME = "mantener_pose"


def hold_trajectory(duration: float, dt: float = 0.02) -> Trajectory:
    """Trayectoria quieta en el origen (prueba de estabilidad del lazo, sin feedforward)."""
    m = int(round(duration / dt)) + 1
    t = torch.linspace(0.0, duration, m, dtype=DTYPE)
    z = torch.zeros(m, dtype=DTYPE)
    data = {"t": t, "x": z, "y": z, "theta": z, "vx": z, "vy": z, "omega": z}
    return Trajectory(HOLD_NAME, "Mantener pose con error inicial (estabilidad del lazo)", float(duration), data)

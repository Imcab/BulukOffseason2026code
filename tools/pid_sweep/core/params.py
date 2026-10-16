"""Carga de params.yaml y muestreo de parametros aleatorizados por entorno."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from pathlib import Path

import torch
import yaml

DTYPE = torch.float32
PACKAGE_DIR = Path(__file__).resolve().parent.parent


def load_config(path: str | Path | None = None) -> dict:
    path = Path(path) if path else PACKAGE_DIR / "params.yaml"
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def variant(cfg: dict, name: str | None = None) -> dict:
    """Devuelve la variante por nombre (o la primera = la del robot hoy)."""
    if name is None:
        return cfg["variants"][0]
    for v in cfg["variants"]:
        if v["name"] == name:
            return v
    raise KeyError(f"variante desconocida: {name}")


@dataclass
class EnvParams:
    """Un valor por entorno (tensores de forma (N,))."""

    mass: torch.Tensor
    moi: torch.Tensor
    cg_height: torch.Tensor
    cg_x: torch.Tensor
    cg_y: torch.Tensor
    cof: torch.Tensor
    speed_factor: torch.Tensor
    latency_s: torch.Tensor
    kp_t: torch.Tensor
    kp_r: torch.Tensor
    init_dx: torch.Tensor
    init_dy: torch.Tensor
    init_dyaw: torch.Tensor

    @property
    def n(self) -> int:
        return self.mass.shape[0]

    def to(self, device) -> "EnvParams":
        return EnvParams(**{f.name: getattr(self, f.name).to(device) for f in fields(self)})

    def as_columns(self) -> dict[str, torch.Tensor]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    def index(self, idx) -> "EnvParams":
        return EnvParams(**{f.name: getattr(self, f.name)[idx] for f in fields(self)})

    def replace(self, **cols) -> "EnvParams":
        c = self.as_columns()
        c.update(cols)
        return EnvParams(**c)

    @staticmethod
    def cat(envs: list["EnvParams"]) -> "EnvParams":
        return EnvParams(**{f.name: torch.cat([getattr(e, f.name) for e in envs]) for f in fields(envs[0])})


def nominal_env_params(cfg: dict, kp_t, kp_r, n: int | None = None, **overrides) -> EnvParams:
    """Robot nominal (sin aleatorizar), util para pruebas. kp_t/kp_r escalares o tensores."""
    kp_t = torch.as_tensor(kp_t, dtype=DTYPE).flatten()
    kp_r = torch.as_tensor(kp_r, dtype=DTYPE).flatten()
    n = n or max(kp_t.numel(), kp_r.numel())
    r = cfg["robot"]

    def full(v):
        return torch.full((n,), float(v), dtype=DTYPE)

    p = dict(
        mass=full(r["mass_kg"] + r["extra_mass_kg"]),
        moi=full(r["moi_yaw_kgm2"]),
        cg_height=full(r["cg_height_m"]),
        cg_x=full(r["cg_offset_m"][0]),
        cg_y=full(r["cg_offset_m"][1]),
        cof=full(1.0),
        speed_factor=full(1.0),
        latency_s=full(0.02),
        kp_t=kp_t.expand(n).clone(),
        kp_r=kp_r.expand(n).clone(),
        init_dx=full(0.0),
        init_dy=full(0.0),
        init_dyaw=full(0.0),
    )
    for k, v in overrides.items():
        p[k] = full(v) if not torch.is_tensor(v) else v.to(DTYPE).expand(n).clone()
    return EnvParams(**p)


def sample_env_params(cfg: dict, kp_t: torch.Tensor, kp_r: torch.Tensor, generator: torch.Generator) -> EnvParams:
    """Aleatoriza segun cfg['randomization'] un robot por cada par (kp_t[i], kp_r[i])."""
    n = kp_t.numel()
    r = cfg["robot"]
    rnd = cfg["randomization"]

    def uni(lo_hi):
        lo, hi = float(lo_hi[0]), float(lo_hi[1])
        return lo + (hi - lo) * torch.rand(n, generator=generator, dtype=DTYPE)

    # Error inicial de pose: direccion uniforme, radio uniforme en area dentro del circulo.
    ang = 2 * math.pi * torch.rand(n, generator=generator, dtype=DTYPE)
    rad = float(rnd["init_pos_offset_m"]) * torch.sqrt(torch.rand(n, generator=generator, dtype=DTYPE))
    dyaw = math.radians(float(rnd["init_yaw_offset_deg"])) * (2 * torch.rand(n, generator=generator, dtype=DTYPE) - 1)

    return EnvParams(
        mass=r["mass_kg"] + r["extra_mass_kg"] + uni(rnd["extra_mass_kg"]),
        moi=uni(rnd["moi_yaw_kgm2"]),
        cg_height=uni(rnd["cg_height_m"]),
        cg_x=torch.full((n,), float(r["cg_offset_m"][0]), dtype=DTYPE),
        cg_y=torch.full((n,), float(r["cg_offset_m"][1]), dtype=DTYPE),
        cof=uni(rnd["cof"]),
        speed_factor=uni(rnd["speed_factor"]),
        latency_s=uni(rnd["latency_s"]),
        kp_t=kp_t.to(DTYPE).clone(),
        kp_r=kp_r.to(DTYPE).clone(),
        init_dx=rad * torch.cos(ang),
        init_dy=rad * torch.sin(ang),
        init_dyaw=dyaw,
    )


def linspace(spec: dict) -> torch.Tensor:
    return torch.linspace(float(spec["start"]), float(spec["stop"]), int(spec["num"]), dtype=DTYPE)


def build_sweep_grid(cfg: dict) -> tuple[torch.Tensor, torch.Tensor]:
    """Producto cartesiano kP_t x kP_r, repetido samples_per_cell veces. Devuelve (kp_t, kp_r) de forma (N,)."""
    kt = linspace(cfg["sweep"]["kp_translation"])
    kr = linspace(cfg["sweep"]["kp_rotation"])
    reps = int(cfg["sweep"]["samples_per_cell"])
    gt, gr = torch.meshgrid(kt, kr, indexing="ij")
    return gt.flatten().repeat_interleave(reps), gr.flatten().repeat_interleave(reps)

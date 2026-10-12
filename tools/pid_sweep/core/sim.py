"""Corre una trayectoria en N entornos en paralelo y calcula las metricas.

El "cuerpo" (integrador del cuerpo rigido) es intercambiable: core.model.TorchBody o el de
isaac/run_sweep.py. Ambos exponen reset(center_xy, yaw), state() y step(force_b, torque).

Secuencia por paso de fisica (dt):
  - cada loop_period (20 ms): el controlador lee la pose del centro y calcula el comando
    (retencion de orden cero entre ciclos). Igual que FollowPathCommand: al pasar
    total_time manda ChassisSpeeds en cero y ya no corrige.
  - el comando pasa por el retraso de latencia (por entorno) y llega a los modulos.
  - el modelo calcula fuerza/torque y el cuerpo integra.
"""

from __future__ import annotations

import math

import torch

from .controller import CommandDelay, pp_holonomic
from .model import SwerveModel, center_pose, wrap
from .params import DTYPE, EnvParams
from .trajectory import HOLD_NAME, Trajectory


class _Zigzag:
    """Cuenta inversiones de una senal con histeresis (detector de oscilacion)."""

    def __init__(self, x0: torch.Tensor, hyst: float):
        self.ext = x0.clone()
        self.dir = torch.zeros_like(x0)
        self.count = torch.zeros_like(x0)
        self.h = hyst

    def update(self, x: torch.Tensor):
        up = x - self.ext > self.h
        down = self.ext - x > self.h
        rev = ((self.dir > 0) & down) | ((self.dir < 0) & up)
        self.count += rev.to(x.dtype)
        new_dir = torch.where(up, torch.ones_like(x), torch.where(down, -torch.ones_like(x), self.dir))
        # Extremo: se mueve con la senal en la direccion actual, o se reinicia al invertir.
        moved = up | down
        extend = ((self.dir > 0) & (x > self.ext)) | ((self.dir < 0) & (x < self.ext))
        self.ext = torch.where(moved | extend, x, self.ext)
        self.dir = new_dir


def _final_approach_dir(traj: Trajectory) -> tuple[float, float] | None:
    vx, vy = traj.data["vx"], traj.data["vy"]
    sp = torch.hypot(vx, vy)
    idx = torch.nonzero(sp > 0.1).flatten()
    if idx.numel() == 0:
        return None
    i = int(idx[-1])
    return float(vx[i] / sp[i]), float(vy[i] / sp[i])


def run_trajectory(cfg: dict, variant: dict, env: EnvParams, traj: Trajectory, body, device="cpu",
                   record: list[int] | None = None) -> dict:
    """Devuelve un dict de tensores (N,) en CPU con las metricas (y 'trace' si record)."""
    sim, st = cfg["sim"], cfg["stability"]
    dt = float(sim["physics_dt_s"])
    loop = max(1, round(float(cfg["code"]["loop_period_s"]) / dt))
    env = env.to(device)
    n = env.n
    model = SwerveModel(cfg, variant, env, device)
    delay = CommandDelay(n, torch.round(env.latency_s / dt).long(), 3)

    s0, s_end = traj.start, traj.end
    T = traj.total_time
    yaw0 = s0["theta"] + env.init_dyaw
    c0 = torch.stack([s0["x"] + env.init_dx, s0["y"] + env.init_dy], dim=-1)
    body.reset(c0, yaw0)

    f = lambda v: torch.full((n,), float(v), dtype=DTYPE, device=device)  # noqa: E731
    zeros = f(0.0)
    end_xy = torch.tensor([s_end["x"], s_end["y"]], dtype=DTYPE, device=device)
    approach = _final_approach_dir(traj)
    dth_total = math.atan2(math.sin(s_end["theta"] - s0["theta"]), math.cos(s_end["theta"] - s0["theta"]))
    yaw_sign = math.copysign(1.0, dth_total) if abs(dth_total) > 1e-3 else 0.0

    lat_sq, n_lat = zeros.clone(), 0
    pos_sq, n_pos = zeros.clone(), 0
    max_pos, max_yaw = zeros.clone(), zeros.clone()
    overshoot_m, overshoot_deg = zeros.clone(), zeros.clone()
    slip_steps, path_steps = zeros.clone(), 0
    t_stop = f(float("nan"))
    nan_flag = torch.zeros(n, dtype=torch.bool, device=device)
    zz = None

    n_steps = int(math.ceil((T + float(sim["settle_time_s"])) / dt))
    held = torch.zeros(n, 3, dtype=DTYPE, device=device)
    trace = {i: [] for i in (record or [])}

    for k in range(n_steps):
        t = k * dt
        pos_cm, yaw, vel, omega = body.state()
        pc = center_pose(pos_cm, yaw, model.cg)
        running = t < T - 1e-9

        if k % loop == 0:
            if running:
                tg = traj.sample(t)
                held, _ = pp_holonomic(pc, yaw, tg, env.kp_t, env.kp_r)
                ex, ey = tg["x"] - pc[:, 0], tg["y"] - pc[:, 1]
                eth = wrap(tg["theta"] - yaw)
                perr = torch.hypot(ex, ey)
                pos_sq += perr**2
                n_pos += 1
                sp = math.hypot(tg["vx"], tg["vy"])
                if sp > 0.1:  # error lateral = perpendicular a la direccion de avance
                    lat = (ex * tg["vy"] - ey * tg["vx"]) / sp
                    lat_sq += lat**2
                    n_lat += 1
                max_pos = torch.maximum(max_pos, torch.nan_to_num(perr, nan=1e9))
                max_yaw = torch.maximum(max_yaw, torch.nan_to_num(eth.abs(), nan=1e9))
                sig = torch.stack([ex, ey, eth], dim=-1)
                if zz is None:
                    zz = (_Zigzag(sig[:, 0], st["osc_hysteresis_m"]), _Zigzag(sig[:, 1], st["osc_hysteresis_m"]),
                          _Zigzag(sig[:, 2], math.radians(st["osc_hysteresis_deg"])))
                else:
                    for j in range(3):
                        zz[j].update(sig[:, j])
            else:
                held = torch.zeros_like(held)  # FollowPathCommand.end(): ChassisSpeeds()

            if approach is not None:
                proj = (pc[:, 0] - end_xy[0]) * approach[0] + (pc[:, 1] - end_xy[1]) * approach[1]
                overshoot_m = torch.maximum(overshoot_m, torch.nan_to_num(proj, nan=0.0))
            if yaw_sign != 0.0:
                over = wrap(yaw - s_end["theta"]) * yaw_sign
                overshoot_deg = torch.maximum(overshoot_deg, torch.nan_to_num(torch.rad2deg(over), nan=0.0))
            if not running:
                stopped = (torch.hypot(vel[:, 0], vel[:, 1]) < st["stopped_speed_mps"]) & (
                    omega.abs() < st["stopped_omega_rad_s"])
                t_stop = torch.where(torch.isnan(t_stop) & stopped, f(t), t_stop)
            for i in trace:
                trace[i].append((t, float(pc[i, 0]), float(pc[i, 1]), float(yaw[i])))

        cmd = delay.push_and_get(held)
        force, torque = model.step(vel, yaw, omega, cmd, dt)
        bad = ~torch.isfinite(force).all(dim=1) | ~torch.isfinite(torque)
        nan_flag |= bad
        force = torch.nan_to_num(force)
        torque = torch.nan_to_num(torque)
        body.step(force, torque)
        if running:
            slip_steps += model.slipping.to(DTYPE)
            path_steps += 1

    pos_cm, yaw, vel, omega = body.state()
    pc = center_pose(pos_cm, yaw, model.cg)
    final_pos = torch.hypot(pc[:, 0] - end_xy[0], pc[:, 1] - end_xy[1])
    final_yaw = torch.rad2deg(wrap(yaw - s_end["theta"]).abs())
    nan_flag |= ~torch.isfinite(final_pos) | ~torch.isfinite(final_yaw)

    rev_t = torch.maximum(zz[0].count, zz[1].count)
    rev_r = zz[2].count
    abort_t = (max_pos > st["abort_pos_err_m"]) | nan_flag
    abort_r = (torch.rad2deg(max_yaw) > st["abort_yaw_err_deg"]) | nan_flag
    unstable_t = (max_pos > st["divergence_pos_err_m"]) | nan_flag
    unstable_r = (torch.rad2deg(max_yaw) > st["divergence_yaw_err_deg"]) | nan_flag
    if traj.name == HOLD_NAME:
        # Prueba de estabilidad: el error oscila (inversiones con histeresis) en vez de asentarse.
        unstable_t |= rev_t > st["hold_max_reversals"]
        unstable_r |= rev_r > st["hold_max_reversals"]

    out = {
        "lateral_rms_m": torch.sqrt(lat_sq / max(n_lat, 1)) if n_lat else zeros,
        "pos_rms_m": torch.sqrt(pos_sq / max(n_pos, 1)),
        "max_pos_err_m": max_pos,
        "max_yaw_err_deg": torch.rad2deg(max_yaw),
        "final_pos_err_m": final_pos,
        "final_yaw_err_deg": final_yaw,
        "overshoot_m": overshoot_m,
        "overshoot_deg": overshoot_deg,
        "path_time_s": f(T),
        "total_time_s": t_stop,
        "slip_frac": slip_steps / max(path_steps, 1),
        "reversals_t": rev_t,
        "reversals_r": rev_r,
        "abort": abort_t | abort_r,
        "unstable_t": unstable_t,
        "unstable_r": unstable_r,
        "wheel_lift": model.wheel_lift.clone(),
    }
    out["unstable"] = out["unstable_t"] | out["unstable_r"]
    out = {k: v.detach().cpu() for k, v in out.items()}
    if record:
        out["trace"] = trace
    return out

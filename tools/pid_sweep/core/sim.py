"""Corre N entornos en paralelo (cada uno con SU trayectoria) y calcula las metricas.

Todo va en un solo lote de torch: entornos con trayectorias distintas (y de distinta
duracion) avanzan juntos; cada uno usa su propio tiempo de fin. Asi la GPU se aprovecha
con decenas de miles de entornos a la vez.

Secuencia por paso de fisica (dt):
  - cada loop_period (20 ms): el controlador lee la pose del centro y calcula el comando
    (retencion de orden cero entre ciclos). Igual que FollowPathCommand: al pasar
    total_time manda ChassisSpeeds en cero y ya no corrige.
  - el comando pasa por el retraso de latencia (por entorno) y llega a los modulos.
  - el modelo calcula fuerza/torque y el cuerpo rigido integra.
"""

from __future__ import annotations

import math

import torch

from .controller import CommandDelay, pp_holonomic
from .model import SwerveModel, TorchBody, center_pose, wrap
from .params import DTYPE, EnvParams
from .trajectory import HOLD_NAME, Trajectory

METRICS = [
    "lateral_rms_m", "pos_rms_m", "yaw_rms_deg", "max_pos_err_m", "max_yaw_err_deg", "final_pos_err_m",
    "final_yaw_err_deg", "overshoot_m", "overshoot_deg", "path_time_s", "total_time_s", "slip_frac",
    "reversals_t", "reversals_r",
]
FLAGS = ["abort", "unstable_t", "unstable_r", "unstable", "wheel_lift"]


class _Zigzag:
    """Cuenta inversiones de una senal con histeresis (detector de oscilacion)."""

    def __init__(self, x0: torch.Tensor, hyst: float):
        self.ext = x0.clone()
        self.dir = torch.zeros_like(x0)
        self.count = torch.zeros_like(x0)
        self.h = hyst

    def update(self, x: torch.Tensor, active: torch.Tensor):
        up = (x - self.ext > self.h) & active
        down = (self.ext - x > self.h) & active
        rev = ((self.dir > 0) & down) | ((self.dir < 0) & up)
        self.count += rev.to(x.dtype)
        new_dir = torch.where(up, torch.ones_like(x), torch.where(down, -torch.ones_like(x), self.dir))
        # Extremo: se mueve con la senal en la direccion actual, o se reinicia al invertir.
        extend = (((self.dir > 0) & (x > self.ext)) | ((self.dir < 0) & (x < self.ext))) & active
        self.ext = torch.where(up | down | extend, x, self.ext)
        self.dir = new_dir


def _final_approach_dir(traj: Trajectory) -> tuple[float, float]:
    vx, vy = traj.data["vx"], traj.data["vy"]
    sp = torch.hypot(vx, vy)
    idx = torch.nonzero(sp > 0.1).flatten()
    if idx.numel() == 0:
        return 0.0, 0.0  # sin traslacion: no hay sobreimpulso lineal
    i = int(idx[-1])
    return float(vx[i] / sp[i]), float(vy[i] / sp[i])


def _target_table(trajs: list[Trajectory], loop_dt: float, device) -> tuple[torch.Tensor, int]:
    """Objetivos pre-muestreados en cada ciclo del controlador: (n_traj, K, 6) x, y, th, vx, vy, w."""
    k_max = max(int(math.ceil(t.total_time / loop_dt)) for t in trajs) + 1
    table = torch.zeros(len(trajs), k_max, 6, dtype=DTYPE)
    keys = ("x", "y", "theta", "vx", "vy", "omega")
    for j, tr in enumerate(trajs):
        for k in range(k_max):
            s = tr.sample(k * loop_dt)
            table[j, k] = torch.tensor([s[key] for key in keys])
    return table.to(device), k_max


def run_batch(cfg: dict, variant: dict, env: EnvParams, trajs: list[Trajectory], traj_id: torch.Tensor,
              device="cpu", body=None, record: list[int] | None = None, record_poses: bool = False) -> dict:
    """Entorno i sigue trajs[traj_id[i]]. Devuelve dict de tensores (N,) en CPU.

    record: indices cuya pose se guarda como lista (debug). record_poses: guarda la pose del
    centro de TODOS los entornos en cada ciclo de 20 ms -> out["poses"] (K, N, 3) x, y, yaw.
    """
    sim, st = cfg["sim"], cfg["stability"]
    dt = float(sim["physics_dt_s"])
    loop = max(1, round(float(cfg["code"]["loop_period_s"]) / dt))
    loop_dt = loop * dt
    env = env.to(device)
    n = env.n
    tid = traj_id.to(device).long()
    model = SwerveModel(cfg, variant, env, device)
    delay = CommandDelay(n, torch.round(env.latency_s / dt).long(), 3)
    body = body or TorchBody(env, dt, device)

    kw = dict(dtype=DTYPE, device=device)
    table, k_max = _target_table(trajs, loop_dt, device)
    T = torch.tensor([t.total_time for t in trajs], **kw)[tid]
    start = table[tid, 0]
    end = torch.stack([torch.tensor([t.end[k] for k in ("x", "y", "theta")], dtype=DTYPE) for t in trajs]).to(device)[tid]
    approach = torch.tensor([_final_approach_dir(t) for t in trajs], **kw)[tid]
    dth = torch.tensor([math.atan2(math.sin(t.end["theta"] - t.start["theta"]),
                                   math.cos(t.end["theta"] - t.start["theta"])) for t in trajs], **kw)[tid]
    yaw_sign = torch.where(dth.abs() > 1e-3, torch.sign(dth), torch.zeros_like(dth))
    is_hold = torch.tensor([t.name == HOLD_NAME for t in trajs], device=device)[tid]

    yaw0 = start[:, 2] + env.init_dyaw
    body.reset(torch.stack([start[:, 0] + env.init_dx, start[:, 1] + env.init_dy], dim=-1), yaw0)

    zeros = torch.zeros(n, **kw)
    lat_sq, n_lat = zeros.clone(), zeros.clone()
    pos_sq, yaw_sq, n_pos = zeros.clone(), zeros.clone(), zeros.clone()
    max_pos, max_yaw = zeros.clone(), zeros.clone()
    overshoot_m, overshoot_deg = zeros.clone(), zeros.clone()
    slip_steps, path_steps = zeros.clone(), zeros.clone()
    t_stop = torch.full((n,), float("nan"), **kw)
    nan_flag = torch.zeros(n, dtype=torch.bool, device=device)
    held = torch.zeros(n, 3, **kw)
    zz = None
    trace = {i: [] for i in (record or [])}
    poses = []
    hyst_m, hyst_r = float(st["osc_hysteresis_m"]), math.radians(float(st["osc_hysteresis_deg"]))
    stop_v, stop_w = float(st["stopped_speed_mps"]), float(st["stopped_omega_rad_s"])

    n_steps = int(math.ceil((float(T.max()) + float(sim["settle_time_s"])) / dt))
    for k in range(n_steps):
        t = k * dt
        running = t < T - 1e-6
        pos_cm, yaw, vel, omega = body.state()

        if k % loop == 0:
            pc = center_pose(pos_cm, yaw, model.cg)
            tg_all = table[tid, min(k // loop, k_max - 1)]
            tg = {key: tg_all[:, j] for j, key in enumerate(("x", "y", "theta", "vx", "vy", "omega"))}
            cmd, _ = pp_holonomic(pc, yaw, tg, env.kp_t, env.kp_r)
            held = torch.where(running.unsqueeze(1), cmd, torch.zeros_like(cmd))  # FollowPathCommand.end()

            ex, ey = tg["x"] - pc[:, 0], tg["y"] - pc[:, 1]
            eth = wrap(tg["theta"] - yaw)
            perr = torch.hypot(ex, ey)
            rf = running.to(DTYPE)
            pos_sq += perr**2 * rf
            yaw_sq += eth**2 * rf
            n_pos += rf
            sp = torch.hypot(tg["vx"], tg["vy"])
            moving = running & (sp > 0.1)  # error lateral = perpendicular a la direccion de avance
            lat = (ex * tg["vy"] - ey * tg["vx"]) / torch.clamp(sp, min=1e-6)
            lat_sq += torch.where(moving, lat**2, zeros)
            n_lat += moving.to(DTYPE)
            max_pos = torch.where(running, torch.maximum(max_pos, torch.nan_to_num(perr, nan=1e9)), max_pos)
            max_yaw = torch.where(running, torch.maximum(max_yaw, torch.nan_to_num(eth.abs(), nan=1e9)), max_yaw)
            if zz is None:
                zz = (_Zigzag(ex, hyst_m), _Zigzag(ey, hyst_m), _Zigzag(eth, hyst_r))
            else:
                for z, sig in zip(zz, (ex, ey, eth)):
                    z.update(sig, running)

            proj = (pc[:, 0] - end[:, 0]) * approach[:, 0] + (pc[:, 1] - end[:, 1]) * approach[:, 1]
            overshoot_m = torch.maximum(overshoot_m, torch.nan_to_num(proj, nan=0.0))
            over = torch.rad2deg(wrap(yaw - end[:, 2]) * yaw_sign)
            overshoot_deg = torch.maximum(overshoot_deg, torch.nan_to_num(over, nan=0.0))
            stopped = (torch.hypot(vel[:, 0], vel[:, 1]) < stop_v) & (omega.abs() < stop_w) & ~running
            t_stop = torch.where(torch.isnan(t_stop) & stopped, torch.full_like(t_stop, t), t_stop)
            for i in trace:
                trace[i].append((t, float(pc[i, 0]), float(pc[i, 1]), float(yaw[i])))
            if record_poses:
                poses.append(torch.cat([pc, yaw.unsqueeze(1)], dim=1).detach().cpu())

        cmd = delay.push_and_get(held)
        force, torque = model.step(vel, yaw, omega, cmd, dt)
        bad = ~torch.isfinite(force).all(dim=1) | ~torch.isfinite(torque)
        nan_flag |= bad
        body.step(torch.nan_to_num(force), torch.nan_to_num(torque))
        rf = running.to(DTYPE)
        slip_steps += model.slipping.to(DTYPE) * rf
        path_steps += rf

    pos_cm, yaw, vel, omega = body.state()
    pc = center_pose(pos_cm, yaw, model.cg)
    final_pos = torch.hypot(pc[:, 0] - end[:, 0], pc[:, 1] - end[:, 1])
    final_yaw = torch.rad2deg(wrap(yaw - end[:, 2]).abs())
    nan_flag |= ~torch.isfinite(final_pos) | ~torch.isfinite(final_yaw)

    rev_t = torch.maximum(zz[0].count, zz[1].count)
    rev_r = zz[2].count
    max_yaw_deg = torch.rad2deg(max_yaw)
    abort_t = (max_pos > st["abort_pos_err_m"]) | nan_flag
    abort_r = (max_yaw_deg > st["abort_yaw_err_deg"]) | nan_flag
    # Inestable: divergencia en cualquier prueba, u oscilacion en la prueba mantener_pose.
    unstable_t = (max_pos > st["divergence_pos_err_m"]) | nan_flag | (is_hold & (rev_t > st["hold_max_reversals"]))
    unstable_r = (max_yaw_deg > st["divergence_yaw_err_deg"]) | nan_flag | (is_hold & (rev_r > st["hold_max_reversals"]))

    out = {
        "lateral_rms_m": torch.sqrt(lat_sq / torch.clamp(n_lat, min=1)),
        "pos_rms_m": torch.sqrt(pos_sq / torch.clamp(n_pos, min=1)),
        "yaw_rms_deg": torch.rad2deg(torch.sqrt(yaw_sq / torch.clamp(n_pos, min=1))),
        "max_pos_err_m": max_pos,
        "max_yaw_err_deg": max_yaw_deg,
        "final_pos_err_m": final_pos,
        "final_yaw_err_deg": final_yaw,
        "overshoot_m": overshoot_m,
        "overshoot_deg": overshoot_deg,
        "path_time_s": T,
        "total_time_s": t_stop,
        "slip_frac": slip_steps / torch.clamp(path_steps, min=1),
        "reversals_t": rev_t,
        "reversals_r": rev_r,
        "abort": abort_t | abort_r,
        "unstable_t": unstable_t,
        "unstable_r": unstable_r,
        "unstable": unstable_t | unstable_r,
        "wheel_lift": model.wheel_lift.clone(),
    }
    out = {k: v.detach().cpu() for k, v in out.items()}
    if record:
        out["trace"] = trace
    if record_poses:
        out["poses"] = torch.stack(poses)
    return out


def run_trajectory(cfg: dict, variant: dict, env: EnvParams, traj: Trajectory, body=None, device="cpu",
                   record: list[int] | None = None) -> dict:
    """Todos los entornos siguen la misma trayectoria."""
    return run_batch(cfg, variant, env, [traj], torch.zeros(env.n, dtype=torch.long), device, body, record)

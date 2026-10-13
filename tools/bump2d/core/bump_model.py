"""Cruce del BUMP visto de lado (plano de pitch), vectorizado en numpy sobre muchos escenarios.

Cuerpo rigido con 3 grados de libertad (x, z, pitch). Contactos con el terreno:
  - "ejes" de ruedas: las ruedas que quedan a la misma distancia a lo largo del avance se apoyan
    juntas (recto: 2 ejes de 2 ruedas; en diamante: 1 adelante, 2 al centro, 1 atras). Cada eje es un
    circulo de radio R contra el
    perfil del bump, resorte-amortiguador normal (compliancia de la llanta), friccion por
    deslizamiento acotada por mu*N, y la velocidad de la rueda como estado (motor con lazo de
    velocidad, kS/kV/kP del robot, limite de stator).
  - La parte de abajo del robot (silueta del CAD: panza, modulos, bumpers, intake): puntos que, si
    tocan el bump, empujan con un resorte duro y rozan con friccion mu_body. Asi aparece el
    "quedarse colgado" (high-centering) y el raspado de bumpers.

Marco: x horizontal en el sentido de avance, z hacia arriba. Marco del cuerpo: origen en el centro de
modulos a la altura del piso (con el robot plano), s hacia adelante.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class Bump:
    """Loma simetrica (MARS cancha_2026.py: sube 16.5 cm en 56 cm y baja igual)."""

    height: float = 0.165
    half_width: float = 0.56
    x0: float = 1.0  # donde empieza la subida

    def segments(self):
        """Segmentos del perfil (x0, z0, x1, z1): piso, subida, bajada, piso."""
        a, b, c = self.x0, self.x0 + self.half_width, self.x0 + 2 * self.half_width
        return np.array([[a - 50.0, 0.0, a, 0.0], [a, 0.0, b, self.height], [b, self.height, c, 0.0], [c, 0.0, c + 50.0, 0.0]])

    def height_at(self, x):
        a, w, h = self.x0, self.half_width, self.height
        return np.clip(h - np.abs(x - (a + w)) * h / w, 0.0, None)

    def slope_at(self, x):
        a, w, h = self.x0, self.half_width, self.height
        up = (x > a) & (x < a + w)
        down = (x >= a + w) & (x < a + 2 * w)
        return np.where(up, h / w, np.where(down, -h / w, 0.0))

    @property
    def end(self):
        return self.x0 + 2 * self.half_width


@dataclass
class Robot:
    mass: float = 57.0
    pitch_inertia: float = 5.0  # TODO estimado: caja de 0.9 m x 0.5 m con 57 kg
    cg_height: float = 0.20  # TODO supuesto (mismo que tools/pid_sweep)
    cg_s: float = 0.0
    wheel_radius: float = 0.0508
    half_base: float = 0.2764
    # (s, numero de ruedas) de cada eje; None = 2 ejes de 2 ruedas en +-half_base
    axles: list | None = None
    # silueta inferior: puntos (s, z) en el marco del cuerpo
    under_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    under_z: np.ndarray = field(default_factory=lambda: np.zeros(0))


@dataclass
class Drive:
    """Motor de traccion por eje (2 ruedas). Valores de TunerConstants / tools/pid_sweep."""

    gear: float = 5.2734375
    stall_torque: float = 9.37
    stall_current: float = 483.0
    free_rpm: float = 5800.0
    free_current: float = 2.0
    stator_limit: float = 70.0
    kS: float = 0.1295
    kV: float = 0.1156
    kP: float = 0.2
    wheel_inertia: float = 8e-4  # kg m2 por rueda (rotor reflejado + rueda), TODO estimado


@dataclass
class Contact:
    tire_k: float = 1.5e5  # N/m por llanta
    tire_zeta: float = 0.3
    slip_c: float = 750.0  # N/(m/s) por llanta: regulariza la friccion de la llanta
    mu_tire: float = 1.1  # TODO COF real del Spike Grip
    body_k: float = 4e5  # N/m por punto de la silueta que toca
    body_c: float = 2000.0
    mu_body: float = 0.3  # aluminio/policarbonato contra el bump


def _closest_on_segments(px, pz, segs):
    """Punto mas cercano de (px, pz) (forma (B,)) al perfil. Devuelve (dist, cx, cz)."""
    best_d = np.full(px.shape, np.inf)
    best_x = np.zeros_like(px)
    best_z = np.zeros_like(px)
    for x0, z0, x1, z1 in segs:
        dx, dz = x1 - x0, z1 - z0
        t = np.clip(((px - x0) * dx + (pz - z0) * dz) / (dx * dx + dz * dz), 0.0, 1.0)
        cx, cz = x0 + t * dx, z0 + t * dz
        d = np.hypot(px - cx, pz - cz)
        better = d < best_d
        best_d = np.where(better, d, best_d)
        best_x = np.where(better, cx, best_x)
        best_z = np.where(better, cz, best_z)
    return best_d, best_x, best_z


def simulate(robots: list[Robot], v_cmd: np.ndarray, bump: Bump, drive: Drive = Drive(),
             contact: Contact = Contact(), dt: float = 1e-4, t_max: float = 8.0, record_every: int = 100):
    """Simula B escenarios en paralelo. robots[i] y v_cmd[i] por escenario. Devuelve (metricas, trazas)."""
    B = len(robots)
    v_cmd = np.asarray(v_cmd, dtype=float)
    m = np.array([r.mass for r in robots])
    I = np.array([r.pitch_inertia for r in robots])
    R = robots[0].wheel_radius
    a = np.array([r.half_base for r in robots])
    cg = np.stack([np.array([r.cg_s for r in robots]), np.array([r.cg_height for r in robots])], 1)  # (B,2)
    P = max(len(r.under_s) for r in robots)
    us = np.full((B, P), np.nan)
    uz = np.full((B, P), np.nan)
    for i, r in enumerate(robots):
        us[i, : len(r.under_s)] = r.under_s
        uz[i, : len(r.under_z)] = r.under_z
    valid = ~np.isnan(us)
    us = np.nan_to_num(us)
    uz = np.nan_to_num(uz, nan=10.0)
    axle_lists = [r.axles if r.axles is not None else [(r.half_base, 2), (-r.half_base, 2)] for r in robots]
    A = max(len(ax) for ax in axle_lists)
    wheel_s = np.zeros((B, A))
    nw = np.zeros((B, A))  # ruedas por eje (0 = eje de relleno)
    for i, ax in enumerate(axle_lists):
        for j, (sj, nj) in enumerate(sorted(ax, key=lambda q: -q[0])):  # de adelante hacia atras
            wheel_s[i, j] = sj
            nw[i, j] = nj
    wheel_z = np.full((B, A), R)
    total_w = nw.sum(1)
    front_extent = np.where(valid, us, -np.inf).max(1)
    rear_extent = np.where(valid, us, np.inf).min(1)

    segs = bump.segments()
    # Estado: CM en el mundo, pitch (positivo = nariz arriba), velocidades, velocidad de las ruedas.
    x = bump.x0 - front_extent - 0.15 + cg[:, 0]
    z = cg[:, 1].copy()
    th = np.zeros(B)
    vx = v_cmd.copy()
    vz = np.zeros(B)
    w = np.zeros(B)
    u = np.repeat(v_cmd[:, None], A, 1)  # velocidad de superficie de cada eje
    m_eq = np.maximum(nw, 1) * drive.wheel_inertia / R**2  # masa equivalente de las ruedas de un eje
    k_ax = contact.tire_k * nw
    c_ax = 2 * contact.tire_zeta * np.sqrt(k_ax * m[:, None] * nw / np.maximum(total_w, 1)[:, None])

    kt = drive.stall_torque / drive.stall_current
    res = 12.0 / drive.stall_current
    kv_rad = drive.free_rpm * 2 * math.pi / 60 / (12.0 - res * drive.free_current)
    to_rps = drive.gear / (2 * math.pi * R)
    rps_t = v_cmd * to_rps
    v_drive = np.clip(np.sign(rps_t) * drive.kS + drive.kV * rps_t, -12, 12)

    def to_world(s, zb):
        c, sn = np.cos(th)[:, None], np.sin(th)[:, None]
        rs, rz = s - cg[:, 0:1], zb - cg[:, 1:2]
        return x[:, None] + c * rs - sn * rz, z[:, None] + sn * rs + c * rz, rs, rz

    # metricas
    x_start = x.copy()
    odo = np.zeros(B)
    max_pitch = np.zeros(B)
    min_pitch = np.zeros(B)
    body_contact_time = np.zeros(B)
    body_max_force = np.zeros(B)
    airborne_time = np.zeros(B)
    axle_lift_time = np.zeros((B, A))
    max_axle_force = np.zeros(B)
    stuck_time = np.zeros(B)
    min_speed = v_cmd.copy()
    done = np.zeros(B, bool)
    t_done = np.full(B, np.nan)
    x_end = np.full(B, np.nan)  # x cuando termina cada escenario (para la odometria)
    first_contact_body_x = np.full(B, np.nan)
    trace = {k: [] for k in ("t", "x", "z", "th", "normals", "body", "bodyx")}

    n_steps = int(t_max / dt)
    for k in range(n_steps):
        c, sn = np.cos(th), np.sin(th)
        # --- llantas ---
        wx, wz, wrs, wrz = to_world(wheel_s, wheel_z)
        Fx = np.zeros(B)
        Fz = -m * G
        Tq = np.zeros(B)
        normals = np.zeros((B, A))
        for j in range(A):
            d, qx, qz = _closest_on_segments(wx[:, j], wz[:, j], segs)
            pen = R - d
            nx = (wx[:, j] - qx) / np.maximum(d, 1e-9)
            nz = (wz[:, j] - qz) / np.maximum(d, 1e-9)
            # velocidad del centro de la rueda
            pvx = vx - w * (sn * wrs[:, j] + c * wrz[:, j])
            pvz = vz + w * (c * wrs[:, j] - sn * wrz[:, j])
            vn = pvx * nx + pvz * nz
            N = np.where((pen > 0) & (nw[:, j] > 0), np.maximum(k_ax[:, j] * pen - c_ax[:, j] * vn, 0.0), 0.0)
            tx, tz = nz, -nx  # tangente del terreno hacia adelante
            vt = pvx * tx + pvz * tz
            Ff = np.clip(contact.slip_c * nw[:, j] * (u[:, j] - vt), -contact.mu_tire * N, contact.mu_tire * N)
            Ff = np.where(N > 0, Ff, 0.0)
            # motor (por eje: 2 motores)
            rps = u[:, j] * to_rps
            volts = np.clip(v_drive + drive.kP * (rps_t - rps), -12, 12)
            cur = np.clip((volts - u[:, j] / R * drive.gear / kv_rad) / res, -drive.stator_limit, drive.stator_limit)
            F_motor = nw[:, j] * kt * cur * drive.gear / R
            u[:, j] += (F_motor - Ff) / m_eq[:, j] * dt
            fx = N * nx + Ff * tx
            fz = N * nz + Ff * tz
            # punto de aplicacion: contacto con el piso (centro de rueda - R*n)
            ax_, az_ = wx[:, j] - R * nx - x, wz[:, j] - R * nz - z
            Fx += fx
            Fz += fz
            Tq += ax_ * fz - az_ * fx
            normals[:, j] = N
            odo += np.where(done, 0.0, u[:, j] * dt * nw[:, j] / np.maximum(total_w, 1))
        # --- silueta inferior contra el bump ---
        bx_, bz_, brs, brz = to_world(us, uz)
        hb = bump.height_at(bx_)
        pen_b = np.where(valid, hb - bz_, -1.0)
        touching = pen_b > 0
        slope = bump.slope_at(bx_)
        nlen = np.sqrt(1 + slope**2)
        nbx, nbz = -slope / nlen, 1 / nlen
        bvx = vx[:, None] - w[:, None] * (sn[:, None] * brs + c[:, None] * brz)
        bvz = vz[:, None] + w[:, None] * (c[:, None] * brs - sn[:, None] * brz)
        bvn = bvx * nbx + bvz * nbz
        Nb = np.where(touching, np.maximum(contact.body_k * pen_b - contact.body_c * bvn, 0.0), 0.0)
        tbx, tbz = nbz, -nbx
        bvt = bvx * tbx + bvz * tbz
        Fb_t = -contact.mu_body * Nb * np.tanh(bvt / 0.01)
        fbx = Nb * nbx + Fb_t * tbx
        fbz = Nb * nbz + Fb_t * tbz
        rax, raz = bx_ - x[:, None], bz_ - z[:, None]
        Fx += fbx.sum(1)
        Fz += fbz.sum(1)
        Tq += (rax * fbz - raz * fbx).sum(1)
        body_force = Nb.sum(1)

        # --- integracion (Euler semi-implicito) ---
        vx += Fx / m * dt
        vz += Fz / m * dt
        w += Tq / I * dt
        x += vx * dt
        z += vz * dt
        th += w * dt

        # --- metricas ---
        live = ~done
        pitch_deg = np.degrees(th)
        max_pitch = np.where(live, np.maximum(max_pitch, pitch_deg), max_pitch)
        min_pitch = np.where(live, np.minimum(min_pitch, pitch_deg), min_pitch)
        on_bump = (x + front_extent > bump.x0) & (x + rear_extent < bump.end)
        in_window = live & on_bump
        body_contact_time += np.where(in_window & (body_force > 0), dt, 0.0)
        body_max_force = np.maximum(body_max_force, np.where(live, body_force, 0.0))
        first_contact_body_x = np.where(np.isnan(first_contact_body_x) & (body_force > 0) & live,
                                        x, first_contact_body_x)
        lifted = (normals <= 0) & (nw > 0)
        axle_lift_time += np.where((in_window)[:, None] & lifted, dt, 0.0)
        airborne_time += np.where(in_window & ((normals <= 0) | (nw == 0)).all(1), dt, 0.0)
        max_axle_force = np.maximum(max_axle_force, np.where(live, normals.max(1), 0.0))
        min_speed = np.where(in_window, np.minimum(min_speed, vx), min_speed)
        stuck_time = np.where(in_window & (np.abs(vx) < 0.05), stuck_time + dt, 0.0)
        crossed = live & (x + rear_extent > bump.end + 0.05)
        t_done = np.where(crossed, (k + 1) * dt, t_done)
        newly = live & (crossed | (stuck_time > 1.0))
        x_end = np.where(newly, x, x_end)
        done |= newly

        if record_every and k % record_every == 0:
            t = k * dt
            trace["t"].append(t)
            trace["x"].append(x.copy())
            trace["z"].append(z.copy())
            trace["th"].append(th.copy())
            trace["normals"].append(normals.copy())
            trace["body"].append(body_force.copy())
            # posicion (en s del cuerpo) del punto de la silueta que mas empuja
            idx = np.argmax(np.where(touching, Nb, -1), 1)
            trace["bodyx"].append(np.where(body_force > 0, us[np.arange(B), idx], np.nan))
        if done.all():
            break

    horiz = np.where(np.isnan(x_end), x, x_end) - x_start
    metrics = {
        "crossed": ~np.isnan(t_done),
        "stuck": stuck_time > 1.0,
        "time_s": t_done,
        "max_pitch_deg": max_pitch,
        "min_pitch_deg": min_pitch,
        "body_contact_s": body_contact_time,
        "body_max_force_n": body_max_force,
        "airborne_s": airborne_time,
        "front_lift_s": axle_lift_time[:, 0],
        "rear_lift_s": axle_lift_time[np.arange(B), (nw > 0).sum(1) - 1],
        "max_axle_force_n": max_axle_force,
        "min_speed_mps": min_speed,
        "odometry_error_m": odo - horiz,
    }
    trace = {k: np.array(v) for k, v in trace.items()}
    return metrics, trace

"""Modelo de swerve vectorizado en torch puro.

Convenciones:
  - Marco de campo: x, y, yaw (rad, CCW). Marco del robot: x adelante, y izquierda.
  - "Centro" = centro geometrico de los modulos (lo que reporta la odometria / PathPlanner).
  - "CM" = centro de masa, desplazado cg_offset del centro. El cuerpo rigido se integra en el CM.
  - Orden de modulos: FL, FR, BL, BR (igual que TunerConstants).

Por cada modulo y cada paso de fisica (dt = 2 ms):
  1. Cinematica inversa del comando de chasis (ChassisSpeeds robot-relative) + desaturacion.
  2. Optimizacion del angulo (si el error es > 90 grados, invierte la rueda) y steer con
     limite de velocidad + primer orden.
  3. Voltaje del drive: open loop (V = 12 * v / kSpeedAt12Volts) o lazo de velocidad
     (kS + kV + kP sobre rps del rotor), saturado a la bateria.
  4. Motor DC (Kraken X60 FOC): I = (V - fcem)/R, con limite de stator y de supply, mas
     friccion de Coulomb (static_friction_v). La "velocidad real" se modela bajando Kv:
     a 12 V llega a speed_factor * 5.85 m/s.
  5. Fuerza longitudinal = Kt*I*G/r; fuerza lateral por angulo de deslizamiento
     (C_alpha proporcional a la carga normal).
  6. Carga normal por modulo: reparto estatico por el offset del CM + transferencia por
     aceleracion (altura del CM). Circulo de friccion |F| <= mu*N.
  7. Suma de fuerzas y torque alrededor del CM.
"""

from __future__ import annotations

import math

import torch

from .params import DTYPE, EnvParams


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


def rotate(x: torch.Tensor, y: torch.Tensor, ang: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    c, s = torch.cos(ang), torch.sin(ang)
    return c * x - s * y, s * x + c * y


class SwerveModel:
    """Calcula la fuerza (marco del robot) y el torque sobre el CM para cada entorno."""

    def __init__(self, cfg: dict, variant: dict, env: EnvParams, device="cpu"):
        self.device = device
        self.env = env.to(device)
        self.n = env.n
        r, m, code = cfg["robot"], cfg["drive_motor"], cfg["code"]
        kw = dict(dtype=DTYPE, device=device)

        a = float(r["module_half_spacing_m"])
        self.mod = torch.tensor([[a, a], [a, -a], [-a, a], [-a, -a]], **kw)  # (4,2)
        self.wheelbase = 2 * a
        self.trackwidth = 2 * a
        self.r_w = float(r["wheel_radius_m"])
        self.gear = float(r["drive_gear_ratio"])

        v_nom = float(m["nominal_voltage"])
        self.kt = float(m["stall_torque_nm"]) / float(m["stall_current_a"])
        self.res = v_nom / float(m["stall_current_a"])
        free_rad = float(m["free_speed_rpm"]) * 2 * math.pi / 60
        self.i_free = float(m["free_current_a"])
        self.kv = free_rad / (v_nom - self.res * self.i_free)  # rad/s por volt
        # Friccion de Coulomb: fuerza en la rueda equivalente a static_friction_v volts de stall.
        fs_v = float(m.get("static_friction_v", 0.0))
        self.f_static = self.kt * (fs_v / self.res) * self.gear / float(r["wheel_radius_m"])
        # Para que speed_factor siga siendo "velocidad real a 12 V / 5.85", la friccion no debe
        # restar velocidad adicional: se compensa en Kv.
        self.kv *= v_nom / (v_nom - fs_v)

        self.v12 = float(code["speed_at_12v_mps"])
        self.desaturate = bool(code["desaturate_wheel_speeds"])
        self.cos_comp = bool(code["cosine_compensation"])
        self.ks, self.kv_code, self.kp_drive = float(code["drive_kS"]), float(code["drive_kV"]), float(code["drive_kP"])

        self.mode = variant["drive_mode"]
        if self.mode not in ("open_loop", "velocity"):
            raise ValueError(f"drive_mode invalido: {self.mode}")
        self.stator = float(variant["stator_limit_a"]) if variant.get("stator_limit_a") else math.inf
        self.supply = float(variant["supply_limit_a"]) if variant.get("supply_limit_a") else math.inf

        self.steer_rate = float(cfg["steer"]["max_rate_rad_s"])
        self.steer_tau = float(cfg["steer"]["time_constant_s"])
        self.c_alpha = float(cfg["tire"]["cornering_stiffness_per_load"])
        self.v_reg = float(cfg["tire"]["low_speed_regularization_mps"])
        self.vbat = float(cfg["sim"]["battery_voltage"])
        self.g = float(cfg["sim"]["gravity_mps2"])

        e = self.env
        self.cg = torch.stack([e.cg_x, e.cg_y], dim=-1)  # (N,2)
        self.rm = self.mod.unsqueeze(0) - self.cg.unsqueeze(1)  # (N,4,2) modulo relativo al CM
        sx, sy = torch.sign(self.mod[:, 0]), torch.sign(self.mod[:, 1])
        self.sx, self.sy = sx, sy
        # Reparto estatico bilineal: fraccion adelante = 0.5 + cg_x/L, izquierda = 0.5 + cg_y/T.
        fx = 0.5 + sx.unsqueeze(0) * e.cg_x.unsqueeze(1) / self.wheelbase
        fy = 0.5 + sy.unsqueeze(0) * e.cg_y.unsqueeze(1) / self.trackwidth
        self.n_static = (e.mass * self.g).unsqueeze(1) * fx * fy  # (N,4)
        self.reset()

    def reset(self):
        kw = dict(dtype=DTYPE, device=self.device)
        self.steer = torch.zeros(self.n, 4, **kw)
        self.last_target_angle = torch.zeros(self.n, 4, **kw)
        self.acc_b = torch.zeros(self.n, 2, **kw)
        self.slipping = torch.zeros(self.n, dtype=torch.bool, device=self.device)
        self.wheel_lift = torch.zeros(self.n, dtype=torch.bool, device=self.device)
        self.normal = self.n_static.clone()
        self.current = torch.zeros(self.n, 4, **kw)

    def module_targets(self, cmd: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """ChassisSpeeds robot-relative (N,3) -> velocidad (N,4) y angulo (N,4) por modulo."""
        vx = cmd[:, 0:1] - cmd[:, 2:3] * self.mod[:, 1]
        vy = cmd[:, 1:2] + cmd[:, 2:3] * self.mod[:, 0]
        speed = torch.hypot(vx, vy)
        if self.desaturate:  # SwerveDriveKinematics.desaturateWheelSpeeds(states, kSpeedAt12Volts)
            top = speed.max(dim=1, keepdim=True).values
            speed = speed * torch.clamp(self.v12 / torch.clamp(top, min=1e-9), max=1.0)
        # Con velocidad ~0, WPILib conserva el ultimo angulo.
        angle = torch.where(speed > 1e-6, torch.atan2(vy, vx), self.last_target_angle)
        self.last_target_angle = angle
        return speed, angle

    def step(self, vel_w: torch.Tensor, yaw: torch.Tensor, omega: torch.Tensor, cmd: torch.Tensor, dt: float):
        """vel_w (N,2) velocidad del CM en campo, yaw (N,), omega (N,), cmd (N,3) robot-relative.

        Devuelve (fuerza (N,2) en marco del robot, torque (N,) alrededor del CM).
        """
        e = self.env
        speed_t, angle_t = self.module_targets(cmd)

        # Optimizacion (SwerveModuleState.optimize) y steer.
        err = wrap(angle_t - self.steer)
        flip = err.abs() > math.pi / 2
        angle_t = torch.where(flip, angle_t + math.pi, angle_t)
        speed_t = torch.where(flip, -speed_t, speed_t)
        err = wrap(angle_t - self.steer)
        rate = torch.clamp(err / self.steer_tau, -self.steer_rate, self.steer_rate)
        step = rate * dt
        step = torch.where(step.abs() > err.abs(), err, step)  # no pasarse del objetivo
        self.steer = wrap(self.steer + step)
        if self.cos_comp:
            speed_t = speed_t * torch.cos(wrap(angle_t - self.steer))

        # Velocidad del piso en cada modulo (marco del robot).
        vbx, vby = rotate(vel_w[:, 0], vel_w[:, 1], -yaw)
        vmx = vbx.unsqueeze(1) - omega.unsqueeze(1) * self.rm[..., 1]
        vmy = vby.unsqueeze(1) + omega.unsqueeze(1) * self.rm[..., 0]
        ux, uy = torch.cos(self.steer), torch.sin(self.steer)
        v_long = vmx * ux + vmy * uy
        v_lat = -vmx * uy + vmy * ux

        # Voltaje del drive.
        if self.mode == "open_loop":
            volts = self.vbat * speed_t / self.v12
        else:
            to_rps = self.gear / (2 * math.pi * self.r_w)
            rps_t, rps = speed_t * to_rps, v_long * to_rps
            ks = torch.where(rps_t.abs() > 1e-3, torch.sign(rps_t) * self.ks, torch.zeros_like(rps_t))
            volts = ks + self.kv_code * rps_t + self.kp_drive * (rps_t - rps)
        volts = torch.clamp(volts, -self.vbat, self.vbat)

        # Motor DC con limites de corriente. speed_factor baja Kv (fcem mas alta).
        w_rotor = v_long / self.r_w * self.gear
        current = (volts - w_rotor / (self.kv * e.speed_factor.unsqueeze(1))) / self.res
        current = torch.clamp(current, -self.stator, self.stator)
        if math.isfinite(self.supply):
            i_sup = (current * volts).abs() / self.vbat
            current = current * torch.clamp(self.supply / torch.clamp(i_sup, min=1e-9), max=1.0)
        self.current = current
        # La corriente libre (friccion interna del motor) no produce fuerza en la rueda.
        i_fric = torch.clamp(self.i_free * w_rotor / 10.0, -self.i_free, self.i_free)
        f_long = self.kt * (current - i_fric) * self.gear / self.r_w
        f_long = f_long - self.f_static * torch.tanh(v_long / 0.02)
        # Carga normal: estatica + transferencia por la aceleracion del paso anterior.
        h = e.cg_height.unsqueeze(1)
        m = e.mass.unsqueeze(1)
        normal = (
            self.n_static
            - self.sx * m * self.acc_b[:, 0:1] * h / (2 * self.wheelbase)
            - self.sy * m * self.acc_b[:, 1:2] * h / (2 * self.trackwidth)
        )
        self.wheel_lift |= (normal < 0).any(dim=1)
        normal = torch.clamp(normal, min=0.0)
        self.normal = normal

        # Fuerza lateral por angulo de deslizamiento (lineal, regularizada a baja velocidad).
        v_mod = torch.clamp(torch.hypot(v_long, v_lat), min=self.v_reg)
        f_lat = -self.c_alpha * normal * v_lat / v_mod

        # Circulo de friccion.
        fx = f_long * ux - f_lat * uy
        fy = f_long * uy + f_lat * ux
        fmag = torch.hypot(fx, fy)
        cap = e.cof.unsqueeze(1) * normal
        scale = torch.clamp(cap / torch.clamp(fmag, min=1e-9), max=1.0)
        self.slipping = (scale < 0.999).any(dim=1)
        fx, fy = fx * scale, fy * scale

        force = torch.stack([fx.sum(dim=1), fy.sum(dim=1)], dim=-1)
        torque = (self.rm[..., 0] * fy - self.rm[..., 1] * fx).sum(dim=1)
        self.acc_b = force / e.mass.unsqueeze(1)
        return force, torque


class TorchBody:
    """Cuerpo rigido plano integrado en torch (Euler semi-implicito)."""

    def __init__(self, env: EnvParams, dt: float, device="cpu"):
        self.env = env.to(device)
        self.dt = dt
        self.device = device
        self.cg = torch.stack([self.env.cg_x, self.env.cg_y], dim=-1)

    def reset(self, center_xy: torch.Tensor, yaw: torch.Tensor):
        cx, cy = rotate(self.cg[:, 0], self.cg[:, 1], yaw)
        self.pos = center_xy + torch.stack([cx, cy], dim=-1)
        self.yaw = yaw.clone()
        self.vel = torch.zeros_like(center_xy)
        self.omega = torch.zeros_like(yaw)

    def state(self):
        """(pos_cm (N,2), yaw (N,), vel_cm (N,2), omega (N,)) en campo."""
        return self.pos, self.yaw, self.vel, self.omega

    def step(self, force_b: torch.Tensor, torque: torch.Tensor):
        fx, fy = rotate(force_b[:, 0], force_b[:, 1], self.yaw)
        self.vel = self.vel + torch.stack([fx, fy], dim=-1) / self.env.mass.unsqueeze(1) * self.dt
        self.omega = self.omega + torque / self.env.moi * self.dt
        self.pos = self.pos + self.vel * self.dt
        self.yaw = self.yaw + self.omega * self.dt


def center_pose(pos_cm: torch.Tensor, yaw: torch.Tensor, cg: torch.Tensor) -> torch.Tensor:
    """Posicion del centro geometrico (lo que ve la odometria) a partir de la del CM."""
    cx, cy = rotate(cg[:, 0], cg[:, 1], yaw)
    return pos_cm - torch.stack([cx, cy], dim=-1)

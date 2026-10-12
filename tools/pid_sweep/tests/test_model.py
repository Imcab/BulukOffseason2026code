"""Modelo de modulo y chasis: velocidad libre, reparto de carga, traccion, limites de corriente."""

import copy
import math

import pytest
import torch

from core.model import SwerveModel, TorchBody
from core.params import nominal_env_params, variant
from core.sim import run_trajectory

DT = 0.002


def drive(cfg, var, env, cmd_fn, seconds):
    """Aplica cmd_fn(t) -> (vx, vy, w) robot-relative en lazo abierto de chasis (sin PathPlanner)."""
    model = SwerveModel(cfg, var, env)
    body = TorchBody(env, DT)
    body.reset(torch.zeros(env.n, 2), torch.zeros(env.n))
    hist = []
    for k in range(int(seconds / DT)):
        pos, yaw, vel, om = body.state()
        cmd = torch.tensor([cmd_fn(k * DT)], dtype=torch.float32).expand(env.n, 3)
        f, tq = model.step(vel, yaw, om, cmd, DT)
        body.step(f, tq)
        hist.append((vel.clone(), f.clone()))
    return model, body, hist


def test_drive_kv_matches_phase1_calc(cfg):
    r = cfg["robot"]
    rotor_rps = cfg["code"]["speed_at_12v_mps"] / (2 * math.pi * r["wheel_radius_m"]) * r["drive_gear_ratio"]
    assert 12 / rotor_rps == pytest.approx(cfg["code"]["drive_kV"], abs=0.001)
    # La Kraken X60 FOC libre (5800 rpm) da exactamente el kSpeedAt12Volts teorico.
    free = cfg["drive_motor"]["free_speed_rpm"] / 60 / r["drive_gear_ratio"] * 2 * math.pi * r["wheel_radius_m"]
    assert free == pytest.approx(cfg["code"]["speed_at_12v_mps"], rel=0.002)


@pytest.mark.parametrize("sf", [1.0, 0.85])
def test_open_loop_free_speed(cfg, sf):
    env = nominal_env_params(cfg, 0, 0, cof=1.6, speed_factor=sf)
    _, body, _ = drive(cfg, variant(cfg, "actual_openloop"), env, lambda t: (5.85, 0, 0), 4.0)
    v = float(torch.hypot(body.vel[0, 0], body.vel[0, 1]))
    assert v == pytest.approx(5.85 * sf, rel=0.01)


def test_static_load_split(cfg):
    env = nominal_env_params(cfg, 0, 0)
    m = SwerveModel(cfg, variant(cfg), env)
    n = m.n_static[0]
    w = float(env.mass[0]) * cfg["sim"]["gravity_mps2"]
    assert float(n.sum()) == pytest.approx(w, rel=1e-5)
    left = float(n[0] + n[2]) / w  # FL + BL
    front = float(n[0] + n[1]) / w
    assert left == pytest.approx(0.5 + 0.042 / 0.55245, abs=1e-4)  # ~0.576 (57/43)
    assert front == pytest.approx(0.5 + 0.005 / 0.55245, abs=1e-4)


def test_load_transfer_to_rear_when_accelerating(cfg):
    env = nominal_env_params(cfg, 0, 0, cof=1.6)
    model, _, _ = drive(cfg, variant(cfg), env, lambda t: (3.0, 0, 0), 0.15)
    n = model.normal[0]
    assert float(n[2] + n[3]) > float(n[0] + n[1])
    assert float(n.sum()) == pytest.approx(float(env.mass[0]) * 9.81, rel=1e-3)


@pytest.mark.parametrize("cof", [0.9, 1.3])
def test_traction_limits_acceleration(cfg, cof):
    env = nominal_env_params(cfg, 0, 0, cof=cof)
    _, _, hist = drive(cfg, variant(cfg), env, lambda t: (5.85, 0, 0), 0.4)
    acc = max(float(f[0, 0]) for _, f in hist) / float(env.mass[0])
    assert acc <= cof * 9.81 * 1.001
    assert acc >= cof * 9.81 * 0.85  # con 120 A de stator la traccion es el limite


def test_stator_limit_caps_force(cfg):
    var = copy.deepcopy(variant(cfg))
    var["stator_limit_a"] = 40.0
    env = nominal_env_params(cfg, 0, 0, cof=3.0)
    _, _, hist = drive(cfg, var, env, lambda t: (5.85, 0, 0), 0.3)
    r, g = cfg["robot"]["wheel_radius_m"], cfg["robot"]["drive_gear_ratio"]
    kt = cfg["drive_motor"]["stall_torque_nm"] / cfg["drive_motor"]["stall_current_a"]
    f_max = 4 * kt * 40 * g / r
    assert max(float(f[0, 0]) for _, f in hist) <= f_max * 1.001


def test_velocity_mode_beats_open_loop_when_slow(cfg):
    env = nominal_env_params(cfg, 0, 0, cof=1.6, speed_factor=0.85)
    _, b_ol, _ = drive(cfg, variant(cfg, "actual_openloop"), env, lambda t: (3.0, 0, 0), 3.0)
    _, b_v, _ = drive(cfg, variant(cfg, "velocity"), env, lambda t: (3.0, 0, 0), 3.0)
    v_ol, v_v = float(b_ol.vel[0, 0]), float(b_v.vel[0, 0])
    # open loop: ~85% y algo menos por la friccion (OpenLoopVoltage no tiene kS)
    assert 3.0 * 0.85 * 0.92 < v_ol < 3.0 * 0.85 * 1.01
    assert abs(v_v - 3.0) < abs(v_ol - 3.0) / 2  # lazo de velocidad corrige buena parte


def test_rest_stays_at_rest(cfg):
    env = nominal_env_params(cfg, 0, 0)
    _, body, _ = drive(cfg, variant(cfg), env, lambda t: (0, 0, 0), 1.0)
    assert float(body.pos.abs().max()) < 0.06  # solo el offset inicial del CM
    assert float(body.vel.abs().max()) < 1e-6


def test_rotation_in_place(cfg):
    env = nominal_env_params(cfg, 0, 0, cof=1.6)
    _, body, _ = drive(cfg, variant(cfg, "velocity"), env, lambda t: (0, 0, 3.0), 2.0)
    assert float(body.omega[0]) == pytest.approx(3.0, rel=0.05)  # kS compensa la friccion
    # El centro de modulos casi no se traslada (el CM gira alrededor de el).
    from core.model import center_pose
    c = center_pose(body.pos, body.yaw, body.cg)
    assert float(torch.hypot(c[0, 0], c[0, 1])) < 0.1


def test_tire_stiffness_insensitive(cfg, trajs):
    """El resultado no depende mucho de la rigidez lateral supuesta (parametro no medido)."""
    out = []
    for k in (15.0, 30.0):
        c = copy.deepcopy(cfg)
        c["tire"]["cornering_stiffness_per_load"] = k
        env = nominal_env_params(c, 6.0, 6.0, latency_s=0.04)
        r = run_trajectory(c, variant(c), env, trajs["recta_6m_giro180"], TorchBody(env, DT))
        out.append(float(r["pos_rms_m"][0]))
    assert out[1] == pytest.approx(out[0], rel=0.25)

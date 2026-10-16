"""Prueba de sanidad pedida: sin PID, el tiempo de un path con kSpeedAt12Volts y los constraints
dados cuadra con lo esperado (calculo a mano), y el robot simulado llega cerca del final."""

import math

import pytest

from core.model import TorchBody
from core.params import nominal_env_params, variant
from core.sim import run_trajectory


def trapezoid_time(d, v, a):
    """Tiempo minimo de una recta con perfil trapezoidal (o triangular) desde y hasta reposo."""
    if v * v / a >= d:
        return 2 * math.sqrt(d / a)
    return 2 * v / a + (d - v * v / a) / v


def test_path_time_matches_hand_calc(cfg, trajs):
    c = cfg["path_constraints"]
    v, a = c["max_vel_mps"], c["max_acc_mps2"]
    assert v <= cfg["code"]["speed_at_12v_mps"]  # los constraints caben en el robot teorico
    assert trajs["recta_4m"].total_time == pytest.approx(trapezoid_time(4, v, a), abs=0.02)  # 2.000 s
    assert trajs["corto_1m"].total_time == pytest.approx(trapezoid_time(1, v, a), abs=0.02)  # 1.000 s
    assert trajs["lateral_2m"].total_time == pytest.approx(trapezoid_time(2, v, a), abs=0.02)  # 1.414 s
    assert trajs["recta_6m_giro180"].total_time == pytest.approx(trapezoid_time(6, v, a), abs=0.02)  # 2.458 s
    w, al = math.radians(c["max_ang_vel_deg_s"]), math.radians(c["max_ang_acc_deg_s2"])
    assert trajs["giro_sitio_180"].total_time == pytest.approx(trapezoid_time(math.pi, w, al), abs=0.02)


@pytest.mark.parametrize("mode", ["actual_openloop", "velocity"])
def test_feedforward_only_reaches_end(cfg, trajs, mode):
    """kP = 0, robot ideal (velocidad 100%, COF alto, latencia minima)."""
    tr = trajs["recta_4m"]
    env = nominal_env_params(cfg, 0.0, 0.0, cof=1.6, speed_factor=1.0, latency_s=0.0)
    r = run_trajectory(cfg, variant(cfg, mode), env, tr, TorchBody(env, cfg["sim"]["physics_dt_s"]))
    # Sin feedforward de aceleracion (como el codigo actual) el robot se atrasa ~tau*v y luego frena
    # con el comando en 0; aun asi debe quedar a menos de ~0.5 m (~12% del recorrido).
    assert float(r["final_pos_err_m"][0]) < 0.5
    assert float(r["total_time_s"][0]) < tr.total_time + 1.0
    assert not bool(r["unstable"][0])


def test_pid_reduces_error_and_slow_robot_needs_it(cfg, trajs):
    tr = trajs["recta_4m"]
    var = variant(cfg, "actual_openloop")
    dt = cfg["sim"]["physics_dt_s"]
    errs = {}
    for kp, sf in ((0.0, 1.0), (0.0, 0.85), (5.0, 0.85)):
        env = nominal_env_params(cfg, kp, kp, cof=1.6, speed_factor=sf, latency_s=0.02)
        errs[(kp, sf)] = float(run_trajectory(cfg, var, env, tr, TorchBody(env, dt))["final_pos_err_m"][0])
    assert errs[(0.0, 0.85)] > errs[(0.0, 1.0)]  # robot real mas lento -> se queda corto en open loop
    assert errs[(5.0, 0.85)] < errs[(0.0, 0.85)] / 2  # el PID lo corrige

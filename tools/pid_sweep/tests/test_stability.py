"""Deteccion de inestabilidad (prueba mantener_pose) y efecto de la latencia."""

import math

import pytest

from core.model import TorchBody
from core.params import nominal_env_params, variant
from core.sim import run_trajectory
from core.trajectory import hold_trajectory

KPS = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 40]


def hold(cfg, mode, latency, kp_t, kp_r):
    env = nominal_env_params(cfg, kp_t, kp_r, latency_s=latency, cof=1.2, speed_factor=0.9,
                             init_dx=0.09, init_dy=0.12, init_dyaw=math.radians(5))
    tr = hold_trajectory(cfg["stability"]["hold_test_time_s"])
    return run_trajectory(cfg, variant(cfg, mode), env, tr, TorchBody(env, cfg["sim"]["physics_dt_s"]))


@pytest.mark.parametrize("mode", ["actual_openloop", "velocity"])
def test_low_gain_stable_high_gain_unstable(cfg, mode):
    r = hold(cfg, mode, 0.06, [4.0, 40.0], [4.0, 40.0])
    assert not bool(r["unstable"][0])
    assert bool(r["unstable_t"][1]) and bool(r["unstable_r"][1])


@pytest.mark.parametrize("mode", ["actual_openloop", "velocity"])
def test_more_latency_lowers_limit(cfg, mode):
    def limit(lat):
        r = hold(cfg, mode, lat, KPS, [2.0] * len(KPS))
        bad = [k for k, u in zip(KPS, r["unstable_t"].tolist()) if u]
        return min(bad) if bad else math.inf

    lo, hi = limit(0.02), limit(0.06)
    assert hi < lo
    assert 6 <= hi <= 20  # orden de magnitud esperado: margen de fase con ~70 ms de retraso efectivo


def test_converges_when_stable(cfg):
    r = hold(cfg, "actual_openloop", 0.02, [5.0], [5.0])
    assert float(r["final_pos_err_m"][0]) < 0.05
    assert float(r["final_yaw_err_deg"][0]) < 1.5

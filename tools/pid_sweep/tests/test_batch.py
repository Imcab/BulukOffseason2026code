"""Consistencia numerica del simulador por lotes: lote = individual, dt, simetria, determinismo, CPU = GPU."""

import copy
import math

import pytest
import torch

from core.params import nominal_env_params, sample_env_params, variant
from core.sim import run_batch, run_trajectory
from core.trajectory import Trajectory

KEYS = ("pos_rms_m", "lateral_rms_m", "final_pos_err_m", "final_yaw_err_deg", "max_yaw_err_deg")


def _robots(cfg, n, seed=7, kp=(6.0, 8.0)):
    g = torch.Generator().manual_seed(seed)
    return sample_env_params(cfg, torch.full((n,), kp[0]), torch.full((n,), kp[1]), g)


def test_batch_matches_individual_runs(cfg, trajs):
    names = ["recta_4m", "giro_sitio_180", "diagonal_giro90"]
    tl = [trajs[nm] for nm in names]
    env = _robots(cfg, 6)
    tid = torch.tensor([0, 1, 2, 2, 1, 0])
    rb = run_batch(cfg, variant(cfg), env, tl, tid)
    for i in range(6):
        ri = run_trajectory(cfg, variant(cfg), env.index(slice(i, i + 1)), tl[int(tid[i])])
        for k in KEYS:
            assert float(rb[k][i]) == pytest.approx(float(ri[k][0]), abs=1e-4), (i, k)


def test_physics_dt_convergence(cfg, trajs):
    """Bajar el paso de integracion a la mitad casi no cambia el resultado."""
    env = _robots(cfg, 16)
    out = []
    for dt in (0.002, 0.001):
        c = copy.deepcopy(cfg)
        c["sim"]["physics_dt_s"] = dt
        out.append(run_trajectory(c, variant(c), env, trajs["recta_6m_giro180"]))
    for k in ("pos_rms_m", "final_pos_err_m"):
        a, b = out[0][k], out[1][k]
        assert torch.allclose(a, b, rtol=0.1, atol=0.01), k


def _mirror(tr: Trajectory) -> Trajectory:
    d = {k: v.clone() for k, v in tr.data.items()}
    for k in ("y", "theta", "vy", "omega"):
        d[k] = -d[k]
    return Trajectory(tr.name + "_espejo", "", tr.total_time, d)


def test_mirror_symmetry_without_cm_offset(cfg, trajs):
    c = copy.deepcopy(cfg)
    c["robot"]["cg_offset_m"] = [0.005, 0.0]
    env = nominal_env_params(c, 6.0, 8.0, latency_s=0.04, cg_y=0.0, cg_x=0.005)
    tr = trajs["arco_90_giro90"]
    a = run_trajectory(c, variant(c), env, tr)
    b = run_trajectory(c, variant(c), env, _mirror(tr))
    for k in KEYS:
        assert float(a[k][0]) == pytest.approx(float(b[k][0]), rel=0.02, abs=2e-3), k


def test_cm_offset_breaks_symmetry(cfg, trajs):
    """Con el CM desplazado 4.2 cm a la izquierda, el espejo ya no es identico (control del test anterior)."""
    env = nominal_env_params(cfg, 6.0, 8.0, latency_s=0.04, cof=0.9)
    tr = trajs["arco_90_giro90"]
    a = run_trajectory(cfg, variant(cfg), env, tr)
    b = run_trajectory(cfg, variant(cfg), env, _mirror(tr))
    assert float(a["max_yaw_err_deg"][0]) != pytest.approx(float(b["max_yaw_err_deg"][0]), abs=1e-3)


def test_deterministic(cfg, trajs):
    env = _robots(cfg, 8)
    a = run_trajectory(cfg, variant(cfg), env, trajs["curva_s_lateral"])
    b = run_trajectory(cfg, variant(cfg), env, trajs["curva_s_lateral"])
    for k in KEYS:
        assert torch.equal(a[k], b[k])


@pytest.mark.skipif(not torch.cuda.is_available(), reason="sin CUDA")
def test_cpu_matches_gpu(cfg, trajs):
    env = _robots(cfg, 32)
    tl = [trajs["recta_6m_giro180"], trajs["curva_s_lateral"]]
    tid = torch.arange(32) % 2
    a = run_batch(cfg, variant(cfg, "velocity"), env, tl, tid, "cpu")
    b = run_batch(cfg, variant(cfg, "velocity"), env, tl, tid, "cuda")
    for k in KEYS:
        assert torch.allclose(a[k], b[k], rtol=1e-3, atol=1e-3), k
    assert torch.equal(a["unstable"], b["unstable"])


@pytest.mark.parametrize("mode", ["actual_openloop", "velocity"])
def test_no_motion_without_trajectory_or_error(cfg, mode):
    """Trayectoria quieta y sin error inicial: el robot no se mueve."""
    from core.trajectory import hold_trajectory

    env = nominal_env_params(cfg, 8.0, 8.0)
    r = run_trajectory(cfg, variant(cfg, mode), env, hold_trajectory(1.0))
    assert float(r["max_pos_err_m"][0]) < 1e-4
    assert float(r["max_yaw_err_deg"][0]) < 1e-3


def test_randomization_within_ranges(cfg):
    g = torch.Generator().manual_seed(0)
    n = 5000
    env = sample_env_params(cfg, torch.zeros(n), torch.zeros(n), g)
    rnd = cfg["randomization"]
    for key, col in (("moi_yaw_kgm2", env.moi), ("cof", env.cof), ("speed_factor", env.speed_factor),
                     ("latency_s", env.latency_s), ("cg_height_m", env.cg_height)):
        lo, hi = rnd[key]
        assert float(col.min()) >= lo - 1e-6 and float(col.max()) <= hi + 1e-6, key
        assert float(col.max()) - float(col.min()) > 0.9 * (hi - lo), key  # cubre el rango
    r = torch.hypot(env.init_dx, env.init_dy)
    assert float(r.max()) <= rnd["init_pos_offset_m"] + 1e-6
    assert float(env.init_dyaw.abs().max()) <= math.radians(rnd["init_yaw_offset_deg"]) + 1e-6

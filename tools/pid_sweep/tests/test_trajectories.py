"""Las trayectorias sinteticas respetan los constraints y llegan a donde deben."""

import math

import numpy as np
import pytest

EXPECTED_END = {
    "recta_4m": (4, 0, 0), "recta_6m_giro180": (6, 0, 180), "curva_s_lateral": (4, 1.5, 0),
    "diagonal_giro90": (3, 2, 90), "lateral_2m": (0, 2, 0), "corto_1m": (1, 0, 0),
    "arco_90_giro90": (2, 2, 90), "giro_sitio_180": (0, 0, 180),
}


def test_all_present(trajs):
    assert set(EXPECTED_END) <= set(trajs)


@pytest.mark.parametrize("name", sorted(EXPECTED_END))
def test_constraints_and_endpoints(cfg, trajs, name):
    c = cfg["path_constraints"]
    d = {k: v.numpy().astype(float) for k, v in trajs[name].data.items()}
    v = np.hypot(d["vx"], d["vy"])
    t = d["t"]
    assert np.all(np.diff(t) > 0)
    assert v.max() <= c["max_vel_mps"] + 1e-3
    assert np.abs(d["omega"]).max() <= math.radians(c["max_ang_vel_deg_s"]) * 1.01
    alpha = np.gradient(d["omega"], t)
    assert np.abs(alpha).max() <= math.radians(c["max_ang_acc_deg_s2"]) * 1.10
    # aceleracion total (tangencial + centripeta) <= ~1.45 * max_acc (ver core/trajgen.py)
    acc = np.hypot(np.gradient(d["vx"], t), np.gradient(d["vy"], t))
    assert acc.max() <= c["max_acc_mps2"] * 1.45
    ex, ey, eth = EXPECTED_END[name]
    assert d["x"][-1] == pytest.approx(ex, abs=1e-3)
    assert d["y"][-1] == pytest.approx(ey, abs=1e-3)
    assert math.degrees(d["theta"][-1]) == pytest.approx(eth, abs=0.1)
    assert v[0] == pytest.approx(0, abs=1e-6) and v[-1] == pytest.approx(0, abs=1e-6)


def test_sample_interpolates_and_wraps(trajs):
    tr = trajs["recta_4m"]
    mid = tr.sample(tr.total_time / 2)
    assert mid["x"] == pytest.approx(2.0, abs=0.02)
    assert tr.sample(-1)["x"] == pytest.approx(0)
    assert tr.sample(99)["x"] == pytest.approx(4)


def test_sample_at_exact_end_for_all(trajs):
    for tr in trajs.values():
        for t in (tr.total_time, tr.total_time - 1e-7, tr.total_time + 1e-7):
            assert tr.sample(t)["x"] == pytest.approx(float(tr.data["x"][-1]), abs=1e-3)

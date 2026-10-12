"""Autotuner: funcion de costo, regla de margen, CEM reproducible y resultado valido."""

import copy

import pytest
import torch

from core.autotune import cem, finalize, quick_autotune_config, run_cost, score_candidates
from core.params import variant


def test_cost_function_by_hand(cfg):
    res = {"pos_rms_m": torch.tensor([0.10, 0.10]), "final_pos_err_m": torch.tensor([0.05, 0.05]),
           "yaw_rms_deg": torch.tensor([2.0, 2.0]), "final_yaw_err_deg": torch.tensor([1.0, 1.0]),
           "abort": torch.tensor([False, True])}
    c = run_cost(cfg, res)
    assert c.tolist() == pytest.approx([0.10 + 0.05 + 0.01 * 3.0, 0.18 + 1.0])


@pytest.fixture(scope="module")
def qcfg(cfg):
    return quick_autotune_config(cfg)


def test_margin_rule_rejects_high_gains(qcfg):
    """(4, 4) es valido; (30, 30) es inestable; (12, 12) es estable pero falla el margen (12/0.65 = 18.5)."""
    kps = torch.tensor([[4.0, 4.0], [30.0, 30.0], [12.0, 12.0]])
    s = score_candidates(qcfg, variant(qcfg), kps, 16, seed=11)
    assert bool(s["feasible"][0])
    assert int(s["unstable_nom"][1]) > 0 and not bool(s["feasible"][1])
    assert not bool(s["feasible"][2])
    assert int(s["unstable_margin"][2]) > 0


def test_cem_is_reproducible_and_returns_valid_gains(qcfg):
    var = variant(qcfg)
    s1 = cem(qcfg, var, log=lambda *_: None)
    s2 = cem(qcfg, var, log=lambda *_: None)
    assert s1["mu"].tolist() == s2["mu"].tolist()
    r = finalize(qcfg, var, s1, log=lambda *_: None)
    best = r["best"]
    assert r["finalists"][0]["feasible"]
    # El resultado respeta la regla de margen con robots nuevos (validacion).
    v = r["validation"][0]
    assert v["unstable_nom"] == 0 and v["unstable_margin"] == 0
    assert 0.5 <= float(best[0]) <= 40 and 0.5 <= float(best[1]) <= 40


def test_cem_moves_away_from_bad_start(qcfg):
    """Arrancando en una zona inestable (25, 25), la busqueda baja hasta encontrar ganancias validas."""
    c = copy.deepcopy(qcfg)
    c["autotune"]["init_kp"] = [25.0, 25.0]
    c["autotune"]["generations"] = 7
    s = cem(c, variant(c), log=lambda *_: None)
    assert float(s["mu"][0]) < 25 * 0.7
    assert any(h["feasible"] for h in s["history"] if h["generation"] >= 4)


def test_backoff_lowers_gains_that_violate_margin(qcfg):
    """Si la busqueda entrega (12, 12) (12/0.65 = 18.5 ya es inestable), el retroceso lo baja."""
    from core.sweep import all_trajectories

    search = {"mu": torch.tensor([12.0, 12.0]), "history": [], "trajs": all_trajectories(qcfg)}
    r = finalize(qcfg, variant(qcfg), search, log=lambda *_: None)
    assert r["backoffs"] >= 1
    assert float(r["best"][0]) < 12.0 and float(r["best"][1]) < 12.0
    assert torch.allclose(r["best"], torch.tensor([12.0, 12.0]) * 0.95 ** r["backoffs"])

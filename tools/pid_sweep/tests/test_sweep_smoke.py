"""El sweep completo (torch) corre de punta a punta y escribe results/."""

import copy
import csv

from core.model import TorchBody
from core.sweep import run_sweep


def test_sweep_writes_outputs(cfg, tmp_path):
    c = copy.deepcopy(cfg)
    c["sweep"]["kp_translation"] = {"start": 3.0, "stop": 30.0, "num": 2}
    c["sweep"]["kp_rotation"] = {"start": 3.0, "stop": 30.0, "num": 2}
    c["sweep"]["samples_per_cell"] = 2
    c["sweep"]["trajectories"] = ["corto_1m"]
    c["variants"] = c["variants"][:2]
    dt = c["sim"]["physics_dt_s"]
    run_sweep(c, lambda env: TorchBody(env, dt), out_dir=tmp_path, log=lambda s: None)

    with open(tmp_path / "sweep_results.csv", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2 * 2 * (2 * 2) * 2  # variantes x trayectorias (con mantener_pose) x celdas x muestras
    for k in ("kp_t", "kp_r", "cof", "latency_s", "lateral_rms_m", "final_pos_err_m", "overshoot_m",
              "total_time_s", "unstable", "abort"):
        assert k in rows[0]
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "Recomendacion" in summary
    assert len(list(tmp_path.glob("*.png"))) == 2 + 2  # 1 mapa por variante + 2 curvas

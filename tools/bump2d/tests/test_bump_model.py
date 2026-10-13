"""Sanidad del modelo 2D del bump contra resultados que se calculan a mano."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.bump_model import Bump, Robot, simulate  # noqa: E402

HERE = Path(__file__).resolve().parent.parent


def flat_bottom_robot(clearance: float) -> Robot:
    """Robot de prueba: panza plana a `clearance` sobre el piso, de bumper a bumper."""
    s = np.linspace(-0.45, 0.45, 91)
    return Robot(under_s=s, under_z=np.full_like(s, clearance))


def required_clearance(bump: Bump, half_base=0.2764, r=0.0508) -> float:
    """Calculo a mano: robot montado en la cresta, una rueda en cada pendiente."""
    ang = math.atan2(bump.height, bump.half_width)
    axle = bump.height * (1 - half_base / bump.half_width) + r / math.cos(ang)
    return bump.height - (axle - r)


def test_flat_ground_is_boring():
    m, _ = simulate([flat_bottom_robot(0.04)], np.array([2.0]), Bump(height=1e-6), t_max=3.0)
    assert m["crossed"][0]
    assert m["body_contact_s"][0] == 0
    assert abs(m["max_pitch_deg"][0]) < 0.5 and abs(m["min_pitch_deg"][0]) < 0.5
    assert abs(m["odometry_error_m"][0]) < 0.01


def test_high_centering_threshold_matches_geometry():
    bump = Bump()
    need = required_clearance(bump)
    assert need == pytest.approx(0.079, abs=0.002)
    slow = np.array([0.4, 0.4])
    m, _ = simulate([flat_bottom_robot(need + 0.012), flat_bottom_robot(need - 0.012)], slow, bump, t_max=8.0)
    assert m["crossed"][0] and m["body_contact_s"][0] == 0  # con 1.2 cm de sobra: limpio
    assert m["body_contact_s"][1] > 0  # con 1.2 cm de menos: la panza toca la cresta


def test_odometry_over_the_bump_counts_the_slope():
    bump = Bump()
    surface = 2 * math.hypot(bump.half_width, bump.height) - 2 * bump.half_width  # 4.76 cm
    m, _ = simulate([flat_bottom_robot(0.12)], np.array([0.5]), bump, t_max=8.0)
    assert m["crossed"][0]
    assert m["odometry_error_m"][0] == pytest.approx(surface, abs=0.015)


def test_profile_from_cad_matches_tuner_constants():
    p = json.loads((HERE / "robot_profile.json").read_text(encoding="utf-8"))
    assert p["wheel_half_base_y_m"] == pytest.approx(0.276225, abs=0.003)  # 10.875 in
    assert p["wheel_half_track_x_m"] == pytest.approx(0.276225, abs=0.003)
    belly = [z for s, z, _ in p["directions"]["intake_atras"]["points"] if abs(s) < 0.12]
    assert 0.02 < min(belly) < 0.06  # la panza del CAD esta a ~4 cm
    # En diamante: una rueda adelante, dos al centro y una atras, a 0.276*sqrt(2) = 0.39 m.
    ax = p["directions"]["diamante_intake"]["axles"]
    assert [n for _, n in ax] == [1, 2, 1]
    assert ax[0][0] == pytest.approx(0.2764 * math.sqrt(2), abs=0.005)
    assert ax[1][0] == pytest.approx(0.0, abs=0.005)


def test_diamond_center_wheels_hold_the_belly_over_the_crest():
    """En diamante, al pasar la cresta bajo la panza, las dos ruedas del centro estan sobre ella."""
    bump = Bump()
    a = 0.2764 * math.sqrt(2)
    # Solo la panza entre las ruedas de esquina: las esquinas del bumper son otro problema (ver abajo).
    diamond = Robot(under_s=np.linspace(-0.37, 0.37, 75), under_z=np.full(75, 0.041),
                    axles=[(a, 1), (0.0, 2), (-a, 1)])
    straight = flat_bottom_robot(0.041)
    m, _ = simulate([straight, diamond], np.array([0.4, 0.4]), bump, t_max=10.0)
    assert m["body_contact_s"][0] > 0  # recto, con 4.1 cm, la panza toca
    assert m["crossed"][1]


def test_diamond_corner_overhang_hits_the_ramp_first():
    """En diamante la esquina sobresale 0.6 - 0.39 = 0.21 m de la rueda: la rampa ahi ya mide 6 cm."""
    bump = Bump()
    a = 0.2764 * math.sqrt(2)
    corner = Robot(under_s=np.linspace(-0.6, 0.6, 121), under_z=np.full(121, 0.035),
                   axles=[(a, 1), (0.0, 2), (-a, 1)])
    m, tr = simulate([corner], np.array([0.4]), bump, t_max=4.0, record_every=100)
    k = np.flatnonzero(~np.isnan(tr["bodyx"][:, 0]))
    assert len(k) and tr["bodyx"][k[0], 0] == pytest.approx(0.6, abs=0.02)


def test_deterministic():
    r = [flat_bottom_robot(0.05)] * 2
    a, _ = simulate(r, np.array([1.5, 1.5]), Bump(), t_max=4.0)
    assert a["time_s"][0] == a["time_s"][1] or (np.isnan(a["time_s"][0]) and np.isnan(a["time_s"][1]))

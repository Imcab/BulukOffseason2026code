"""Valida en simulacion el metodo de DriveCharacterizationCommand.java (girar en el lugar).

Se aplica |V| en {1..6} V en ambos sentidos con los modulos tangentes, se mide la rps estable
del rotor y se ajusta |V| = kS + kV * rps. El resultado debe recuperar la friccion (kS) y el kV
"verdaderos" del modelo, aun con CM desplazado, velocidad real distinta y COF bajo.
"""

import copy
import math

import pytest
import torch

from core.model import SwerveModel, TorchBody
from core.params import nominal_env_params, variant

VOLTS = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0]
DT = 0.002


def characterize(cfg, env, settle=1.5, sample=0.5):
    """Replica del comando de Java sobre el modelo. Devuelve (kS, kV, radio estimado)."""
    var = variant(cfg, "actual_openloop")  # SysIdSwerveRotation = VoltageOut al drive
    r_mod = cfg["robot"]["module_half_spacing_m"] * math.sqrt(2)
    v12, g, rw = cfg["code"]["speed_at_12v_mps"], cfg["robot"]["drive_gear_ratio"], cfg["robot"]["wheel_radius_m"]
    xs, ys, radii = [], [], []
    for sign in (1, -1):
        model = SwerveModel(cfg, var, env)
        body = TorchBody(env, DT)
        body.reset(torch.zeros(1, 2), torch.zeros(1))
        for volts in VOLTS:
            w_cmd = sign * volts * v12 / (12 * r_mod)  # como withRotationalRate en el comando
            acc_rps, acc_w, n = 0.0, 0.0, 0
            for k in range(int((settle + sample) / DT)):
                pos, yaw, vel, om = body.state()
                f, tq = model.step(vel, yaw, om, torch.tensor([[0.0, 0.0, w_cmd]]), DT)
                body.step(f, tq)
                if k * DT > settle:
                    # rps del rotor desde la velocidad de piso de cada modulo (sin derrape longitudinal)
                    vbx, vby = vel[0, 0], vel[0, 1]
                    rm = model.rm[0]
                    vmx = vbx - om[0] * rm[:, 1]
                    vmy = vby + om[0] * rm[:, 0]
                    v_long = (vmx * torch.cos(model.steer[0]) + vmy * torch.sin(model.steer[0])).abs().mean()
                    acc_rps += float(v_long) / rw * g / (2 * math.pi)
                    acc_w += abs(float(om[0]))
                    n += 1
            rps, w = acc_rps / n, acc_w / n
            xs.append(rps)
            ys.append(volts)
            radii.append(w * r_mod / (2 * math.pi * rps / g))
    x, y = torch.tensor(xs, dtype=torch.float64), torch.tensor(ys, dtype=torch.float64)
    A = torch.stack([torch.ones_like(x), x], dim=1)
    ks, kv = torch.linalg.lstsq(A, y.unsqueeze(1)).solution.flatten().tolist()
    return ks, kv, sum(radii) / len(radii)


def truth(cfg, sf):
    """kS y kV verdaderos del modelo: V = fs + R*I_libre + rps * 2*pi * (12 - fs) / (12 * Kv * sf).

    El kS real incluye la friccion interna del motor (R * corriente libre ~ 0.05 V), igual que en
    el robot: el comando mide la suma.
    """
    m = cfg["drive_motor"]
    fs = m["static_friction_v"]
    res = m["nominal_voltage"] / m["stall_current_a"]
    kv_rad = m["free_speed_rpm"] * 2 * math.pi / 60 / (m["nominal_voltage"] - res * m["free_current_a"])
    return fs + res * m["free_current_a"], 2 * math.pi * (12 - fs) / (12 * kv_rad * sf)


@pytest.mark.parametrize("sf,cof,fs", [(1.0, 1.2, 0.408), (0.88, 0.9, 0.25), (0.95, 1.5, 0.6)])
def test_spin_in_place_recovers_ks_kv(cfg, sf, cof, fs):
    c = copy.deepcopy(cfg)
    c["drive_motor"]["static_friction_v"] = fs
    env = nominal_env_params(c, 0, 0, cof=cof, speed_factor=sf)
    ks, kv, radius = characterize(c, env)
    ks_true, kv_true = truth(c, sf)
    assert ks == pytest.approx(ks_true, abs=0.01)
    assert kv == pytest.approx(kv_true, rel=0.01)
    assert radius == pytest.approx(c["robot"]["wheel_radius_m"], rel=0.02)


def test_theoretical_kv_matches_tuner_constant(cfg):
    _, kv_true = truth(cfg, 1.0)
    assert kv_true == pytest.approx(cfg["code"]["drive_kV"], rel=0.04)  # 0.124 V/rps

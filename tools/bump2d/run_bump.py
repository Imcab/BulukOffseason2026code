"""Barre el cruce del bump: velocidades x direcciones x altura extra del chasis.

Uso (desde tools/bump2d):
  python run_bump.py                       # barrido por default, imprime la tabla
  python run_bump.py --viewer out.html     # ademas genera el visor animado

Direcciones (la silueta sale de robot_profile.json, extraida del CAD):
  intake_adelante : cruza con el lado del intake (-Y del CAD) por delante
  intake_atras    : cruza con el otro lado por delante
  de_lado         : cruza de lado (a lo largo de X del CAD)
  diamante_intake : en diagonal (45 grados), con la esquina del lado del intake por delante
  diamante_atras  : en diagonal, con la otra esquina por delante
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from core.bump_model import Bump, Robot, simulate

HERE = Path(__file__).resolve().parent


def robot_for(profile: dict, direction: str, lift: float = 0.0, **kw) -> Robot:
    """lift: cuanto se sube TODO el chasis (no las ruedas), p. ej. con modulos mas bajos o placas."""
    d = profile["directions"][direction]
    s = np.array([p[0] for p in d["points"]])
    z = np.array([p[1] for p in d["points"]]) + lift
    axles = [(sa, n) for sa, n in d["axles"]]
    return Robot(under_s=s, under_z=z, axles=axles, wheel_radius=profile["wheel_radius_m"], **kw)


def run(speeds, directions, lifts, profile, bump=Bump(), t_max=8.0, record_every=100):
    cases = [(d, v, l) for d in directions for l in lifts for v in speeds]
    robots = [robot_for(profile, d, l) for d, v, l in cases]
    metrics, trace = simulate(robots, np.array([v for _, v, _ in cases]), bump, t_max=t_max, record_every=record_every)
    return cases, metrics, trace


def table(cases, metrics):
    print(f"{'direccion':16s} {'chasis':>7s} {'vel':>5s} | {'resultado':10s} {'tiempo':>6s} {'roce':>6s} "
          f"{'F roce':>7s} {'pitch':>11s} {'en aire':>7s} {'odom':>6s} {'vel min':>7s}")
    for i, (d, v, l) in enumerate(cases):
        res = "CRUZA" if metrics["crossed"][i] else ("COLGADO" if metrics["stuck"][i] else "NO CRUZA")
        if res == "CRUZA" and metrics["body_contact_s"][i] > 0:
            res = "cruza+roce"
        print(f"{d:16s} {('+' + str(int(round(l * 100))) + ' cm') if l else 'actual':>7s} {v:5.1f} | {res:10s} "
              f"{metrics['time_s'][i]:6.2f} {metrics['body_contact_s'][i]:6.2f} {metrics['body_max_force_n'][i]:7.0f} "
              f"{metrics['min_pitch_deg'][i]:5.1f}/{metrics['max_pitch_deg'][i]:4.1f} {metrics['airborne_s'][i]:7.2f} "
              f"{100 * metrics['odometry_error_m'][i]:5.1f}c {metrics['min_speed_mps'][i]:7.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", type=Path, default=HERE / "robot_profile.json")
    ap.add_argument("--speeds", default="0.5,1,1.5,2,3,4")
    ap.add_argument("--lifts-cm", default="0,2,4")
    ap.add_argument("--directions", default="intake_adelante,intake_atras,de_lado,diamante_intake,diamante_atras")
    ap.add_argument("--viewer", type=Path, default=None)
    args = ap.parse_args()
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    speeds = [float(s) for s in args.speeds.split(",")]
    lifts = [float(s) / 100 for s in args.lifts_cm.split(",")]
    directions = args.directions.split(",")
    cases, metrics, trace = run(speeds, directions, lifts, profile)
    table(cases, metrics)
    if args.viewer:
        from core.viewer import write_viewer

        write_viewer(args.viewer, profile, cases, metrics, trace, Bump())
        print(f"visor: {args.viewer}")


if __name__ == "__main__":
    main()

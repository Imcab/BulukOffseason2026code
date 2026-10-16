"""Genera el visor HTML animado del cruce (vista lateral) a partir de los resultados de simulate()."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

TEMPLATE = Path(__file__).with_name("viewer_template.html")


def _r(a, nd=3):
    return [None if (isinstance(v, float) and math.isnan(v)) else round(float(v), nd) for v in a]


def write_viewer(out: Path, profile: dict, cases, metrics, trace, bump, robot_defaults=None, fragment=False,
                 notes: dict | None = None):
    from .bump_model import Drive, Robot

    rd = robot_defaults or Robot()
    dt = float(trace["t"][1] - trace["t"][0]) if len(trace["t"]) > 1 else 0.01
    data_cases = []
    for i, (d, v, lift) in enumerate(cases):
        tr = {
            "t": _r(trace["t"]),
            "x": _r(trace["x"][:, i], 4),
            "z": _r(trace["z"][:, i], 4),
            "th": _r(trace["th"][:, i], 4),
            "n": [[round(float(n), 0) for n in row] for row in trace["normals"][:, i, :]],
            "b": _r(trace["body"][:, i], 0),
            "bx": _r(trace["bodyx"][:, i], 3),
        }
        # recortar cuando ya termino (el robot sigue en la simulacion pero ya no importa)
        end = metrics["time_s"][i]
        if not np.isnan(end):
            k = min(len(tr["t"]), int((end + 0.4) / dt) + 1)
            tr = {key: val[:k] for key, val in tr.items()}
        data_cases.append({
            "dir": d, "v": float(v), "lift": round(float(lift), 3),
            "crossed": bool(metrics["crossed"][i]), "stuck": bool(metrics["stuck"][i]),
            "time": None if np.isnan(metrics["time_s"][i]) else round(float(metrics["time_s"][i]), 2),
            "contact": round(float(metrics["body_contact_s"][i]), 2),
            "force": round(float(metrics["body_max_force_n"][i]), 0),
            "pmin": round(float(metrics["min_pitch_deg"][i]), 1),
            "pmax": round(float(metrics["max_pitch_deg"][i]), 1),
            "air": round(float(metrics["airborne_s"][i]), 2),
            "odo": round(float(metrics["odometry_error_m"][i]), 3),
            "note": (notes or {}).get(d, ""),
            "trace": tr,
        })
    data = {
        "source": profile.get("source", "CAD"),
        "profiles": profile["directions"],
        "bump": {"x0": bump.x0, "w": bump.half_width, "h": bump.height},
        "R": profile["wheel_radius_m"],
        "cg": rd.cg_height, "cgS": rd.cg_s, "mass": rd.mass, "stator": Drive().stator_limit,
        "dt": dt,
        "cases": data_cases,
    }
    html = TEMPLATE.read_text(encoding="utf-8").replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    if not fragment:
        html = ('<!doctype html>\n<html lang="es"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width, initial-scale=1"></head><body>\n' + html + "\n</body></html>\n")
    Path(out).write_text(html, encoding="utf-8")

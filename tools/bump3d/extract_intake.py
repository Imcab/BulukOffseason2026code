#!/usr/bin/env python3
"""Saca del STL del robot la parte del intake extendido que sobresale del bumper, como cajas.

En el modelo de MARS los mecanismos no chocan con nada; aqui el intake extendido se vuelve
colision: franjas de 2 cm a lo largo de lo que sobresale, cada una una caja de su altura
minima a su altura maxima. Salida en el marco del modelo de MARS (+X = lado del intake,
z desde el piso, origen en el centro de los modulos).

  python -I extract_intake.py ruta/al/robot.stl --out intake_profile.json

El STL se lee con un parser binario propio (numpy) y se trata como datos.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

BUMPER_HALF = 0.4508      # medio lado del bumper (robotbase_bumper.stl de MARS: +-0.451)
CAD_MODULE_CENTER = (-0.0125, 0.1207)  # de tools/bump2d/robot_profile.json
SLICE = 0.02


def read_stl(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        head = f.read(84)
        n = int(np.frombuffer(head[80:84], "<u4")[0])
        if 84 + 50 * n != path.stat().st_size:
            raise SystemExit(f"{path} no es un STL binario valido")
        tri = np.fromfile(f, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    return tri["v"].reshape(-1, 3).astype(np.float64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("intake_profile.json"))
    a = ap.parse_args()
    v = read_stl(a.stl)
    if np.ptp(v[:, 0]) > 50:          # en milimetros
        v = v / 1000.0
    z = v[:, 2] - v[:, 2].min()
    # CAD -> modelo de MARS: el intake esta en -Y del CAD y es +X del modelo.
    mx = -(v[:, 1] - CAD_MODULE_CENTER[1])
    my = v[:, 0] - CAD_MODULE_CENTER[0]
    out = mx > BUMPER_HALF + 0.004
    if not out.any():
        raise SystemExit("no hay nada fuera del bumper del lado del intake")
    boxes = []
    x = BUMPER_HALF
    tip = mx[out].max()
    while x < tip:
        m = out & (mx >= x) & (mx < x + SLICE)
        if m.any():
            boxes.append({"x0": round(x, 4), "x1": round(min(x + SLICE, tip), 4),
                          "y0": round(float(my[m].min()), 4), "y1": round(float(my[m].max()), 4),
                          "z0": round(float(z[m].min()), 4), "z1": round(float(z[m].max()), 4)})
        x += SLICE
    prof = {"source": a.stl.name, "frame": "modelo MARS: +X intake, z sobre el piso, origen centro de modulos",
            "tip_beyond_bumper_m": round(float(tip - BUMPER_HALF), 4),
            "lowest_m": round(min(b["z0"] for b in boxes), 4), "boxes": boxes}
    a.out.write_text(json.dumps(prof, indent=1), encoding="utf-8")
    print(f"{len(boxes)} cajas; sobresale {prof['tip_beyond_bumper_m'] * 100:.1f} cm; "
          f"lo mas bajo a {prof['lowest_m'] * 100:.1f} cm -> {a.out}")


if __name__ == "__main__":
    main()

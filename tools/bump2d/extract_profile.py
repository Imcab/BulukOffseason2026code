"""Saca del CAD (STL) la silueta lateral del robot para el modelo 2D del bump.

Uso (desde tools/bump2d):
  python extract_profile.py C:/ruta/al/Robot.stl            -> robot_profile.json

Supone el STL en metros con Z hacia arriba (como el que exporta el visualizador 3D de MARS). Detecta
el piso (punto mas bajo = fondo de las llantas), los centros de las 4 ruedas y, para cada direccion
de cruce (a lo largo de Y y de X del CAD), la altura minima (parte de abajo) y maxima (parte de
arriba) de todo lo que NO es llanta, en tiras de 1 cm, y donde quedan las ruedas a lo largo del
avance (en diamante: una esquina adelante, dos al centro y una atras). Los modulos, la panza, los bumpers y el
intake cuentan: cualquiera puede tocar el bump. Solo se excluye el volumen de las llantas.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

WHEEL_RADIUS = 0.0508
STEP = 0.01


def load_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        raise ValueError("STL en texto (ASCII): exportalo en binario")
    n = int(np.frombuffer(data[80:84], dtype="<u4")[0])
    tri = np.frombuffer(data[84 : 84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return tri["v"].reshape(-1, 3).astype(np.float64)


def find_wheels(v: np.ndarray) -> np.ndarray:
    low = v[v[:, 2] < 0.004]
    cx, cy = np.median(low[:, 0]), np.median(low[:, 1])
    wheels = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            q = low[(np.sign(low[:, 0] - cx) == sx) & (np.sign(low[:, 1] - cy) == sy)]
            if len(q) < 10:
                raise ValueError("no se encontraron 4 ruedas tocando el piso")
            wheels.append((q[:, 0].mean(), q[:, 1].mean()))
    return np.array(wheels)


def silhouette(coord: np.ndarray, z: np.ndarray) -> list[list[float]]:
    """[[posicion, z_abajo, z_arriba], ...] en tiras de STEP."""
    idx = np.floor(coord / STEP).astype(int)
    order = np.argsort(idx)
    idx, z = idx[order], z[order]
    starts = np.flatnonzero(np.r_[True, idx[1:] != idx[:-1]])
    ends = np.r_[starts[1:], len(idx)]
    return [[round((idx[a] + 0.5) * STEP, 3), round(float(z[a:b].min()), 4), round(float(z[a:b].max()), 4)]
            for a, b in zip(starts, ends)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("robot_profile.json"))
    args = ap.parse_args()

    v = load_stl(args.stl)
    v[:, 2] -= v[:, 2].min()  # z = altura sobre el piso
    wheels = find_wheels(v)
    cx, cy = wheels[:, 0].mean(), wheels[:, 1].mean()

    keep = np.ones(len(v), bool)
    for wx, wy in wheels:  # cilindro de la llanta (eje X): ancho real con su borde y la banda
        keep &= ~((np.abs(v[:, 0] - wx) < 0.06) & (np.hypot(v[:, 1] - wy, v[:, 2] - WHEEL_RADIUS) < WHEEL_RADIUS + 0.012))
    body = v[keep]
    bx, by, bz = body[:, 0] - cx, body[:, 1] - cy, body[:, 2]

    # Direcciones de cruce (vector unitario de avance en el plano XY del CAD). -Y = lado del intake.
    r2 = 1 / np.sqrt(2)
    directions = {
        "intake_adelante": (0.0, -1.0),
        "intake_atras": (0.0, 1.0),
        "de_lado": (1.0, 0.0),
        "diamante_intake": (r2, -r2),  # en diagonal, con la esquina del lado del intake por delante
        "diamante_atras": (r2, r2),
    }
    wrel = wheels - np.array([cx, cy])
    out = {
        "source": args.stl.name,
        "wheel_radius_m": WHEEL_RADIUS,
        "wheel_half_base_y_m": round(float(np.ptp(wheels[:, 1]) / 2), 4),
        "wheel_half_track_x_m": round(float(np.ptp(wheels[:, 0]) / 2), 4),
        "cad_module_center": [round(float(cx), 4), round(float(cy), 4)],
        "note": "Coordenadas relativas al centro de modulos; z sobre el piso. Para cada direccion: points = "
                "[s, z_abajo, z_arriba] a lo largo del avance (s > 0 adelante) y axles = [s, ruedas] (ruedas "
                "con la misma s se apoyan juntas).",
        "directions": {},
    }
    for name, (dx, dy) in directions.items():
        s_body = bx * dx + by * dy
        s_wheels = wrel[:, 0] * dx + wrel[:, 1] * dy
        axles = []
        for sw in sorted(s_wheels, reverse=True):
            if axles and abs(axles[-1][0] - sw) < 0.03:
                axles[-1][1] += 1
            else:
                axles.append([round(float(sw), 4), 1])
        out["directions"][name] = {"vector": [dx, dy], "points": silhouette(s_body, bz), "axles": axles}
    args.out.write_text(json.dumps(out), encoding="utf-8")
    mid = [z for p, z, _ in out["directions"]["intake_atras"]["points"] if abs(p) < 0.2]
    print(f"{len(v) // 3} triangulos | ruedas en y +-{out['wheel_half_base_y_m']} x +-{out['wheel_half_track_x_m']} | "
          f"panza entre ejes: {100 * min(mid):.1f} cm | -> {args.out}")


if __name__ == "__main__":
    main()

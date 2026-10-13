#!/usr/bin/env python3
"""Banco del cruce en la cancha REAL: hub + bump + trench, con el intake extendido.

Cada escena es una copia del tramo de la cancha 2026 donde el robot cruza el bump: el
hub de un lado, el riser del trench del otro (esta ENCIMA de los ultimos 30 cm del bump)
y el brazo del trench arriba. Medidas del modelo oficial de la cancha (el GLB de MARS):

  hub     x -4.26..-2.73, y -0.75..+0.75, 3.08 m de alto
  bump    centro (-3.645, +1.6345), 1.12 m a lo largo x 2.23 m de ancho, 16.5 cm
  trench  riser x -4.23..-3.06, y 2.45..2.75, 0.50 m de alto; brazo a 0.565 m del piso
  => pasillo libre sobre el bump: y 0.75..2.45 = 1.71 m

El robot sale de la zona de alianza (x = -5.65) y cruza hacia +x. Variables por escena:
direccion del chasis, posicion lateral en el pasillo (0 = centro; - hacia el hub),
angulo de llegada (0 = perpendicular al bump) y velocidad.

  python gen_field_bench.py
  python gen_field_bench.py --offsets-cm -30,-15,0,15,30 --angles-deg -10,0,10 --speeds 1.5,2.5
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import gen_bump_bench as gb

HERE = Path(__file__).resolve().parent

# --- cancha 2026 (marco de Gazebo, origen al centro; lado azul, bump de y positiva) ----
HUB = ((-3.4933, 0.0051, 1.5407), (1.5306, 1.4947, 3.0814))
BUMP_C = (-3.645, 1.6345)
BUMP_W = 2.2333
RISER = ((-3.645, 2.6035, 0.25), (1.168, 0.301, 0.50))           # skirts: caja cerrada
POST = ((-3.645, 2.6025, 0.2785), (0.152, 0.295, 0.557))
ARM = ((-3.645, 3.355, 0.616), (0.152, 1.799, 0.102))
WALL_Y = 4.105                                                    # 8.21 / 2
CORRIDOR = (HUB[0][1] + HUB[1][1] / 2, RISER[0][1] - RISER[1][1] / 2)  # (0.75, 2.45)
LANE_Y = (CORRIDOR[0] + CORRIDOR[1]) / 2

START_BEFORE = 2.0       # el robot arranca a 2 m (a lo largo de su avance) del centro del bump
SLICE_DX, SLICE_DY = 7.0, 6.5


def box(name, center, size, color, rpy=(0, 0, 0), mu=0.6):
    (x, y, z), (sx, sy, sz) = center, size
    r, g, b = color
    pose = f"{x:.4f} {y:.4f} {z:.4f} {rpy[0]} {rpy[1]:.5f} {rpy[2]}"
    geo = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    return (f'      <collision name="{name}"><pose>{pose}</pose>{geo}<surface><friction><ode><mu>{mu}</mu>'
            f"<mu2>{mu}</mu2></ode></friction></surface></collision>\n"
            f'      <visual name="{name}"><pose>{pose}</pose>{geo}<material><ambient>{r * .5} {g * .5} {b * .5} 1</ambient>'
            f"<diffuse>{r} {g} {b} 1</diffuse></material></visual>")


def field_slice(tag: str, ox: float, oy: float) -> str:
    """Hub, bump y trench de una escena, corridos (ox, oy)."""
    def at(c):
        return (c[0] + ox, c[1] + oy, c[2])

    parts = [f'    <model name="cancha_{tag}"><static>true</static><link name="link">']
    parts.append(box("hub", at(HUB[0]), HUB[1], (0.25, 0.35, 0.75)))
    # bump: dos losas inclinadas, como cancha_2026._rampas()
    ang = math.atan2(gb.BUMP_HEIGHT, gb.BUMP_HALF)
    largo = math.hypot(gb.BUMP_HALF, gb.BUMP_HEIGHT)
    for i, s in enumerate((-1, 1), start=1):
        mx, mz = BUMP_C[0] + s * gb.BUMP_HALF / 2, gb.BUMP_HEIGHT / 2
        p = s * ang
        c = (mx - math.sin(p) * gb.SLAB / 2 + ox, BUMP_C[1] + oy, mz - math.cos(p) * gb.SLAB / 2)
        parts.append(box(f"bump_{i}", c, (largo, BUMP_W, gb.SLAB), (0.55, 0.5, 0.42), rpy=(0, p, 0), mu=0.9))
    parts.append(box("trench_riser", at(RISER[0]), RISER[1], (0.85, 0.85, 0.85)))
    parts.append(box("trench_poste", at(POST[0]), POST[1], (0.6, 0.6, 0.6)))
    parts.append(box("trench_brazo", at(ARM[0]), ARM[1], (0.95, 0.75, 0.1)))
    parts.append(box("pared", (BUMP_C[0] + ox, WALL_Y + 0.05 + oy, 0.25), (4.0, 0.1, 0.5), (0.3, 0.3, 0.3)))
    parts.append("    </link></model>")
    return "\n".join(parts)


def build(mars_sim: Path, directions, offsets, angles, speeds, intake: dict | None):
    base, mec = gb.robot_model(mars_sim)
    scenes, body = [], []
    rows = list(itertools.product(offsets, angles, speeds))
    for col, d in enumerate(directions):
        for row, (off, angle, v) in enumerate(rows):
            ox, oy = col * SLICE_DX, row * SLICE_DY
            th = math.radians(angle)
            # pasa por (bump, carril + off); arranca START_BEFORE antes a lo largo de su avance
            px, py = BUMP_C[0] + ox, LANE_Y + off + oy
            sx, sy = px - START_BEFORE * math.cos(th), py - START_BEFORE * math.sin(th)
            tag = f"{d}_o{round(off * 100):+d}_a{round(angle):+d}_{str(v).replace('.', 'p')}ms".replace("+", "p").replace("-", "m")
            ux, uy = gb.DIRECTIONS[d]
            yaw = th - math.atan2(uy, ux)
            # hacia donde mira el intake (+X del modelo) en el mundo: -y = hub, +y = trench
            iy = math.sin(yaw)
            side = "hub" if iy < -0.3 else ("trench" if iy > 0.3 else ("adelante" if math.cos(yaw) > 0 else "atras"))
            scenes.append({"name": tag, "direction": d, "speed": v, "lift": 0.0, "offset": off, "angle": angle,
                           "intake_side": side,
                           "x": sx, "y": sy, "yaw": yaw,
                           # cruzo cuando su centro pasa 0.8 m mas alla del final del bump (en x)
                           "end_x": (BUMP_C[0] + gb.BUMP_HALF + 0.8) + ox - sx})
            body.append(f"    <!-- {tag} -->")
            body.append(field_slice(tag, ox, oy))
            body.append("    " + gb.scene_robot(base, mec, tag, sx, sy, d, v, 0.0, travel_yaw=th, intake=intake))
    head = [
        '<?xml version="1.0" ?>', '<sdf version="1.11">', '  <world name="mars">',
        '    <physics name="250hz" type="ignored"><max_step_size>0.004</max_step_size>'
        "<real_time_factor>1.0</real_time_factor></physics>",
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>',
        "    <gravity>0 0 -9.8</gravity>",
        '    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.7 0.75 0.8 1</background></scene>',
        '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>'
        "<diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>",
        '    <model name="ground_plane"><static>true</static><link name="link">'
        '<collision name="collision"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>'
        "<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>"
        '<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>'
        "<material><ambient>0.3 0.3 0.33 1</ambient><diffuse>0.35 0.35 0.38 1</diffuse></material></visual>"
        "</link></model>",
    ]
    return "\n".join(head + body + ["  </world>", "</sdf>", ""]), scenes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mars-sim", type=Path, default=gb.default_mars_sim())
    ap.add_argument("--directions", default=",".join(gb.DIRECTIONS))
    ap.add_argument("--offsets-cm", default="-20,0,20", help="posicion lateral en el pasillo; - = hacia el hub")
    ap.add_argument("--angles-deg", default="0", help="angulo de llegada respecto a perpendicular al bump")
    ap.add_argument("--speeds", default="2")
    ap.add_argument("--sin-intake", action="store_true", help="intake sin colision (como el mundo de MARS)")
    ap.add_argument("--intake-profile", type=Path, default=HERE / "intake_profile.json")
    ap.add_argument("--out", type=Path, default=HERE / "build" / "field_bench.sdf")
    a = ap.parse_args()
    dirs = a.directions.split(",")
    for d in dirs:
        if d not in gb.DIRECTIONS:
            raise SystemExit(f"direccion desconocida: {d}")
    offsets = [float(s) / 100 for s in a.offsets_cm.split(",")]
    angles = [float(s) for s in a.angles_deg.split(",")]
    speeds = [float(s) for s in a.speeds.split(",")]
    intake = None if a.sin_intake else json.loads(a.intake_profile.read_text(encoding="utf-8"))
    sdf, scenes = build(a.mars_sim, dirs, offsets, angles, speeds, intake)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(sdf, encoding="utf-8", newline="\n")
    meta = {"kind": "field", "corridor_y": CORRIDOR, "wheel_radius": gb.WHEEL_RADIUS, "intake": intake is not None,
            "bump": {"x0": 0, "half": 0, "height": gb.BUMP_HEIGHT}, "scenes": scenes}
    a.out.with_suffix(".json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    gb.gui_config(a.mars_sim, [{"x": s["x"] + 2.5, "y": s["y"]} for s in scenes], a.out.with_name("gui.config"))
    print(f"{len(scenes)} escenas -> {a.out}  (pasillo libre {CORRIDOR[1] - CORRIDOR[0]:.2f} m; "
          f"intake {'con' if intake else 'sin'} colision)")


if __name__ == "__main__":
    main()

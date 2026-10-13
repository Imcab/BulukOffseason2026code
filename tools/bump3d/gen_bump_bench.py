#!/usr/bin/env python3
"""Genera un mundo de Gazebo con VARIAS escenas del cruce del bump corriendo en paralelo.

Cada escena es una copia del robot de MARS (models/offseason2026: mallas del CAD,
colision del bumper a su altura real) en su propio carril, con su propio bump
enfrente, cruzando en una direccion y a una velocidad. Todas arrancan a la vez.

No usa el codigo del robot ni el bridge: MarsLink escucha topicos fijos, asi que en
un mundo solo un robot puede recibir ordenes del codigo. Aqui cada robot lleva
plugins ESTANDAR de gz-sim:
  - la direccion queda fija en el angulo de cruce (los modulos apuntan hacia
    donde va el robot, como cuando el swerve cruza en linea recta);
  - cada rueda va con un JointController de velocidad cuyo par esta topado en
    el limite de stator de TunerConstants (70 A), que es lo que de verdad
    limita la traccion al subir.

Uso (con el python del entorno mars-sim o cualquier python 3):
  python gen_bump_bench.py                         # 5 direcciones x 4 velocidades
  python gen_bump_bench.py --speeds 1,2 --lifts-cm 0,4 --out bench.sdf
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# --- de TunerConstants.java ----------------------------------------------------
STATOR_LIMIT_A = 70.0                      # kSlipCurrent = StatorCurrentLimit
DRIVE_GEAR = 5.2734375                     # kDriveGearRatio
KT_KRAKEN = 9.37 / 483                     # par / corriente de calado, Kraken X60 FOC
WHEEL_TORQUE_MAX = STATOR_LIMIT_A * KT_KRAKEN * DRIVE_GEAR  # ~7.2 N*m en la rueda
WHEEL_RADIUS = 0.0508

# --- el bump (mismo que worlds/cancha_2026.py de MARS) --------------------------
BUMP_HEIGHT = 0.165
BUMP_HALF = 0.56
BUMP_WIDTH = 2.2333                        # a lo ancho del carril
SLAB = 0.08                                # grosor de la losa inclinada
# Friccion del bumper (tela) contra la rampa y el piso. Gazebo pone 1.0 si no se
# dice nada, y eso deja al robot clavado en la esquina del bumper. 0.3 es la
# misma que usa el modelo 2D (tools/bump2d, Contact.mu_body). TODO medir.
MU_BUMPER = 0.3

LANE = 3.0                                 # separacion entre carriles (filas: velocidad y altura)
COLUMN = 9.0                               # separacion entre columnas (una por direccion)
BUMP_X0 = 1.3                              # donde empieza la rampa (el robot sale de x = 0)

R2 = 1 / math.sqrt(2)
# Vector de avance en el marco del modelo de MARS: +X = lado del intake.
DIRECTIONS = {
    "intake_adelante": (1.0, 0.0),
    "intake_atras": (-1.0, 0.0),
    "de_lado": (0.0, 1.0),
    "diamante_intake": (R2, R2),
    "diamante_atras": (-R2, R2),
    # espejos: en la cancha importa de que lado queda el intake (hub o trench)
    "de_lado_espejo": (0.0, -1.0),
    "diamante_intake_espejo": (R2, -R2),
    "diamante_atras_espejo": (-R2, -R2),
}
BENCH_DIRECTIONS = ["intake_adelante", "intake_atras", "de_lado", "diamante_intake", "diamante_atras"]


def default_mars_sim() -> Path:
    return Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "MARS" / "sim"


def robot_model(mars_sim: Path) -> tuple[str, dict]:
    """El <model name="robot"> que genera MARS para este robot, sin tocar sus archivos."""
    sys.path.insert(0, str(mars_sim / "worlds"))
    import gen_offseason2026 as o  # noqa: E402  (de la instalacion de MARS)

    g = o.g
    g.FUEL = 0
    g.HITBOXES = False
    g.CAMPO_VISUAL = "ninguno"
    mec = o.cargar_mecanismo()
    if mec is None:
        raise SystemExit(f"falta {o.MECANISMO}: sin el no hay mallas ni bumper real")
    o.ajustar_masas(mec)
    world = o.con_mecanismo(g.generar_sdf(), mec)
    m = re.search(r'<model name="robot">.*?\n    </model>', world, re.S)
    if not m:
        raise SystemExit("gen_offseason2026 cambio la forma del modelo")
    return m.group(0), mec


def scene_robot(base: str, mec: dict, name: str, x: float, y: float, direction: str, speed: float,
                lift: float, travel_yaw: float = 0.0, intake: dict | None = None) -> str:
    """travel_yaw: hacia donde avanza en el mundo (0 = +X). intake: intake_profile.json para
    que el intake extendido choque (sin el, como en MARS, el intake no tiene colision)."""
    ux, uy = DIRECTIONS[direction]
    alpha = math.atan2(uy, ux)           # angulo de los modulos en el marco del robot
    yaw = travel_yaw - alpha             # girar el robot para que ese vector apunte a travel_yaw
    t = base.replace('<model name="robot">', f'<model name="{name}">', 1)
    t = re.sub(r"(<model name=\"[^\"]+\">\s*<pose>)[^<]*(</pose>)",
               lambda m: f"{m.group(1)}{x:.4f} {y:.4f} 0 0 0 {yaw:.6f}{m.group(2)}", t, count=1)

    # Modulos: direccion fija en alpha. El link de la rueda gira su eje (Z local,
    # que con roll -90 queda como +Y) con la misma alpha.
    def wheel(m):
        return f'{m.group(1)}{m.group(2)} {m.group(3)} {m.group(4)} -1.5707963 0 {alpha:.6f}</pose>'

    t = re.sub(r'(<link name="(?:fl|fr|bl|br)_wheel">\s*<pose>)(\S+) (\S+) (\S+) -1\.5707963 0 0</pose>', wheel, t)
    t = re.sub(r'(<link name="(?:fl|fr|bl|br)_steer">\s*<pose>)(\S+) (\S+) (\S+) 0 0 0</pose>',
               lambda m: f'{m.group(1)}{m.group(2)} {m.group(3)} {m.group(4)} 0 0 {alpha:.6f}</pose>', t)

    # Direccion y mecanismos como joints fijos: nadie los manda en esta escena.
    fixed = [f"{c}_steer_joint" for c in ("fl", "fr", "bl", "br")] + [f"{p['nombre']}_joint" for p in mec["piezas"]]
    for j in fixed:
        t, n = re.subn(rf'<joint name="{j}" type="\w+">(.*?)<axis>.*?</axis>\s*</joint>',
                       lambda m: f'<joint name="{j}" type="fixed">{m.group(1)}</joint>', t, count=1, flags=re.S)
        assert n == 1, f"no encontre el joint {j}"

    # Chasis mas alto (lift): suben el chasis y los mecanismos, no las ruedas.
    if lift:
        movable = ["chassis"] + [p["nombre"] for p in mec["piezas"]]
        for link in movable:
            t = re.sub(rf'(<link name="{link}">\s*<pose>)(\S+) (\S+) (\S+)',
                       lambda m: f"{m.group(1)}{m.group(2)} {m.group(3)} {float(m.group(4)) + lift:.4f}", t, count=1)

    # Sensor de contacto en el chasis: dice cuando el bumper toca el bump.
    for col in ("bumper", "torre"):
        t, n = re.subn(rf'(<collision name="{col}">.*?</geometry>)(</collision>)',
                       lambda m: f"{m.group(1)}<surface><friction><ode><mu>{MU_BUMPER}</mu><mu2>{MU_BUMPER}</mu2>"
                                 f"</ode></friction></surface>{m.group(2)}", t, count=1, flags=re.S)
        assert n == 1, f"no encontre la colision {col}"

    # Intake extendido: cajas de colision en el chasis (su joint ya es fijo).
    watched = ["bumper"]
    if intake:
        cols = []
        for i, b in enumerate(intake["boxes"]):
            cx, cy, cz = (b["x0"] + b["x1"]) / 2, (b["y0"] + b["y1"]) / 2, (b["z0"] + b["z1"]) / 2
            size = f'{b["x1"] - b["x0"]:.4f} {b["y1"] - b["y0"]:.4f} {b["z1"] - b["z0"]:.4f}'
            cols.append(f'<collision name="intake_{i}"><pose>{cx:.4f} {cy:.4f} {cz - 0.2:.4f} 0 0 0</pose>'
                        f"<geometry><box><size>{size}</size></box></geometry><surface><friction><ode>"
                        f"<mu>{MU_BUMPER}</mu><mu2>{MU_BUMPER}</mu2></ode></friction></surface></collision>")
            watched.append(f"intake_{i}")
        t = t.replace('<collision name="torre">', "\n        ".join(cols) + '\n        <collision name="torre">', 1)
    watched.append("torre")

    # Publica en /world/mars/model/<escena>/link/chassis/sensor/contacto/contact.
    sensor = ('\n        <sensor name="contacto" type="contact"><always_on>1</always_on><update_rate>250</update_rate>'
              '<contact>' + "".join(f"<collision>{c}</collision>" for c in watched) + '</contact></sensor>')
    t = t.replace('<collision name="torre">', sensor.strip() + '\n        <collision name="torre">', 1)

    # Fuera MarsLink; un JointController de velocidad por rueda.
    t, n = re.subn(r'\s*<!-- El ORDEN es el contrato.*?</plugin>', "", t, count=1, flags=re.S)
    assert n == 1, "no encontre el plugin MarsLink"
    w = speed / WHEEL_RADIUS
    ctrl = []
    for c in ("fl", "fr", "bl", "br"):
        ctrl.append(
            f'      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">'
            f"<joint_name>{c}_drive_joint</joint_name><use_force_commands>true</use_force_commands>"
            f"<p_gain>2.0</p_gain><i_gain>5.0</i_gain><d_gain>0</d_gain><i_max>3</i_max><i_min>-3</i_min>"
            f"<cmd_max>{WHEEL_TORQUE_MAX:.3f}</cmd_max><cmd_min>{-WHEEL_TORQUE_MAX:.3f}</cmd_min>"
            f"<initial_velocity>{w:.4f}</initial_velocity></plugin>")
    return t.replace("\n    </model>", "\n" + "\n".join(ctrl) + "\n    </model>", 1)


def bump(name: str, x: float, y: float) -> str:
    """Dos losas inclinadas cuya cara de arriba es la rampa (igual que _rampas() de cancha_2026.py)."""
    ang = math.atan2(BUMP_HEIGHT, BUMP_HALF)
    largo = math.hypot(BUMP_HALF, BUMP_HEIGHT)
    cx = BUMP_X0 + BUMP_HALF
    out = [f'    <model name="{name}"><static>true</static><pose>{x:.4f} {y:.4f} 0 0 0 0</pose><link name="link">']
    for i, s in enumerate((-1, 1), start=1):
        mx, mz = cx + s * BUMP_HALF / 2, BUMP_HEIGHT / 2
        p = s * ang
        nx, nz = math.sin(p), math.cos(p)
        pose = f"{mx - nx * SLAB / 2:.4f} 0 {mz - nz * SLAB / 2:.4f} 0 {p:.5f} 0"
        geo = f"<geometry><box><size>{largo:.4f} {BUMP_WIDTH} {SLAB}</size></box></geometry>"
        out.append(f'      <collision name="rampa_{i}"><pose>{pose}</pose>{geo}'
                   f"<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>")
        out.append(f'      <visual name="rampa_{i}"><pose>{pose}</pose>{geo}'
                   f"<material><ambient>0.35 0.32 0.27 1</ambient><diffuse>0.55 0.5 0.42 1</diffuse></material></visual>")
    out.append("    </link></model>")
    return "\n".join(out)


def lane_marker(name: str, x: float, y: float, color: tuple) -> str:
    """Franja de color al inicio del carril para reconocer la escena en la ventana."""
    r, g, b = color
    return (f'    <model name="{name}"><static>true</static><pose>{x - 0.9:.4f} {y:.4f} 0.001 0 0 0</pose><link name="link">'
            f'<visual name="v"><geometry><box><size>0.25 1.2 0.002</size></box></geometry>'
            f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>"
            f"</link></model>")


SPEED_COLORS = [(0.2, 0.6, 1.0), (0.2, 0.8, 0.4), (1.0, 0.75, 0.1), (1.0, 0.35, 0.2), (0.75, 0.3, 0.9)]


def build(mars_sim: Path, directions, speeds, lifts) -> tuple[str, list]:
    base, mec = robot_model(mars_sim)
    scenes = []
    for col, d in enumerate(directions):
        row = 0
        for lift in lifts:
            for v in speeds:
                tag = f"{d}_{str(v).replace('.', 'p')}ms" + (f"_mas{round(lift * 100)}cm" if lift else "")
                ux, uy = DIRECTIONS[d]
                scenes.append({"name": tag, "direction": d, "speed": v, "lift": lift,
                               "x": col * COLUMN, "y": row * LANE, "yaw": -math.atan2(uy, ux)})
                row += 1
    parts = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.11">',
        '  <world name="mars">',
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
        '<collision name="collision"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>'
        "<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>"
        '<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>'
        "<material><ambient>0.3 0.3 0.33 1</ambient><diffuse>0.35 0.35 0.38 1</diffuse></material></visual>"
        "</link></model>",
    ]
    for s in scenes:
        parts.append(f"    <!-- escena {s['name']}: {s['direction']} a {s['speed']} m/s, chasis +{s['lift'] * 100:.0f} cm -->")
        parts.append(bump(f"bump_{s['name']}", s["x"], s["y"]))
        parts.append(lane_marker(f"marca_{s['name']}", s["x"], s["y"],
                                 SPEED_COLORS[speeds.index(s["speed"]) % len(SPEED_COLORS)]))
        parts.append("    " + scene_robot(base, mec, s["name"], s["x"], s["y"], s["direction"], s["speed"], s["lift"]))
    parts += ["  </world>", "</sdf>", ""]
    return "\n".join(parts), scenes


def gui_config(mars_sim: Path, scenes, out: Path):
    """La configuracion de ventana de MARS con la camara encuadrando toda la cuadricula."""
    cfg = (mars_sim / "gui" / "mars.config").read_text(encoding="utf-8")
    x1 = max(s["x"] for s in scenes) + 5.0
    y1 = max(s["y"] for s in scenes) + 1.5
    cx, cy = x1 / 2 - 0.5, y1 / 2 - 0.5
    span = max(x1, y1 * 1.6, 8.0)
    pose = f"{cx:.2f} {cy - 0.75 * span:.2f} {0.5 * span:.2f} 0 0.58 1.5708"
    cfg, n = re.subn(r"<camera_pose>[^<]*</camera_pose>", f"<camera_pose>{pose}</camera_pose>", cfg, count=1)
    if n != 1:
        print("[gen] aviso: no encontre <camera_pose> en mars.config; la camara queda la de MARS")
    out.write_text(cfg, encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mars-sim", type=Path, default=default_mars_sim(), help="carpeta sim/ de la instalacion de MARS")
    ap.add_argument("--speeds", default="0.5,1,2,3")
    ap.add_argument("--directions", default=",".join(BENCH_DIRECTIONS))
    ap.add_argument("--lifts-cm", default="0")
    ap.add_argument("--out", type=Path, default=HERE / "build" / "bump_bench.sdf")
    a = ap.parse_args()
    speeds = [float(s) for s in a.speeds.split(",")]
    dirs = a.directions.split(",")
    for d in dirs:
        if d not in DIRECTIONS:
            raise SystemExit(f"direccion desconocida: {d} (hay: {', '.join(DIRECTIONS)})")
    lifts = [float(s) / 100 for s in a.lifts_cm.split(",")]
    sdf, scenes = build(a.mars_sim, dirs, speeds, lifts)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(sdf, encoding="utf-8", newline="\n")
    meta = {"bump": {"x0": BUMP_X0, "half": BUMP_HALF, "height": BUMP_HEIGHT}, "lane": LANE, "wheel_radius": WHEEL_RADIUS,
            "wheel_torque_max_nm": WHEEL_TORQUE_MAX, "scenes": scenes}
    a.out.with_suffix(".json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    gui_config(a.mars_sim, scenes, a.out.with_name("gui.config"))
    print(f"{len(scenes)} escenas -> {a.out}  (par maximo por rueda {WHEEL_TORQUE_MAX:.2f} N*m)")


if __name__ == "__main__":
    main()

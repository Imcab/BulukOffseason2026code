#!/usr/bin/env python3
"""Mide las escenas del banco mientras corren: quien cruza, quien se atora, pitch y roces.

Se suscribe a las poses (/world/mars/pose/info) y al sensor de contacto del
bumper de cada robot. Necesita el python del entorno mars-sim (gz.transport).

  python measure_bench.py build/bump_bench.json --seconds 10
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

from gz.msgs.clock_pb2 import Clock
from gz.msgs.contacts_pb2 import Contacts
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.double_pb2 import Double
from gz.msgs.world_control_pb2 import WorldControl
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node

ROBOT_HALF = 0.64  # media diagonal del bumper: el robot entero ya paso cuando su centro esta asi de lejos


def pitch_of(q) -> float:
    """Inclinacion del chasis respecto al piso, sin importar el yaw (angulo entre su Z y la vertical)."""
    # z del cuerpo en el mundo = tercera columna de R(q)
    zx = 2 * (q.x * q.z + q.w * q.y)
    zy = 2 * (q.y * q.z - q.w * q.x)
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    tilt = math.degrees(math.atan2(math.hypot(zx, zy), zz))
    # Subiendo, la Z del chasis se inclina hacia atras (-X del mundo): eso es nariz arriba, +.
    return tilt if zx <= 0 else -tilt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("meta", type=Path)
    ap.add_argument("--seconds", type=float, default=10.0, help="segundos de SIMULACION a medir")
    ap.add_argument("--timeout", type=float, default=180.0, help="segundos reales maximos esperando")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--unpause-after", type=float, default=None,
                    help="segundos reales a esperar (a que abra la ventana) antes de darle Play")
    a = ap.parse_args()
    meta = json.loads(a.meta.read_text(encoding="utf-8"))
    scenes = {s["name"]: s for s in meta["scenes"]}
    end_x = meta["bump"]["x0"] + 2 * meta["bump"]["half"] + ROBOT_HALF  # relativo al inicio de cada escena
    st = {n: {"t_cross": None, "max_pitch": 0.0, "min_pitch": 0.0, "max_z": 0.0, "x": 0.0, "lat": 0.0,
              "contact_s": 0.0, "last_contact": None, "x_hist": [], "trace": [], "touch": False,
              "floor": False} for n in scenes}
    lock = threading.Lock()
    sim_t = [0.0]

    def on_clock(msg: Clock):
        sim_t[0] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg: Pose_V):
        t = sim_t[0]
        with lock:
            for p in msg.pose:
                s = st.get(p.name)
                if s is None:
                    continue
                x0, y0 = scenes[p.name]["x"], scenes[p.name]["y"]
                px = p.position.x - x0
                s["x"], s["lat"] = px, p.position.y - y0
                s["max_z"] = max(s["max_z"], p.position.z)
                pt = pitch_of(p.orientation)
                s["max_pitch"], s["min_pitch"] = max(s["max_pitch"], pt), min(s["min_pitch"], pt)
                if s["t_cross"] is None and px > end_x:
                    s["t_cross"] = t
                s["x_hist"].append((t, px))
                # traza cada 20 ms: t, x, y, z, pitch, tocando el bump (para el visor y para depurar)
                if not s["trace"] or t - s["trace"][-1][0] >= 0.02:
                    s["trace"].append([round(t, 3), round(px, 4), round(p.position.y - y0, 4),
                                       round(p.position.z, 4), round(pt, 2), int(s["touch"])])

    def contact_cb(name):
        def cb(msg: Contacts):
            names = [c.collision1.name + "|" + c.collision2.name for c in msg.contact]
            hit = any("bump_" in n for n in names)
            floor = any("ground_plane" in n for n in names)
            t = sim_t[0]
            with lock:
                s = st[name]
                s["touch"] = hit
                if floor:
                    s["floor"] = True
                if hit:
                    if s["last_contact"] is not None and t - s["last_contact"] < 0.05:
                        s["contact_s"] += t - s["last_contact"]
                    s["last_contact"] = t
        return cb

    node = Node()
    node.subscribe(Clock, "/world/mars/clock", on_clock)
    node.subscribe(Pose_V, "/world/mars/pose/info", on_pose)
    for n in scenes:
        # El sistema Contact publica en su propio topico (ignora <topic> del sensor).
        node.subscribe(Contacts, f"/world/mars/model/{n}/link/chassis/sensor/contacto/contact", contact_cb(n))

    # Frenar cada robot poco despues de cruzar, para que la cuadricula se quede en cuadro.
    stop_pubs = {n: [node.advertise(f"/model/{n}/joint/{c}_drive_joint/cmd_vel", Double)
                     for c in ("fl", "fr", "bl", "br")] for n in scenes}
    stopped = set()
    if a.unpause_after is not None:
        time.sleep(a.unpause_after)
        ok, _ = node.request("/world/mars/control", WorldControl(pause=False), WorldControl, Boolean, 5000)
        print("[medir] simulacion en marcha" if ok else "[medir] no pude darle Play; usa el boton de la ventana")
    t0 = time.time()
    while sim_t[0] < a.seconds and time.time() - t0 < a.timeout:
        time.sleep(0.1)
        with lock:
            to_stop = [n for n, s in st.items()
                       if n not in stopped and s["t_cross"] is not None and sim_t[0] > s["t_cross"] + 0.6]
        for n in to_stop:
            for pub in stop_pubs[n]:
                pub.publish(Double(data=0.0))
            stopped.add(n)
    if sim_t[0] < a.seconds:
        print(f"[medir] ojo: la simulacion solo llego a {sim_t[0]:.1f} s (esta corriendo el servidor?)")

    rows = []
    with lock:
        for n, s in st.items():
            sc = scenes[n]
            hist = [x for t, x in s["x_hist"] if t > sim_t[0] - 1.0]
            moving = (max(hist) - min(hist)) > 0.05 if hist else False
            if s["t_cross"] is not None:
                res = "cruza+roce" if s["contact_s"] > 0 else "CRUZA"
            else:
                res = "en camino" if moving else "ATORADO"
            if s["floor"]:
                res += " (bumper al piso)"
            rows.append({"escena": n, "direccion": sc["direction"], "vel": sc["speed"], "lift_cm": round(sc["lift"] * 100),
                         "resultado": res, "t_cruce": s["t_cross"], "roce_s": round(s["contact_s"], 2),
                         "pitch_min": round(s["min_pitch"], 1), "pitch_max": round(s["max_pitch"], 1),
                         "x_final": round(s["x"], 2), "desvio_lateral": round(s["lat"], 2), "traza": s["trace"]})
    print(f"{'direccion':16s} {'chasis':>6s} {'vel':>4s} | {'resultado':26s} {'t cruce':>7s} {'roce':>5s} "
          f"{'pitch':>11s} {'desvio':>6s}")
    for r in rows:
        tc = f"{r['t_cruce']:.2f}" if r["t_cruce"] is not None else "-"
        print(f"{r['direccion']:16s} {('+%d cm' % r['lift_cm']) if r['lift_cm'] else 'actual':>6s} {r['vel']:4.1f} | "
              f"{r['resultado']:26s} {tc:>7s} {r['roce_s']:5.2f} {r['pitch_min']:5.1f}/{r['pitch_max']:4.1f} "
              f"{r['desvio_lateral']:6.2f}")
    if a.out:
        a.out.write_text(json.dumps({"sim_seconds": sim_t[0], "rows": rows}, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()

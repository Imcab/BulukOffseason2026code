"""Generador de trayectorias SINTETICAS (no hay paths de PathPlanner en el proyecto).

Perfil de velocidad "tiempo-optimo" sobre una curva parametrica, respetando:
  - |v| <= max_vel, aceleracion tangencial <= max_acc
  - aceleracion centripeta v^2*kappa <= max_acc
  - |omega| <= max_ang_vel y |alpha| <= max_ang_acc (rotacion holonomica)
La rotacion se interpola con smoothstep sobre una ventana del avance del path, parecido
a como PathPlanner interpola entre rotation targets.

Diferencias conocidas con PathPlanner: PathPlanner limita con un modelo de fuerzas por
modulo (RobotConfig) y combina aceleracion tangencial + centripeta; aqui se limitan por
separado, asi que en curvas la aceleracion total puede llegar a ~1.4*max_acc.
"""

from __future__ import annotations

import math

import numpy as np


def _smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def _profile_along(s, vcap, a_max):
    v = np.minimum(vcap, np.inf)
    v[0] = 0.0
    v[-1] = 0.0
    ds = np.diff(s)
    for i in range(len(s) - 1):
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_max * ds[i]))
    for i in range(len(s) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
    return v


def _times(s, v):
    ds = np.diff(s)
    vm = v[:-1] + v[1:]
    dt = np.where(vm > 1e-9, 2 * ds / np.maximum(vm, 1e-9), 0.0)
    return np.concatenate([[0.0], np.cumsum(dt)])


def generate_path(name, description, curve, theta0, dtheta, rot_window, c, dt=0.02, ds=0.005):
    """curve: f(u) -> (x, y) con u en [0,1]. rot_window: (u0, u1) fraccion de longitud."""
    v_max, a_max = c["max_vel_mps"], c["max_acc_mps2"]
    w_max, al_max = math.radians(c["max_ang_vel_deg_s"]), math.radians(c["max_ang_acc_deg_s2"])

    u = np.linspace(0, 1, 4001)
    xy = np.array([curve(ui) for ui in u])
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s_u = np.concatenate([[0.0], np.cumsum(seg)])
    L = s_u[-1]
    s = np.linspace(0, L, max(int(L / ds), 2) + 1)
    x = np.interp(s, s_u, xy[:, 0])
    y = np.interp(s, s_u, xy[:, 1])
    tx, ty = np.gradient(x, s), np.gradient(y, s)
    psi = np.unwrap(np.arctan2(ty, tx))
    kappa = np.abs(np.gradient(psi, s))

    u0, u1 = rot_window
    th = theta0 + dtheta * _smoothstep((s / L - u0) / (u1 - u0))
    dth = np.gradient(th, s)
    ddth = np.gradient(dth, s)

    vcap = np.full_like(s, v_max)
    vcap = np.minimum(vcap, np.sqrt(a_max / np.maximum(kappa, 1e-9)))
    vcap = np.minimum(vcap, w_max / np.maximum(np.abs(dth), 1e-9))
    vcap = np.minimum(vcap, np.sqrt(al_max / np.maximum(np.abs(ddth), 1e-9)))

    # Itera bajando el tope donde alpha = th''*v^2 + th'*a se pase del limite.
    for _ in range(200):
        v = _profile_along(s, vcap.copy(), a_max)
        t = _times(s, v)
        a = np.gradient(v, s) * v
        alpha = ddth * v**2 + dth * a
        bad = np.abs(alpha) > al_max * 0.98
        if not bad.any():
            break
        vcap[bad] = v[bad] * 0.95
    else:
        raise RuntimeError(f"{name}: no se pudo cumplir max_ang_acc")

    T = float(t[-1])
    tq = np.append(np.arange(0.0, T - 1e-9, dt), T)
    sq = np.interp(tq, t, s)
    vq = np.interp(tq, t, v)
    txq, tyq = np.interp(sq, s, tx), np.interp(sq, s, ty)
    nrm = np.maximum(np.hypot(txq, tyq), 1e-12)
    return _pack(
        name, description, c, dt, tq,
        np.interp(sq, s, x), np.interp(sq, s, y), np.interp(sq, s, th),
        vq * txq / nrm, vq * tyq / nrm, np.interp(sq, s, dth) * vq,
    )


def generate_rotation(name, description, x, y, theta0, dtheta, c, dt=0.02):
    """Giro en el lugar con perfil trapezoidal (o triangular) en theta."""
    w_max, al = math.radians(c["max_ang_vel_deg_s"]), math.radians(c["max_ang_acc_deg_s2"])
    D = abs(dtheta)
    sgn = math.copysign(1.0, dtheta)
    t_acc = w_max / al
    if al * t_acc**2 >= D:  # triangular
        t_acc = math.sqrt(D / al)
        t_cru = 0.0
    else:
        t_cru = (D - al * t_acc**2) / w_max
    T = 2 * t_acc + t_cru
    tq = np.append(np.arange(0.0, T - 1e-9, dt), T)
    w_pk = al * t_acc

    def ang(tt):
        if tt < t_acc:
            return 0.5 * al * tt**2, al * tt
        if tt < t_acc + t_cru:
            return 0.5 * al * t_acc**2 + w_pk * (tt - t_acc), w_pk
        td = T - tt
        return D - 0.5 * al * td**2, al * td

    th, om = zip(*(ang(tt) for tt in tq))
    th = theta0 + sgn * np.array(th)
    om = sgn * np.array(om)
    z = np.zeros_like(tq)
    return _pack(name, description, c, dt, tq, z + x, z + y, th, z, z, om)


def _pack(name, description, c, dt, t, x, y, th, vx, vy, om):
    r = lambda a: [round(float(v), 6) for v in a]  # noqa: E731
    return {
        "name": name,
        "description": description,
        "generated_by": "core/trajgen.py (sintetica, no PathPlanner)",
        "dt": dt,
        "total_time": round(float(t[-1]), 6),
        "constraints": c,
        "samples": {"t": r(t), "x": r(x), "y": r(y), "theta": r(th), "vx": r(vx), "vy": r(vy), "omega": r(om)},
    }


def _line(p0, p1):
    return lambda u: (p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1]))


def _bezier(p0, p1, p2, p3):
    def f(u):
        a, b, cc, d = (1 - u) ** 3, 3 * u * (1 - u) ** 2, 3 * u**2 * (1 - u), u**3
        return (a * p0[0] + b * p1[0] + cc * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + cc * p2[1] + d * p3[1])
    return f


def _arc(r):
    return lambda u: (r * math.sin(u * math.pi / 2), r * (1 - math.cos(u * math.pi / 2)))


def standard_set(c: dict) -> list[dict]:
    pi = math.pi
    return [
        generate_path("recta_4m", "Recta 4 m hacia adelante, sin giro", _line((0, 0), (4, 0)), 0, 0, (0, 1), c),
        generate_path("recta_6m_giro180", "Recta 6 m girando 180 grados en el trayecto",
                      _line((0, 0), (6, 0)), 0, pi, (0.05, 0.95), c),
        generate_path("curva_s_lateral", "Curva en S: 4 m adelante y 1.5 m de lado, sin giro",
                      _bezier((0, 0), (2, 0), (2, 1.5), (4, 1.5)), 0, 0, (0, 1), c),
        generate_path("diagonal_giro90", "Diagonal (3, 2) m girando 90 grados",
                      _line((0, 0), (3, 2)), 0, pi / 2, (0.1, 0.9), c),
        generate_path("lateral_2m", "Strafe puro 2 m a la izquierda, sin giro", _line((0, 0), (0, 2)), 0, 0, (0, 1), c),
        generate_path("corto_1m", "Recta corta 1 m (sensible al error final)", _line((0, 0), (1, 0)), 0, 0, (0, 1), c),
        generate_path("arco_90_giro90", "Arco de 90 grados con radio 2 m, girando 90 grados",
                      _arc(2.0), 0, pi / 2, (0.0, 1.0), c),
        generate_rotation("giro_sitio_180", "Giro de 180 grados en el lugar", 0, 0, 0, pi, c),
    ]

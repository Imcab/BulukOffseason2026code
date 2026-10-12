"""Prueba isaac/run_sweep.py con un Isaac Lab FALSO (stubs) que integra con las mismas ecuaciones.

Esto NO valida la API real de Isaac Lab. Solo valida la logica propia de la capa: conversion
CM <-> centro, quaternion wxyz <-> yaw, origenes de cada entorno, marco de las fuerzas (cuerpo),
y que masa/inercia por entorno se escriben en el lugar correcto. Si esto pasa y el script real
falla, el problema esta en la API de Isaac (pega el error a Claude).
"""

import importlib.util
import math
import sys
import types
from pathlib import Path

import pytest
import torch

from core.model import TorchBody
from core.params import nominal_env_params, variant
from core.sim import run_trajectory

RUN_SWEEP = Path(__file__).resolve().parent.parent / "isaac" / "run_sweep.py"


class _Cfg:
    def __init__(self, *a, **kw):
        self.__dict__.update(kw)


class _FakeView:
    def __init__(self, n):
        self.m = torch.full((n, 1), 1.0)
        self.i = torch.zeros(n, 9)

    def get_masses(self):
        return self.m.clone()

    def set_masses(self, m, idx):
        self.m[idx.long()] = m[idx.long()]

    def get_inertias(self):
        return self.i.clone()

    def set_inertias(self, i, idx):
        self.i[idx.long()] = i[idx.long()]


class _FakeData:
    pass


class _FakeRobot:
    def __init__(self, n, dt):
        self.device = "cpu"
        self.n, self.dt = n, dt
        self.root_physx_view = _FakeView(n)
        self.data = _FakeData()
        self.data.root_pos_w = torch.zeros(n, 3)
        self.data.root_quat_w = torch.tensor([[1.0, 0, 0, 0]]).repeat(n, 1)
        self.data.root_lin_vel_w = torch.zeros(n, 3)
        self.data.root_ang_vel_w = torch.zeros(n, 3)
        self.permanent_wrench_composer = types.SimpleNamespace(set_forces_and_torques=self._set)
        self.f = torch.zeros(n, 1, 3)
        self.t = torch.zeros(n, 1, 3)

    def _set(self, forces, torques, is_global):
        assert not is_global and forces.shape == (self.n, 1, 3)
        self.f, self.t = forces.clone(), torques.clone()

    def write_root_pose_to_sim(self, pose):
        self.data.root_pos_w = pose[:, :3].clone()
        self.data.root_quat_w = pose[:, 3:7].clone()

    def write_root_velocity_to_sim(self, vel):
        self.data.root_lin_vel_w = vel[:, :3].clone()
        self.data.root_ang_vel_w = vel[:, 3:].clone()

    def reset(self):
        pass

    def physics_step(self):
        """Euler semi-implicito con fuerza en marco del cuerpo, como PhysX con is_global=False."""
        d = self.data
        q = d.root_quat_w
        yaw = 2 * torch.atan2(q[:, 3], q[:, 0])
        c, s = torch.cos(yaw), torch.sin(yaw)
        fx = c * self.f[:, 0, 0] - s * self.f[:, 0, 1]
        fy = s * self.f[:, 0, 0] + c * self.f[:, 0, 1]
        m = self.root_physx_view.m[:, 0]
        izz = self.root_physx_view.i[:, 8]
        d.root_lin_vel_w[:, 0] += fx / m * self.dt
        d.root_lin_vel_w[:, 1] += fy / m * self.dt
        d.root_ang_vel_w[:, 2] += self.t[:, 0, 2] / izz * self.dt
        d.root_pos_w[:, :2] += d.root_lin_vel_w[:, :2] * self.dt
        yaw = yaw + d.root_ang_vel_w[:, 2] * self.dt
        d.root_quat_w = torch.stack([torch.cos(yaw / 2), 0 * yaw, 0 * yaw, torch.sin(yaw / 2)], dim=-1)


def _install_stubs(monkeypatch):
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    class AppLauncher:
        @staticmethod
        def add_app_launcher_args(p):
            p.add_argument("--headless", action="store_true")
            p.add_argument("--device", default="cpu")

        def __init__(self, args):
            self.app = types.SimpleNamespace(close=lambda: None)

    mod("isaaclab")
    mod("isaaclab.app", AppLauncher=AppLauncher)
    mod("isaaclab.sim", CuboidCfg=_Cfg, RigidBodyPropertiesCfg=_Cfg, MassPropertiesCfg=_Cfg,
        SimulationCfg=_Cfg, SimulationContext=_Cfg)
    mod("isaaclab.assets", RigidObjectCfg=type("RigidObjectCfg", (_Cfg,), {"InitialStateCfg": _Cfg}))
    mod("isaaclab.scene", InteractiveScene=_Cfg, InteractiveSceneCfg=_Cfg)
    mod("isaaclab.utils", configclass=lambda c: c)


def _load_run_sweep(monkeypatch):
    _install_stubs(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["run_sweep.py", "--headless"])
    spec = importlib.util.spec_from_file_location("isaac_run_sweep_mock", RUN_SWEEP)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.mark.parametrize("traj_name", ["diagonal_giro90", "giro_sitio_180"])
def test_isaac_body_matches_torch_body(cfg, trajs, monkeypatch, traj_name):
    rs = _load_run_sweep(monkeypatch)
    n, dt = 3, cfg["sim"]["physics_dt_s"]
    env = nominal_env_params(cfg, [3.0, 6.0, 9.0], [3.0, 6.0, 9.0], latency_s=0.04,
                             init_dx=0.1, init_dy=-0.05, init_dyaw=math.radians(4))
    env.moi[:] = torch.tensor([6.6, 7.0, 7.4])
    env.mass[:] = torch.tensor([57.0, 58.0, 59.0])

    robot = _FakeRobot(n, dt)
    origins = torch.tensor([[0.0, 0, 0], [1.5, 0, 0], [0, 1.5, 0]])
    scene = types.SimpleNamespace(env_origins=origins, write_data_to_sim=lambda: None, update=lambda dt: None)
    sim = types.SimpleNamespace(step=lambda render=False: robot.physics_step())
    body = rs.IsaacBody(sim, scene, robot, dt).configure(env)
    assert robot.root_physx_view.m[:, 0].tolist() == pytest.approx([57.0, 58.0, 59.0])
    assert robot.root_physx_view.i[:, 8].tolist() == pytest.approx([6.6, 7.0, 7.4])

    tr = trajs[traj_name]
    var = variant(cfg)
    r_isaac = run_trajectory(cfg, var, env, tr, body)
    r_torch = run_trajectory(cfg, var, env, tr, TorchBody(env, dt))
    for k in ("pos_rms_m", "final_pos_err_m", "final_yaw_err_deg", "max_yaw_err_deg"):
        assert r_isaac[k].tolist() == pytest.approx(r_torch[k].tolist(), abs=2e-3), k

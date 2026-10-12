"""Sweep de kP con Isaac Lab como integrador del cuerpo rigido.

*** NO PROBADO *** - escrito sin acceso a Isaac Sim. Si falla, copia el error completo
y pegalo de vuelta en la conversacion con Claude.

Version asumida: Isaac Lab v2.3.2 (Isaac Sim 4.5 / 5.0 / 5.1, Python 3.11).
API verificada contra el codigo fuente de esa version (source/isaaclab/isaaclab/...):
  - isaaclab.app.AppLauncher (add_app_launcher_args, .app)
  - isaaclab.scene.InteractiveScene / InteractiveSceneCfg(num_envs, env_spacing, replicate_physics)
  - isaaclab.assets.RigidObject / RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Robot", spawn, init_state)
  - isaaclab.sim.CuboidCfg, RigidBodyPropertiesCfg, MassPropertiesCfg, SimulationCfg(dt, device, gravity)
  - RigidObject.permanent_wrench_composer.set_forces_and_torques(forces, torques, is_global=False)
    (en 2.3.x set_external_force_and_torque sigue existiendo pero imprime aviso de deprecacion)
  - RigidObject.write_root_pose_to_sim (N,7: pos + quat wxyz), write_root_velocity_to_sim (N,6)
  - RigidObject.data.root_pos_w / root_quat_w (wxyz) / root_lin_vel_w / root_ang_vel_w
  - root_physx_view.get_masses/set_masses, get_inertias/set_inertias (tensores en CPU)

Que hace Isaac aqui: SOLO integra el cuerpo rigido plano (masa, inercia yaw, CM). Las fuerzas
de modulos, traccion, latencia y el controlador PPHolonomic vienen de core/ (torch puro), igual
que en core/sweep_torch.py. No hay URDF, ni contacto con el piso, ni gravedad: el cuerpo flota
y recibe la fuerza/torque total de los 4 modulos en su CM. Por eso los resultados deberian
coincidir con core/sweep_torch.py; si difieren mucho, algo esta mal en esta capa.

Uso (desde la carpeta de Isaac Lab):
  ./isaaclab.sh -p /ruta/a/pid_sweep/isaac/run_sweep.py --headless --quick
  ./isaaclab.sh -p /ruta/a/pid_sweep/isaac/run_sweep.py --headless
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

parser = argparse.ArgumentParser(description="Sweep de kP de PathPlanner en Isaac Lab (NO PROBADO)")
parser.add_argument("--config", default=None, help="ruta a params.yaml (default: el del paquete)")
parser.add_argument("--out", default=str(PKG / "results"), help="carpeta de salida")
parser.add_argument("--quick", action="store_true", help="pocos entornos y 3 trayectorias")
AppLauncher.add_app_launcher_args(parser)  # agrega --headless, --device, etc.
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- Todo lo de Isaac se importa DESPUES de lanzar la app. ---
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from core.model import rotate  # noqa: E402
from core.params import DTYPE, build_sweep_grid, load_config  # noqa: E402
from core.sweep import quick_config, run_sweep  # noqa: E402

Z_HEIGHT = 0.2  # altura visual del cuerpo; no afecta la dinamica (sin gravedad ni contacto)


@configclass
class SweepSceneCfg(InteractiveSceneCfg):
    robot: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.CuboidCfg(
            size=(0.9017, 0.9017, 0.15),  # bumpers 35.5 in; solo visual, la inercia se fija abajo
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                linear_damping=0.0,
                angular_damping=0.0,
                sleep_threshold=0.0,  # un cuerpo dormido ignora fuerzas externas
                stabilization_threshold=0.0,
                enable_gyroscopic_forces=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=57.0),
            collision_props=None,  # sin colisiones: los entornos pueden traslaparse
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, Z_HEIGHT)),
    )


class IsaacBody:
    """Misma interfaz que core.model.TorchBody, integrando con PhysX."""

    def __init__(self, sim, scene, robot, dt: float):
        self.sim, self.scene, self.robot, self.dt = sim, scene, robot, dt
        self.device = robot.device
        self.origins = scene.env_origins[:, :2]
        self.env = None

    def configure(self, env):
        """Fija masa e inercia por entorno (una vez; los robots aleatorizados no cambian)."""
        if self.env is not None:
            self.env = env  # mismo robot, solo cambia el error inicial (prueba mantener_pose)
            return self
        self.env = env
        n = env.n
        view = self.robot.root_physx_view
        idx = torch.arange(n, dtype=torch.int32)  # los setters de masa/inercia usan indices en CPU
        masses = view.get_masses()
        masses.view(n, -1)[:, 0] = env.mass.cpu()
        view.set_masses(masses, idx)
        inert = view.get_inertias()
        flat = inert.view(n, -1, 9)
        flat.zero_()
        flat[:, 0, 0] = env.moi.cpu()  # Ixx: irrelevante (sin torques en x)
        flat[:, 0, 4] = env.moi.cpu()  # Iyy: irrelevante
        flat[:, 0, 8] = env.moi.cpu()  # Izz = MOI de yaw aleatorizado
        view.set_inertias(inert, idx)
        self.cg = torch.stack([env.cg_x, env.cg_y], dim=-1).to(self.device)
        print(f"[isaac] masas {float(env.mass.min()):.1f}-{float(env.mass.max()):.1f} kg, "
              f"Izz {float(env.moi.min()):.2f}-{float(env.moi.max()):.2f} kg m2 aplicadas a {n} entornos")
        return self

    def reset(self, center_xy: torch.Tensor, yaw: torch.Tensor):
        n = center_xy.shape[0]
        cx, cy = rotate(self.cg[:, 0], self.cg[:, 1], yaw)
        pos_cm = center_xy + torch.stack([cx, cy], dim=-1)
        pose = torch.zeros(n, 7, device=self.device)
        pose[:, :2] = pos_cm + self.origins
        pose[:, 2] = Z_HEIGHT
        pose[:, 3] = torch.cos(yaw / 2)  # quaternion wxyz, giro solo en z
        pose[:, 6] = torch.sin(yaw / 2)
        self.robot.write_root_pose_to_sim(pose)
        self.robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=self.device))
        self.robot.reset()
        self._apply(torch.zeros(n, 2, device=self.device), torch.zeros(n, device=self.device))
        self._pos, self._yaw = pos_cm.to(DTYPE), yaw.clone().to(DTYPE)
        self._vel = torch.zeros(n, 2, dtype=DTYPE, device=self.device)
        self._omega = torch.zeros(n, dtype=DTYPE, device=self.device)

    def state(self):
        return self._pos, self._yaw, self._vel, self._omega

    def _apply(self, force_b, torque):
        n = force_b.shape[0]
        f = torch.zeros(n, 1, 3, device=self.device)
        t = torch.zeros(n, 1, 3, device=self.device)
        f[:, 0, :2] = force_b
        t[:, 0, 2] = torque
        composer = getattr(self.robot, "permanent_wrench_composer", None)
        if composer is not None:  # Isaac Lab >= 2.3
            composer.set_forces_and_torques(forces=f, torques=t, is_global=False)
        else:  # versiones anteriores
            self.robot.set_external_force_and_torque(f, t)

    def step(self, force_b: torch.Tensor, torque: torch.Tensor):
        self._apply(force_b.float(), torque.float())
        self.scene.write_data_to_sim()
        self.sim.step(render=False)
        self.scene.update(self.dt)
        d = self.robot.data
        self._pos = (d.root_pos_w[:, :2] - self.origins).to(DTYPE)
        q = d.root_quat_w  # wxyz
        self._yaw = 2 * torch.atan2(q[:, 3], q[:, 0]).to(DTYPE)  # solo giro en z
        self._vel = d.root_lin_vel_w[:, :2].to(DTYPE)
        self._omega = d.root_ang_vel_w[:, 2].to(DTYPE)


def main():
    cfg = load_config(args_cli.config)
    if args_cli.quick:
        cfg = quick_config(cfg)
    dt = float(cfg["sim"]["physics_dt_s"])
    n_env = build_sweep_grid(cfg)[0].numel()

    sim_cfg = sim_utils.SimulationCfg(dt=dt, device=args_cli.device, gravity=(0.0, 0.0, 0.0))
    sim = sim_utils.SimulationContext(sim_cfg)
    # replicate_physics=False para poder fijar masa/inercia distinta en cada entorno.
    scene = InteractiveScene(SweepSceneCfg(num_envs=n_env, env_spacing=1.5, replicate_physics=False))
    sim.reset()
    robot = scene["robot"]
    device = robot.device
    print(f"[isaac] escena lista: {n_env} entornos, dt={dt}, device={device}")

    body = IsaacBody(sim, scene, robot, dt)
    run_sweep(cfg, lambda env: body.configure(env.to(device)), device=device, out_dir=args_cli.out,
              backend=f"isaaclab-{device}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

"""PPHolonomicDriveController vectorizado (kI = 0, kD = 0).

Replica PathplannerLib 2026.1.2, PPHolonomicDriveController.calculateRobotRelativeSpeeds():
  xFF, yFF, rotFF = velocidades de CAMPO de la trayectoria
  xFeedback = kP_t * (x_obj - x),  yFeedback = kP_t * (y_obj - y)
  rotFeedback = kP_r * wrap(theta_obj - theta)    (enableContinuousInput(-pi, pi))
  ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFb, yFF + yFb, rotFF + rotFb, pose.rotation)
"""

from __future__ import annotations

import torch

from .model import rotate, wrap


class CommandDelay:
    """Retraso puro por entorno (en pasos de fisica) entre el comando calculado y el aplicado."""

    def __init__(self, n: int, delay_steps: torch.Tensor, width: int = 3):
        self.delay = delay_steps.long()
        self.size = int(self.delay.max()) + 1
        self.buf = torch.zeros(n, self.size, width, dtype=torch.float32, device=delay_steps.device)
        self.idx = 0
        self.rows = torch.arange(n, device=delay_steps.device)

    def push_and_get(self, cmd: torch.Tensor) -> torch.Tensor:
        self.buf[:, self.idx] = cmd
        out = self.buf[self.rows, (self.idx - self.delay) % self.size]
        self.idx = (self.idx + 1) % self.size
        return out


def pp_holonomic(pose_xy: torch.Tensor, yaw: torch.Tensor, target: dict, kp_t: torch.Tensor, kp_r: torch.Tensor):
    """Devuelve (ChassisSpeeds robot-relative (N,3), feedback de campo (N,3))."""
    fb_x = kp_t * (target["x"] - pose_xy[:, 0])
    fb_y = kp_t * (target["y"] - pose_xy[:, 1])
    fb_w = kp_r * wrap(target["theta"] - yaw)
    vx_f = target["vx"] + fb_x
    vy_f = target["vy"] + fb_y
    vx_r, vy_r = rotate(vx_f, vy_f, -yaw)
    cmd = torch.stack([vx_r, vy_r, target["omega"] + fb_w], dim=-1)
    return cmd, torch.stack([fb_x, fb_y, fb_w], dim=-1)

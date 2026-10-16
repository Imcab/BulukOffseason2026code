"""PPHolonomicDriveController vectorizado y retraso de comandos."""

import math

import pytest
import torch

from core.controller import CommandDelay, pp_holonomic


def _target(**kw):
    t = dict(x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0, omega=0.0)
    t.update(kw)
    return t


def test_feedforward_plus_p():
    pose = torch.tensor([[1.0, 2.0]])
    cmd, fb = pp_holonomic(pose, torch.tensor([0.0]), _target(x=1.5, y=1.0, vx=2.0, omega=0.5),
                           torch.tensor([4.0]), torch.tensor([3.0]))
    assert fb[0].tolist() == pytest.approx([2.0, -4.0, 0.0])
    assert cmd[0].tolist() == pytest.approx([4.0, -4.0, 0.5])


def test_rotation_error_is_continuous():
    yaw = torch.tensor([math.radians(179)])
    _, fb = pp_holonomic(torch.zeros(1, 2), yaw, _target(theta=math.radians(-179)), torch.tensor([1.0]),
                         torch.tensor([10.0]))
    assert float(fb[0, 2]) == pytest.approx(10 * math.radians(2), rel=1e-4)


def test_field_to_robot_relative():
    # Robot mirando +y (90 deg): una velocidad de campo +x es -y en el marco del robot.
    cmd, _ = pp_holonomic(torch.zeros(1, 2), torch.tensor([math.pi / 2]), _target(vx=1.0, theta=math.pi / 2),
                          torch.tensor([0.0]), torch.tensor([0.0]))
    assert cmd[0, 0] == pytest.approx(0.0, abs=1e-6)
    assert cmd[0, 1] == pytest.approx(-1.0, abs=1e-6)


def test_command_delay():
    d = CommandDelay(2, torch.tensor([0, 3]))
    outs = [d.push_and_get(torch.full((2, 3), float(k))) for k in range(6)]
    assert [float(o[0, 0]) for o in outs] == [0, 1, 2, 3, 4, 5]
    assert [float(o[1, 0]) for o in outs] == [0, 0, 0, 0, 1, 2]

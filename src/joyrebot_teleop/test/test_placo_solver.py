import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from joyrebot_teleop.kinematics import SerialChain
from joyrebot_teleop.placo_solver import PlacoChain


URDF = Path(__file__).parents[1] / "config/rebot_b601_kinematics.urdf"
HOME = np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0])


class FakeState:
    def __init__(self):
        # PlaCo's RobotWrapper exposes a seven-value floating base followed by
        # the six arm joints.
        self.q = np.zeros(13)


class FakeRobot:
    def __init__(self, _):
        self.state = FakeState()
        self.joints = {f"joint{i}": 0.0 for i in range(1, 7)}
        self.limits = {}
        self.velocity_limits = {}
        self.update_count = 0

    def set_joint_limits(self, name, lower, upper):
        self.limits[name] = (lower, upper)

    def set_velocity_limit(self, name, limit):
        self.velocity_limits[name] = limit

    def set_joint(self, name, value):
        self.joints[name] = value

    def get_joint(self, name):
        return self.joints[name]

    def update_kinematics(self):
        self.update_count += 1

    def get_T_world_frame(self, _):
        transform = np.eye(4)
        transform[0, 3] = self.joints["joint1"]
        return transform


class FakeTask:
    def __init__(self, target):
        self.T_world_frame = np.asarray(target)
        self.configuration = None

    def configure(self, *args):
        self.configuration = args


class FakeSolver:
    def __init__(self, robot):
        self.robot = robot
        self.dt = None
        self.frame_task = None
        self.masked_base = False
        self.joint_limits_enabled = False
        self.velocity_limits_enabled = False
        self.solve_count = 0
        self.raise_error = False
        self.forced_step = None

    def mask_fbase(self, enabled):
        self.masked_base = enabled

    def enable_joint_limits(self, enabled):
        self.joint_limits_enabled = enabled

    def enable_velocity_limits(self, enabled):
        self.velocity_limits_enabled = enabled

    def add_frame_task(self, _, target):
        self.frame_task = FakeTask(target)
        return self.frame_task

    def add_manipulability_task(self, *_):
        return FakeTask(np.eye(4))

    def solve(self, apply):
        assert apply
        self.solve_count += 1
        if self.raise_error:
            raise RuntimeError("infeasible")
        step = (self.forced_step if self.forced_step is not None
                else self.robot.velocity_limits["joint1"] * self.dt)
        self.robot.joints["joint1"] += step
        return np.array([step])


def install_fake_placo(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "placo",
        SimpleNamespace(RobotWrapper=FakeRobot, KinematicsSolver=FakeSolver))


def test_qp_receives_soft_joint_and_real_velocity_limits(monkeypatch):
    install_fake_placo(monkeypatch)
    chain = PlacoChain(
        URDF, dt=0.02, joint_margin=0.03, max_joint_speed=0.5,
        position_weight=2.0, orientation_weight=0.4, solve_iterations=2)
    mechanical = SerialChain.from_urdf(URDF)

    assert np.allclose(chain.lower, mechanical.lower + 0.03)
    assert np.allclose(chain.upper, mechanical.upper - 0.03)
    for name, lower, upper in zip(chain.names, chain.lower, chain.upper):
        assert chain.robot.limits[name] == (lower, upper)
        assert chain.robot.velocity_limits[name] == 0.5
    assert chain.solver.dt == 0.01
    assert chain.solver.masked_base
    assert chain.solver.joint_limits_enabled
    assert chain.solver.velocity_limits_enabled
    assert chain.robot.state.q[6] == 1.0
    assert chain.frame_task.configuration == ("ee_pose", "soft", 2.0, 0.4)


def test_substeps_share_one_control_period(monkeypatch):
    install_fake_placo(monkeypatch)
    chain = PlacoChain(
        URDF, dt=0.02, max_joint_speed=0.5, solve_iterations=2)
    target = np.eye(4)
    target[0, 3] = 0.01

    solution, success = chain.inverse(target, HOME)

    assert success
    assert chain.solver.solve_count == 2
    assert np.isclose(solution[0] - HOME[0], 0.5 * 0.02)
    assert chain.last_diagnostics.target_reached
    assert chain.last_diagnostics.velocity_limited


def test_runtime_failure_holds_seed_and_reports_reason(monkeypatch):
    install_fake_placo(monkeypatch)
    chain = PlacoChain(URDF)
    chain.solver.raise_error = True

    solution, success = chain.inverse(np.eye(4), HOME)

    assert not success
    assert np.allclose(solution, HOME)
    assert "infeasible" in chain.last_diagnostics.failure


def test_native_result_crossing_velocity_boundary_is_rejected(monkeypatch):
    install_fake_placo(monkeypatch)
    chain = PlacoChain(URDF, dt=0.02, max_joint_speed=0.5)
    chain.solver.forced_step = 0.02

    solution, success = chain.inverse(np.eye(4), HOME)

    assert not success
    assert np.allclose(solution, HOME)
    assert "velocity limit" in chain.last_diagnostics.failure


def test_invalid_target_never_reaches_native_solver(monkeypatch):
    install_fake_placo(monkeypatch)
    chain = PlacoChain(URDF)

    solution, success = chain.inverse(np.full((4, 4), np.nan), HOME)

    assert not success
    assert np.allclose(solution, HOME)
    assert chain.solver.solve_count == 0
    assert chain.last_diagnostics.failure == "invalid target transform"


def test_default_position_priority_is_high(monkeypatch):
    install_fake_placo(monkeypatch)

    chain = PlacoChain(URDF)

    assert chain.frame_task.configuration == (
        "ee_pose", "soft", 100.0, 0.35)


def test_high_position_priority_bounds_drift_during_large_yaw():
    pytest.importorskip("placo")
    chain = PlacoChain(URDF, dt=1.0 / 60.0)
    target = chain.forward(HOME)
    fixed_position = target[:3, 3].copy()
    target = target.copy()
    target[:3, :3] = (
        Rotation.from_euler("z", 0.86).as_matrix()
        @ target[:3, :3])

    q = HOME.copy()
    position_errors = []
    for _ in range(60):
        q, success = chain.inverse(target, q)
        assert success
        position_errors.append(np.linalg.norm(
            chain.forward(q)[:3, 3] - fixed_position))

    # The previous 1.0:0.35 weights drifted about 18 cm in this stress case.
    assert max(position_errors) < 0.012

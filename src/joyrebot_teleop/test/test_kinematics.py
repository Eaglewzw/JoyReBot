from pathlib import Path

import numpy as np

from joyrebot_teleop.kinematics import SerialChain


URDF = Path(__file__).parents[1] / "config/rebot_b601_kinematics.urdf"


def test_chain_contains_six_arm_joints():
    chain = SerialChain.from_urdf(URDF)
    assert chain.names == [f"joint{i}" for i in range(1, 7)]


def test_inverse_recovers_nearby_pose():
    chain = SerialChain.from_urdf(URDF)
    reference = np.asarray([0.2, 1.0, 1.0, 0.1, -0.2, 0.3])
    target = chain.forward(reference)
    solution, success = chain.inverse(target, reference + 0.03)
    assert success
    error = chain.pose_error(chain.forward(solution), target)
    assert np.linalg.norm(error[:3]) < 0.004
    assert np.linalg.norm(error[3:]) < 0.04


def test_joint_limits_are_respected():
    chain = SerialChain.from_urdf(URDF)
    target = chain.forward(np.asarray([0.0, 1.2, 1.0, 0.0, 0.0, 0.0]))
    solution, _ = chain.inverse(target, np.full(6, 100.0))
    assert np.all(solution <= chain.upper)
    assert np.all(solution >= chain.lower)


def test_configured_home_supports_small_rotation_targets():
    chain = SerialChain.from_urdf(URDF)
    home = np.asarray([0.0, 0.3, 0.3, 0.0, 0.0, 0.0])
    home_pose = chain.forward(home)
    chain.lower += 0.02
    chain.upper -= 0.02
    from scipy.spatial.transform import Rotation
    for axis in range(3):
        for angle in (-np.deg2rad(10), np.deg2rad(10)):
            target = home_pose.copy()
            rotation = np.zeros(3)
            rotation[axis] = angle
            target[:3, :3] = (
                Rotation.from_rotvec(rotation).as_matrix() @ home_pose[:3, :3])
            solution, success = chain.inverse(
                target, home, orientation_tolerance=0.015)
            error = chain.pose_error(chain.forward(solution), target)
            assert success
            assert np.linalg.norm(error[:3]) <= 0.004
            assert np.linalg.norm(error[3:]) <= 0.015

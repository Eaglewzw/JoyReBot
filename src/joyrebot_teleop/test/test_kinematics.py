from pathlib import Path

import numpy as np

from joyrebot_teleop.kinematics import SerialChain


URDF = Path(__file__).parents[2] / "joyrebot_gazebo_sim/urdf/rebot_b601_rs.urdf"


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

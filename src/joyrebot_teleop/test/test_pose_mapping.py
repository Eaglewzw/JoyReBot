import numpy as np
from scipy.spatial.transform import Rotation

from joyrebot_teleop.pose_mapping import RelativePoseMapper


def make_mapper():
    return RelativePoseMapper([2, 1, 1], [1, 1, 1], [0, 1, 2], [1, -1, 1],
                              [0, 1, 2], [1, 1, 1], [0.7, 0.17, 0.8],
                              [-1, -1, -1], [1, 1, 1])


def test_relative_position_mapping_and_clamp():
    mapper = make_mapper()
    controller = np.eye(4)
    robot = np.eye(4)
    robot[:3, 3] = [0.5, 0.2, 0.3]
    mapper.engage(controller, robot)
    controller[:3, 3] = [0.4, 0.1, 2.0]
    output = mapper.map(controller)
    assert np.allclose(output[:3, 3], [1.0, 0.1, 1.0])


def test_rotation_is_relative_to_engagement():
    mapper = make_mapper()
    controller = np.eye(4)
    robot = np.eye(4)
    mapper.engage(controller, robot)
    controller[:3, :3] = Rotation.from_euler("z", 0.2).as_matrix()
    output = mapper.map(controller)
    assert np.allclose(Rotation.from_matrix(output[:3, :3]).as_rotvec(), [0, 0, 0.2])


def test_rotation_vector_is_limited_per_robot_axis():
    mapper = make_mapper()
    controller = np.eye(4)
    mapper.engage(controller, np.eye(4))
    controller[:3, :3] = Rotation.from_rotvec([0.9, -0.4, 1.0]).as_matrix()
    output = mapper.map(controller)
    assert np.allclose(
        Rotation.from_matrix(output[:3, :3]).as_rotvec(),
        [0.7, -0.17, 0.8], atol=1e-8)

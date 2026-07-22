import numpy as np
from scipy.spatial.transform import Rotation


def pose_to_matrix(position, quaternion):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = position
    return transform


class RelativePoseMapper:
    def __init__(self, position_scale, orientation_scale, position_map, position_sign,
                 orientation_map, orientation_sign, orientation_limit,
                 workspace_min, workspace_max):
        self.position_scale = np.asarray(position_scale)
        self.orientation_scale = np.asarray(orientation_scale)
        self.position_map = np.asarray(position_map, dtype=int)
        self.position_sign = np.asarray(position_sign)
        self.orientation_map = np.asarray(orientation_map, dtype=int)
        self.orientation_sign = np.asarray(orientation_sign)
        self.orientation_limit = np.asarray(orientation_limit, dtype=float)
        if self.orientation_limit.shape != (3,) or np.any(self.orientation_limit <= 0.0):
            raise ValueError("orientation_limit must contain three positive radians")
        self.workspace_min = np.asarray(workspace_min)
        self.workspace_max = np.asarray(workspace_max)
        self.input_anchor = None
        self.robot_anchor = None

    def engage(self, input_pose, robot_pose):
        self.input_anchor = np.array(input_pose, copy=True)
        self.robot_anchor = np.array(robot_pose, copy=True)

    def clear(self):
        self.input_anchor = self.robot_anchor = None

    def map(self, input_pose):
        if self.input_anchor is None:
            raise RuntimeError("Pose mapper is not engaged")
        delta_position = input_pose[:3, 3] - self.input_anchor[:3, 3]
        mapped_position = (delta_position[self.position_map] * self.position_sign * self.position_scale)
        output = np.array(self.robot_anchor, copy=True)
        output[:3, 3] = np.clip(self.robot_anchor[:3, 3] + mapped_position,
                                self.workspace_min, self.workspace_max)
        delta_rotation = Rotation.from_matrix(input_pose[:3, :3] @ self.input_anchor[:3, :3].T).as_rotvec()
        mapped_rotation = delta_rotation[self.orientation_map] * self.orientation_sign * self.orientation_scale
        mapped_rotation = np.clip(mapped_rotation, -self.orientation_limit,
                                  self.orientation_limit)
        output[:3, :3] = Rotation.from_rotvec(mapped_rotation).as_matrix() @ self.robot_anchor[:3, :3]
        return output

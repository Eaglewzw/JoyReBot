"""Small URDF serial-chain FK/IK implementation used by the teleop node."""

from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def _vector(text, default):
    return np.asarray([float(v) for v in text.split()], dtype=float) if text else np.asarray(default, dtype=float)


def _transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis, angle):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
    return result


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class SerialChain:
    def __init__(self, joints):
        self.joints = joints
        self.active = [joint for joint in joints if joint.kind in ("revolute", "continuous")]
        self.names = [joint.name for joint in self.active]
        self.lower = np.asarray([joint.lower for joint in self.active])
        self.upper = np.asarray([joint.upper for joint in self.active])

    @classmethod
    def from_urdf(cls, path, base_link="base_link", tip_link="gripper_end"):
        root = ET.parse(path).getroot()
        by_child = {}
        for element in root.findall("joint"):
            origin = element.find("origin")
            axis_element = element.find("axis")
            limit = element.find("limit")
            kind = element.attrib["type"]
            by_child[element.find("child").attrib["link"]] = Joint(
                element.attrib["name"], kind,
                element.find("parent").attrib["link"], element.find("child").attrib["link"],
                _transform(_vector(origin.attrib.get("xyz"), [0, 0, 0]) if origin is not None else [0, 0, 0],
                           _vector(origin.attrib.get("rpy"), [0, 0, 0]) if origin is not None else [0, 0, 0]),
                _vector(axis_element.attrib.get("xyz"), [1, 0, 0]) if axis_element is not None else np.zeros(3),
                float(limit.attrib.get("lower", -np.pi)) if limit is not None else -np.inf,
                float(limit.attrib.get("upper", np.pi)) if limit is not None else np.inf,
            )
        chain = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"No URDF chain from {base_link} to {tip_link}; stopped at {link}")
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return cls(chain)

    def forward(self, q):
        transform = np.eye(4)
        active_index = 0
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.kind in ("revolute", "continuous"):
                transform = transform @ _axis_rotation(joint.axis, q[active_index])
                active_index += 1
        return transform

    @staticmethod
    def pose_error(current, target):
        position = target[:3, 3] - current[:3, 3]
        rotation = Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()
        return np.concatenate((position, rotation))

    def jacobian(self, q, epsilon=1e-5):
        base = self.forward(q)
        jacobian = np.zeros((6, len(q)))
        for index in range(len(q)):
            shifted = np.array(q, copy=True)
            shifted[index] += epsilon
            jacobian[:, index] = self.pose_error(base, self.forward(shifted)) / epsilon
        return jacobian

    def inverse(self, target, seed, damping=0.06, max_iterations=120,
                position_tolerance=0.004, orientation_tolerance=0.04):
        q = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        for _ in range(max_iterations):
            error = self.pose_error(self.forward(q), target)
            if np.linalg.norm(error[:3]) <= position_tolerance and np.linalg.norm(error[3:]) <= orientation_tolerance:
                return q, True
            jacobian = self.jacobian(q)
            lhs = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
            step = jacobian.T @ np.linalg.solve(lhs, error)
            q = np.clip(q + np.clip(step, -0.12, 0.12), self.lower, self.upper)
        return q, False

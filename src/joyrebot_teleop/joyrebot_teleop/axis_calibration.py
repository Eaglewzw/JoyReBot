"""Interactive helper for deriving Joy-Con-to-robot axis configuration."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


AXIS_NAMES = ("X", "Y", "Z")


@dataclass
class AxisSample:
    output_axis: int
    input_axis: int
    sign: float
    magnitude: float


class AxisCalibration:
    """Collect three unambiguous motions and turn them into map/sign arrays."""

    def __init__(self, minimum_motion):
        self.minimum_motion = float(minimum_motion)
        self.samples = []

    def add(self, delta, output_axis):
        delta = np.asarray(delta, dtype=float)
        input_axis = int(np.argmax(np.abs(delta)))
        magnitude = float(abs(delta[input_axis]))
        if magnitude < self.minimum_motion:
            raise ValueError(
                f"motion too small ({magnitude:.4f} < {self.minimum_motion:.4f})")
        if any(sample.input_axis == input_axis for sample in self.samples):
            raise ValueError(
                f"input {AXIS_NAMES[input_axis]} axis was already used; isolate one axis per trial")
        sample = AxisSample(
            output_axis=int(output_axis), input_axis=input_axis,
            sign=1.0 if delta[input_axis] > 0.0 else -1.0,
            magnitude=magnitude)
        self.samples.append(sample)
        return sample

    @property
    def complete(self):
        return len(self.samples) == 3

    def result(self):
        if not self.complete:
            raise RuntimeError("three axis samples are required")
        ordered = sorted(self.samples, key=lambda sample: sample.output_axis)
        return ([sample.input_axis for sample in ordered],
                [sample.sign for sample in ordered])


def pose_components(message):
    p = message.pose.position
    q = message.pose.orientation
    position = np.asarray([p.x, p.y, p.z], dtype=float)
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return position, rotation

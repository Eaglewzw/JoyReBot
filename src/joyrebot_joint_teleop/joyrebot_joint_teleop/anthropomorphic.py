"""Anthropomorphic channel mapping: one Joy-Con degree of freedom per arm joint.

The Joy-Con offers exactly as many independent channels as the arm has joints --
three IMU rotations, two stick axes and one button pair -- so every joint can be
driven at once without any mode switching, the way a hand moves.

Each channel carries its own semantics:

``absolute``
    The joint follows the channel one-to-one from the anchor captured on engage.
    Correct for roll and pitch, which the IMU references against gravity.
``rate``
    The channel commands a joint velocity. Correct for yaw, which has no absolute
    reference and would otherwise drift the joint on its own, and for the
    self-centring stick and button channels.
``off``
    Channel ignored; the joint holds.
"""

import numpy as np


ABSOLUTE = "absolute"
RATE = "rate"
OFF = "off"
MODES = (ABSOLUTE, RATE, OFF)

# Fixed channel order. Everything configurable is a parallel array in this order.
CHANNEL_NAMES = ("roll", "pitch", "yaw", "stick_vertical", "stick_horizontal", "buttons")


def deadzone(value, width):
    """Suppress noise around neutral and rescale so the edge stays continuous."""
    magnitude = abs(float(value))
    if magnitude <= width:
        return 0.0
    return float(np.sign(value) * (magnitude - width))


class AnthropomorphicMap:
    """Bind the six Joy-Con channels to six joints and resolve one control cycle."""

    def __init__(self, joints, modes, scales, signs, deadzones, joint_count):
        arrays = (joints, modes, scales, signs, deadzones)
        if any(len(item) != len(CHANNEL_NAMES) for item in arrays):
            raise ValueError(f"every channel array must hold {len(CHANNEL_NAMES)} entries "
                             f"in the order {CHANNEL_NAMES}")
        if any(mode not in MODES for mode in modes):
            raise ValueError(f"channel_mode entries must be one of {MODES}")
        self.joints = [int(joint) for joint in joints]
        if any(joint < 0 or joint >= joint_count for joint in self.joints):
            raise ValueError(f"channel_joint entries must lie within 0..{joint_count - 1}")
        active = [joint for joint, mode in zip(self.joints, modes) if mode != OFF]
        if len(set(active)) != len(active):
            raise ValueError("two active channels drive the same joint")
        self.modes = list(modes)
        self.scales = np.asarray(scales, dtype=float)
        self.signs = np.asarray(signs, dtype=float)
        self.deadzones = np.asarray(deadzones, dtype=float)
        self.joint_count = int(joint_count)
        self.anchor_inputs = None
        self.anchor_command = None

    @property
    def engaged(self):
        return self.anchor_inputs is not None

    def engage(self, inputs, command):
        """Pin the absolute channels to the operator's current wrist pose.

        Called on startup and every time the clutch is released, so that letting
        go of the clutch never steps the arm.
        """
        self.anchor_inputs = np.asarray(inputs, dtype=float).copy()
        self.anchor_command = np.asarray(command, dtype=float).copy()

    def clear(self):
        self.anchor_inputs = self.anchor_command = None

    def target(self, inputs, command, dt):
        """Desired joint command for this cycle, before rate limiting and clamping."""
        if not self.engaged:
            raise RuntimeError("anthropomorphic map is not engaged")
        inputs = np.asarray(inputs, dtype=float)
        command = np.asarray(command, dtype=float)
        target = command.copy()
        for index, (joint, mode) in enumerate(zip(self.joints, self.modes)):
            if mode == OFF:
                continue
            delta = float(inputs[index] - self.anchor_inputs[index])
            gain = self.signs[index] * self.scales[index]
            if mode == ABSOLUTE:
                # Absolute channels stay tied to the anchor, so drift in the joint
                # command cannot accumulate cycle over cycle.
                target[joint] = self.anchor_command[joint] + gain * delta
            else:
                target[joint] = command[joint] + gain * deadzone(delta, self.deadzones[index]) * dt
        return target

    def absolute_joints(self):
        """Joints driven one-to-one; the display marks these differently."""
        return {joint for joint, mode in zip(self.joints, self.modes) if mode == ABSOLUTE}


def rate_limit(command, target, max_step):
    """Never let a single cycle move a joint further than the speed limit allows.

    This is what keeps an IMU glitch on an absolute channel from slamming a joint.
    """
    command = np.asarray(command, dtype=float)
    difference = np.asarray(target, dtype=float) - command
    return command + np.clip(difference, -max_step, max_step)


def button_axis(positive, negative):
    """Two buttons acting as one signed channel."""
    return float(bool(positive)) - float(bool(negative))

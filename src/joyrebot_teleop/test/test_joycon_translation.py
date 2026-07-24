import math

import numpy as np

from joyrebot_teleop.vendor.joyconrobotics.joyconrobotics import (
    controller_translation_vectors,
)


def as_array(vector):
    return np.asarray([vector[0], vector[1], vector[2]], dtype=float)


def test_planar_stick_axes_depend_only_on_yaw_and_have_no_z_component():
    yaw = 0.63
    first = controller_translation_vectors(
        roll=0.71, pitch=-0.48, yaw=yaw,
        planar_stick_translation=True)
    second = controller_translation_vectors(
        roll=-1.02, pitch=0.84, yaw=yaw,
        planar_stick_translation=True)

    expected_forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    expected_right = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    assert np.allclose(as_array(first[0]), expected_forward)
    assert np.allclose(as_array(first[1]), expected_right)
    assert np.allclose(as_array(second[0]), expected_forward)
    assert np.allclose(as_array(second[1]), expected_right)
    assert np.isclose(np.dot(as_array(first[0]), as_array(first[1])), 0.0)


def test_nonplanar_mode_retains_original_pitch_and_roll_z_components():
    roll, pitch, yaw = 0.71, -0.48, 0.63
    forward, right, _ = controller_translation_vectors(
        roll, pitch, yaw, planar_stick_translation=False)

    assert np.isclose(as_array(forward)[2], math.sin(pitch))
    assert np.isclose(as_array(right)[2], math.sin(-roll))

import numpy as np

from joyrebot_teleop.vendor.joyconrobotics.attitude import (
    MahonyAttitudeEstimator,
)
from joyrebot_teleop.vendor.joyconrobotics.wrappers import PythonicJoyCon


def acceleration_for_euler(roll, pitch):
    """Joy-Con acceleration corresponding to a static output orientation."""
    return np.array([
        np.sin(pitch),
        np.cos(pitch) * np.sin(roll),
        -np.cos(pitch) * np.cos(roll),
    ])


def test_acceleration_initializes_level_referenced_roll_and_pitch():
    estimator = MahonyAttitudeEstimator()
    expected = np.array([0.31, -0.17, 0.0])

    orientation = estimator.update(
        np.zeros(3), acceleration_for_euler(*expected[:2]))

    assert np.allclose(orientation, expected, atol=1e-10)


def test_quaternion_integration_produces_continuous_true_yaw():
    dt = 0.005
    estimator = MahonyAttitudeEstimator(sample_period=dt)
    estimator.update(np.zeros(3), [0.0, 0.0, -1.0])

    # Sensor Z is opposite the output convention. Integrate through more than
    # 90 degrees to prove yaw is an angle, not a bounded direction component.
    for _ in range(800):
        orientation = estimator.update(
            [0.0, 0.0, -0.5], [0.0, 0.0, -1.0])

    assert np.allclose(orientation[:2], 0.0, atol=1e-10)
    assert np.isclose(orientation[2], 2.0, atol=1e-10)


def test_large_linear_acceleration_is_not_mistaken_for_tilt():
    estimator = MahonyAttitudeEstimator(
        sample_period=0.005, accel_rejection=0.25)
    estimator.update(np.zeros(3), [0.0, 0.0, -1.0])

    for _ in range(200):
        orientation = estimator.update(
            np.zeros(3), [1.0, 0.0, -1.0])

    assert np.allclose(orientation, 0.0, atol=1e-10)


def test_reset_yaw_preserves_tilt_and_yaw_offset_is_thread_safe():
    estimator = MahonyAttitudeEstimator(sample_period=0.005)
    tilt = acceleration_for_euler(0.2, -0.1)
    estimator.update(np.zeros(3), tilt)
    for _ in range(100):
        estimator.update([0.0, 0.0, -0.4], tilt)

    before = estimator.get_euler()
    estimator.reset_yaw()
    after = estimator.get_euler()
    estimator.set_yaw_diff(0.15)

    assert np.allclose(after[:2], before[:2], atol=1e-10)
    assert np.isclose(after[2], 0.0, atol=1e-10)
    assert np.isclose(estimator.get_euler()[2], -0.15, atol=1e-10)


def test_gyro_radian_conversion_matches_degree_conversion():
    joycon = object.__new__(PythonicJoyCon)
    joycon._ime_yz_coeff = 1.0
    joycon.get_gyro_x = lambda _: 100.0
    joycon.get_gyro_y = lambda _: -200.0
    joycon.get_gyro_z = lambda _: 300.0

    degrees = np.asarray(joycon.gyro_in_deg)
    radians = np.asarray(joycon.gyro_in_rad)

    assert np.allclose(radians, np.deg2rad(degrees), rtol=1e-5)

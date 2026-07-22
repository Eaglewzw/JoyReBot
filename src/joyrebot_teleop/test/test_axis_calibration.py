import numpy as np
import pytest

from joyrebot_teleop.axis_calibration import AxisCalibration


def test_derives_map_and_sign_in_output_axis_order():
    calibration = AxisCalibration(0.1)
    calibration.add([0.0, -0.8, 0.02], 0)
    calibration.add([0.03, 0.01, 0.7], 1)
    calibration.add([0.9, 0.01, 0.0], 2)
    assert calibration.result() == ([1, 2, 0], [-1.0, 1.0, 1.0])


def test_rejects_small_or_reused_input_axis():
    calibration = AxisCalibration(0.1)
    with pytest.raises(ValueError, match="too small"):
        calibration.add([0.01, 0.02, 0.03], 0)
    calibration.add([0.8, 0.0, 0.0], 0)
    with pytest.raises(ValueError, match="already used"):
        calibration.add([-0.7, 0.1, 0.0], 1)


def test_rejects_result_until_all_axes_are_sampled():
    calibration = AxisCalibration(0.1)
    calibration.add(np.array([0.2, 0.0, 0.0]), 0)
    with pytest.raises(RuntimeError, match="three axis samples"):
        calibration.result()

import pytest

from joyrebot_joint_teleop.device_input import normalize_axis, report_is_ready


def test_normalize_axis_centres_and_saturates():
    assert normalize_axis(2048, 2048.0, 1400.0) == pytest.approx(0.0)
    assert normalize_axis(2048 + 700, 2048.0, 1400.0) == pytest.approx(0.5)
    assert normalize_axis(4095, 2048.0, 1400.0) == pytest.approx(1.0)
    assert normalize_axis(0, 2048.0, 1400.0) == pytest.approx(-1.0)


def test_zero_filled_report_is_rejected():
    # A zeroed buffer decodes as full negative stick deflection, so it must never
    # be treated as live input.
    assert not report_is_ready(bytes(49))
    assert not report_is_ready(b"")
    assert normalize_axis(0, 2048.0, 1400.0) == pytest.approx(-1.0)
    assert report_is_ready(b"\x30" + bytes(48))

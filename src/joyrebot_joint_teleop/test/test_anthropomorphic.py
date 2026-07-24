import numpy as np
import pytest

from joyrebot_joint_teleop.anthropomorphic import (ABSOLUTE, OFF, RATE, AnthropomorphicMap,
                                                   button_axis, deadzone, rate_limit)


# 通道顺序：roll、pitch、yaw、stick_vertical、stick_horizontal、buttons。
# roll->joint6(5)、pitch->joint4(3)、yaw->joint1(0)，
# stick_v->joint2(1)、stick_h->joint5(4)、buttons->joint3(2)
JOINTS = [5, 3, 0, 1, 4, 2]
MODES = [ABSOLUTE, ABSOLUTE, RATE, RATE, RATE, RATE]
SCALES = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
DEADZONES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ROLL, PITCH, YAW, STICK_V, STICK_H, BUTTONS = range(6)


def build(**overrides):
    kwargs = dict(joints=JOINTS, modes=MODES, scales=SCALES, signs=SIGNS,
                  deadzones=DEADZONES, joint_count=6)
    kwargs.update(overrides)
    return AnthropomorphicMap(**kwargs)


def channel(index, value):
    inputs = np.zeros(6)
    inputs[index] = value
    return inputs


def test_engage_produces_no_step():
    mapper = build()
    command = np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0])
    inputs = [0.2, -0.1, 0.4, 0.0, 0.0, 0.0]
    mapper.engage(inputs, command)
    # 手腕保持在锚定位置时，机械臂必须保持不动。
    assert np.allclose(mapper.target(inputs, command, 1 / 60), command)


def test_wrist_channels_drive_joint6_and_joint4():
    mapper = build()
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    target = mapper.target([0.5, -0.25, 0.0, 0.0, 0.0, 0.0], command, 1 / 60)
    assert target[5] == pytest.approx(0.5)    # roll  -> joint6 末端旋转
    assert target[3] == pytest.approx(-0.25)  # pitch -> joint4 腕俯仰


def test_button_channel_drives_joint3():
    mapper = build()
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    for _ in range(60):
        command = mapper.target(channel(BUTTONS, button_axis(True, False)), command, 1 / 60)
    assert command[2] == pytest.approx(1.0)   # buttons -> joint3 小臂
    assert np.count_nonzero(command) == 1


def test_horizontal_stick_drives_joint5():
    mapper = build()
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    for _ in range(60):
        command = mapper.target(channel(STICK_H, 0.5), command, 1 / 60)
    assert command[4] == pytest.approx(0.5)   # stick_horizontal -> joint5 腕滚转


def test_absolute_channel_is_anchored_not_accumulated():
    """手腕角度保持不变时，关节角度也必须保持不变。"""
    mapper = build()
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    inputs = channel(ROLL, 0.4)
    for _ in range(100):
        command = mapper.target(inputs, command, 1 / 60)
    assert command[5] == pytest.approx(0.4)


def test_absolute_scale_amplifies_wrist_travel():
    mapper = build(scales=[2.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    assert mapper.target(channel(ROLL, 0.5), command, 1 / 60)[5] == pytest.approx(1.0)


def test_rate_channel_integrates_over_time():
    mapper = build()
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    inputs = channel(STICK_V, 0.5)            # stick_vertical 控制 joint2。
    for _ in range(60):
        command = mapper.target(inputs, command, 1 / 60)
    assert command[1] == pytest.approx(0.5, abs=1e-9)


def test_rate_deadzone_ignores_small_yaw_wobble():
    mapper = build(deadzones=[0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    for _ in range(120):
        command = mapper.target(channel(YAW, 0.15), command, 1 / 60)
    assert command[0] == 0.0


def test_sign_flips_channel_direction():
    mapper = build(signs=[-1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    assert mapper.target(channel(ROLL, 0.5), command, 1 / 60)[5] == pytest.approx(-0.5)


def test_off_channel_leaves_its_joint_alone():
    mapper = build(modes=[OFF] + MODES[1:])
    command = np.zeros(6)
    mapper.engage(np.zeros(6), command)
    assert mapper.target(channel(ROLL, 5.0), command, 1 / 60)[5] == 0.0


def test_every_joint_has_exactly_one_channel():
    assert sorted(build().joints) == list(range(6))


def test_absolute_joints_are_reported_for_the_display():
    assert build().absolute_joints() == {5, 3}


def test_reengage_after_clutch_does_not_jump():
    """冻结后将手腕移远再松开，机械臂必须保持当前位置。"""
    mapper = build()
    command = np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0])
    mapper.engage(np.zeros(6), command)
    command = mapper.target(channel(ROLL, 0.6), command, 1 / 60)
    parked = command.copy()
    # 按住离合后，操作者回正手腕，再松开离合。
    mapper.engage(np.zeros(6), command)
    assert np.allclose(mapper.target(np.zeros(6), command, 1 / 60), parked)


def test_rate_limit_caps_a_glitching_absolute_channel():
    stepped = rate_limit(np.zeros(6), np.array([9.0, -9.0, 0, 0, 0, 0]), 0.0117)
    assert stepped[0] == pytest.approx(0.0117)
    assert stepped[1] == pytest.approx(-0.0117)


def test_button_axis_is_signed():
    assert button_axis(True, False) == 1.0
    assert button_axis(False, True) == -1.0
    assert button_axis(True, True) == 0.0
    assert button_axis(False, False) == 0.0


def test_deadzone_is_continuous_at_the_edge():
    assert deadzone(0.2, 0.2) == 0.0
    assert deadzone(0.25, 0.2) == pytest.approx(0.05)
    assert deadzone(-0.25, 0.2) == pytest.approx(-0.05)


def test_target_before_engage_is_refused():
    with pytest.raises(RuntimeError):
        build().target(np.zeros(6), np.zeros(6), 1 / 60)


@pytest.mark.parametrize("overrides", [
    {"modes": [ABSOLUTE] * 5},                        # 长度错误
    {"modes": ["sideways"] + MODES[1:]},              # 未知模式
    {"joints": [9, 3, 0, 1, 4, 2]},                   # 超出范围
    {"joints": [5, 5, 0, 1, 4, 2]},                   # 两个通道控制同一关节
])
def test_invalid_configuration_is_refused(overrides):
    with pytest.raises(ValueError):
        build(**overrides)


def test_duplicate_joint_allowed_when_one_channel_is_off():
    build(joints=[5, 5, 0, 1, 4, 2], modes=[OFF] + MODES[1:])

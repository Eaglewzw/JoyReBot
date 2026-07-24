"""拟人化通道映射：Joy-Con 的每个自由度对应机械臂的一个关节。

Joy-Con 提供的独立通道数量恰好等于机械臂关节数：三个 IMU 旋转、两个摇杆轴和一对
按键。因此无需切换模式，六个关节即可像人手一样同时受控。

每个通道具有独立语义：

``absolute``
    关节从接合时捕获的锚点开始一对一跟随通道，适用于以重力为参考的 roll 和 pitch。
``rate``
    通道控制关节速度，适用于没有绝对参考、否则会自行漂移的 yaw，以及会自动回中的
    摇杆和按键通道。
``off``
    忽略该通道，关节保持当前位置。
"""

import numpy as np


ABSOLUTE = "absolute"
RATE = "rate"
OFF = "off"
MODES = (ABSOLUTE, RATE, OFF)

# 固定通道顺序；所有可配置数组均按此顺序逐项对应。
CHANNEL_NAMES = ("roll", "pitch", "yaw", "stick_vertical", "stick_horizontal", "buttons")


def deadzone(value, width):
    """抑制中位附近的噪声，并保持死区边界处的输出连续。"""
    magnitude = abs(float(value))
    if magnitude <= width:
        return 0.0
    return float(np.sign(value) * (magnitude - width))


class AnthropomorphicMap:
    """将六个 Joy-Con 通道绑定到六个关节，并计算一个控制周期的目标。"""

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
        """将绝对通道固定在操作者当前的手腕姿态。

        启动时及每次松开离合时调用，确保松开离合不会使机械臂发生跳变。
        """
        self.anchor_inputs = np.asarray(inputs, dtype=float).copy()
        self.anchor_command = np.asarray(command, dtype=float).copy()

    def clear(self):
        self.anchor_inputs = self.anchor_command = None

    def target(self, inputs, command, dt):
        """计算本周期的期望关节命令，尚未执行限速和限位裁剪。"""
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
                # 绝对通道始终以锚点为基准，关节命令不会在控制周期之间累计漂移。
                target[joint] = self.anchor_command[joint] + gain * delta
            else:
                target[joint] = command[joint] + gain * deadzone(delta, self.deadzones[index]) * dt
        return target

    def absolute_joints(self):
        """返回一对一控制的关节集合，供终端面板作区别标记。"""
        return {joint for joint, mode in zip(self.joints, self.modes) if mode == ABSOLUTE}


def rate_limit(command, target, max_step):
    """限制单周期关节变化量，使其不超过允许的速度上限。

    即使绝对通道的 IMU 出现异常跳变，也不会让关节在一个控制周期内猛烈运动。
    """
    command = np.asarray(command, dtype=float)
    difference = np.asarray(target, dtype=float) - command
    return command + np.clip(difference, -max_step, max_step)


def button_axis(positive, negative):
    """将一对按键组合成一个带正负方向的控制通道。"""
    return float(bool(positive)) - float(bool(negative))

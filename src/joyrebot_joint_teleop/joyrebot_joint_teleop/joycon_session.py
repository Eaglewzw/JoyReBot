"""与 ROS 无关的 Joy-Con 获取、输入校验、保活和重连管理。"""

from dataclasses import dataclass
import time

import numpy as np


# 将左右 Joy-Con 不同的底层方法名统一为稳定的语义按键和摇杆名称。
SIDE_BINDINGS = {
    "right": {
        "gripper": "get_button_zr", "shoulder": "get_button_r", "stick_press": "get_button_r_stick",
        "clutch": "get_button_b", "reanchor": "get_button_x", "home": "get_button_home",
        "stick": ("get_stick_right_horizontal", "get_stick_right_vertical"),
    },
    "left": {
        "gripper": "get_button_zl", "shoulder": "get_button_l", "stick_press": "get_button_l_stick",
        "clutch": "get_button_down", "reanchor": "get_button_up", "home": "get_button_capture",
        "stick": ("get_stick_left_horizontal", "get_stick_left_vertical"),
    },
}
BUTTON_KEYS = ("gripper", "shoulder", "stick_press", "clutch", "reanchor", "home")


def report_is_ready(report):
    """判断驱动是否已收到真实输入报文，避免把初始全零缓冲区用于控制。

    vendor 驱动启动时的缓冲区全为零；零摇杆计数会被解码为负向满偏，若直接控制会在
    启动初期让关节全速转动，因此首个真实报文到达前必须保持不动。
    """
    return bool(report) and bool(report[0])


def normalize_axis(raw, center, half_range):
    """将 Joy-Con 的 12 位摇杆原始计数归一化并裁剪到 [-1, 1]。"""
    return float(np.clip((float(raw) - center) / max(1.0, float(half_range)), -1.0, 1.0))


@dataclass(frozen=True)
class HandControllerSample:
    """一帧已标准化、与左右手柄无关的关节遥操输入。"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    stick_vertical: float = 0.0
    stick_horizontal: float = 0.0
    buttons: dict | None = None
    fresh: bool = False
    connected: bool = False
    side: str | None = None
    connection_generation: int = 0


class JoyconSession:
    """独占一个 Joy-Con，并输出经过校验和标准化的控制样本。"""

    def __init__(self, input_timeout, stick_center, stick_half_range,
                 stick_horizontal_sign, stick_vertical_sign, clock=time.monotonic,
                 controller_factory=None, info=lambda message: None,
                 warning=lambda message: None):
        self.input_timeout = float(input_timeout)
        self.stick_center = float(stick_center)
        self.stick_half_range = float(stick_half_range)
        self.stick_horizontal_sign = float(stick_horizontal_sign)
        self.stick_vertical_sign = float(stick_vertical_sign)
        self.clock = clock
        if controller_factory is None:
            from joyrebot_teleop.vendor.joyconrobotics import JoyconRobotics
            controller_factory = JoyconRobotics
        self.controller_factory = controller_factory
        self.info = info
        self.warning = warning
        self.controller = None
        self.side = None
        self.last_report = None
        self.last_input_time = None
        self.connection_generation = 0

    @property
    def connected(self):
        return self.controller is not None

    def _sample(self, **values):
        return HandControllerSample(
            connected=self.connected,
            side=self.side,
            connection_generation=self.connection_generation,
            **values,
        )

    def connect(self, announce_failure=False):
        """按右手柄优先、左手柄回退的顺序连接一个 Joy-Con。"""
        if self.connected:
            return True
        for side in ("right", "left"):
            try:
                self.controller = self.controller_factory(
                    side, without_rest_init=False, all_button_return=True,
                    gripper_open=1.0, gripper_close=0.0, enable_shoulder_translation=True)
                self.side = side
                self.last_report = None
                self.last_input_time = None
                self.connection_generation += 1
                self.info(f"{side.capitalize()} Joy-Con connected")
                return True
            except Exception as error:
                self.controller = None
                if announce_failure:
                    self.warning(f"{side.capitalize()} Joy-Con unavailable: {error}")
        if announce_failure:
            self.warning(
                "No Joy-Con connected. The node keeps running, holds the arm still and rescans.")
        return False

    def rescan(self):
        return self.connect()

    def _drop(self):
        if self.controller is not None:
            try:
                self.controller.running = False
                self.controller.disconnnect()  # vendor 中的接口名即为此拼写。
            except Exception:
                pass
        was_connected = self.connected
        self.controller = None
        self.side = None
        self.last_report = None
        self.last_input_time = None
        if was_connected:
            self.connection_generation += 1

    def poll(self):
        """读取当前手柄样本；未就绪或超时的报文不会产生可执行输入。"""
        if not self.connected:
            return self._sample()
        bindings = SIDE_BINDINGS[self.side]
        joycon = self.controller.joycon
        try:
            report = bytes(joycon._input_report)
            posture = self.controller.get_control()[0]
            raw_sticks = tuple(getattr(joycon, name)() for name in bindings["stick"])
            buttons = {key: bool(getattr(joycon, bindings[key])()) for key in BUTTON_KEYS}
        except Exception as error:
            self.warning(f"Joy-Con read failed ({error}); dropping the connection")
            self._drop()
            return self._sample()
        if not report_is_ready(report):
            return self._sample()
        now = self.clock()
        if report != self.last_report:
            self.last_report = report
            self.last_input_time = now
        if now - self.last_input_time > self.input_timeout:
            return self._sample(buttons=buttons)
        horizontal = normalize_axis(
            raw_sticks[0], self.stick_center, self.stick_half_range) * self.stick_horizontal_sign
        vertical = normalize_axis(
            raw_sticks[1], self.stick_center, self.stick_half_range) * self.stick_vertical_sign
        roll, pitch, yaw = posture[3:6]
        return self._sample(
            roll=float(roll), pitch=float(pitch), yaw=float(yaw),
            stick_vertical=float(vertical), stick_horizontal=float(horizontal),
            buttons=buttons, fresh=True)

    def battery_level(self):
        try:
            return int(self.controller.joycon.get_battery_level())
        except Exception:
            return None

    def close(self):
        """释放当前 Joy-Con；可重复调用。"""
        self._drop()

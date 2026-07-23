"""Anthropomorphic joint-space Joy-Con control.

All six joints move at once, the way a hand moves. The Joy-Con happens to offer
exactly six independent channels -- three IMU rotations, two stick axes and one
button pair -- so every joint gets its own, with no mode switching anywhere.

Wrist roll and pitch drive the arm's wrist one-to-one against gravity; yaw, the
sticks and the button pair command joint velocity.

Runs *instead of* joyrebot_teleop's joycon_input + teleop_controller pair, never
alongside it: the Joy-Con is an exclusively-owned HID device, and both nodes
publish the same /rebot/joint*/cmd_pos topics.
"""

import csv
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from joyrebot_teleop.kinematics import SerialChain
from joyrebot_teleop.vendor.joyconrobotics import JoyconRobotics

from .anthropomorphic import CHANNEL_NAMES, AnthropomorphicMap, button_axis, rate_limit
from .device_input import normalize_axis, report_is_ready
from .joint_display import print_dashboard


# Key layout keeps joyrebot_teleop's habits: the trigger is still the gripper and
# Home/Capture still returns to the startup pose. Plus/Minus is left alone because
# the vendored driver binds it to a two-second IMU recalibration.
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
EDGE_BUTTONS = ("gripper", "reanchor", "home")


class JointTeleop(Node):
    JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

    def __init__(self):
        super().__init__("joint_teleop")
        defaults = {
            "control_rate": 60.0, "input_timeout": 0.30,
            "display_rate": 10.0, "rescan_rate": 2.0, "terminal_display": True,
            "stick_center": 2048.0, "stick_half_range": 1400.0,
            # vertical = 摇杆前后 -> joint2; horizontal = 摇杆左右 -> joint5
            "stick_horizontal_sign": -1.0, "stick_vertical_sign": 1.0,
            # Channels, in the fixed order
            # (roll, pitch, yaw, stick_vertical, stick_horizontal, buttons),
            # each bound to one joint index:
            #   roll  -> joint6 末端旋转     pitch -> joint4 腕俯仰
            #   yaw   -> joint1 底座回转     摇杆垂直 -> joint2 大臂
            #   摇杆水平 -> joint5 腕滚转    R/摇杆按下 -> joint3 小臂
            "channel_joint": [5, 3, 0, 1, 4, 2],
            "channel_mode": ["absolute", "absolute", "rate", "rate", "rate", "rate"],
            # yaw (third entry) is deliberately the least sensitive channel: it has
            # no absolute reference and drifts, so low gain plus a wide dead zone.
            "channel_scale": [0.75, 0.5, 0.35, 0.25, 0.4, 0.25],
            "channel_sign": [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
            "channel_deadzone": [0.0, 0.0, 0.35, 0.15, 0.15, 0.0],
            "max_tracking_speed": 3.0,
            "joint_margin": 0.02, "max_joint_speed": 0.7,
            "home_joint_positions": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
            "move_home_on_startup": True,
            "gripper_open": 0.05, "gripper_closed": 0.0,
            "data_logging": True, "data_log_directory": "joint_teleop_logs",
            "data_log_flush_interval": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        urdf = Path(get_package_share_directory("joyrebot_teleop")) / "config/rebot_b601_kinematics.urdf"
        chain = SerialChain.from_urdf(urdf)
        margin = float(param("joint_margin"))
        self.lower = chain.lower + margin
        self.upper = chain.upper - margin
        joint_count = len(self.JOINT_NAMES)
        configured_home = np.asarray(param("home_joint_positions"), dtype=float)
        if configured_home.shape != (joint_count,):
            raise ValueError("home_joint_positions must contain exactly six values")
        if np.any(configured_home < self.lower) or np.any(configured_home > self.upper):
            raise ValueError("home_joint_positions must be inside the soft joint limits")
        self.configured_home = configured_home
        self.mapper = AnthropomorphicMap(param("channel_joint"), param("channel_mode"),
                                         param("channel_scale"), param("channel_sign"),
                                         param("channel_deadzone"), joint_count)

        self.joint_pubs = {
            name: self.create_publisher(Float64, f"/rebot/{name}/cmd_pos", 10)
            for name in self.JOINT_NAMES
        }
        self.gripper_pub = self.create_publisher(Float64, "/rebot/gripper/cmd_pos", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

        self.controller = None
        self.side = None
        self.q = self.command = self.home_command = None
        self.last_report = None
        self.last_input_time = None
        self.previous = {name: False for name in EDGE_BUTTONS}
        self.returning_home = False
        self.clutch = False
        self.gripper = 1.0
        self.gripper_command = float(param("gripper_open"))
        self.status = "waiting_for_joint_states"
        self.velocity = np.zeros(joint_count)
        self.inputs = np.zeros(len(CHANNEL_NAMES))

        rate = float(param("control_rate"))
        self.dt = 1.0 / rate
        self.log_file = None
        self.log_writer = None
        self.log_rows_since_flush = 0
        self.log_flush_every = max(1, int(rate * float(param("data_log_flush_interval"))))
        self._open_data_log()
        self.connect(announce_failure=True)
        self.create_timer(self.dt, self.control)
        self.create_timer(1.0 / max(0.2, float(param("rescan_rate"))), self.rescan)
        if bool(param("terminal_display")):
            self.create_timer(1.0 / max(0.2, float(param("display_rate"))), self.print_status)
        self.exclusivity_timer = self.create_timer(2.0, self._check_exclusive)
        self.get_logger().info(
            "Anthropomorphic joint control ready. "
            "Do not run joyrebot_teleop at the same time.")
        self._log_channel_map()

    def _log_channel_map(self):
        """Print the binding once at startup -- 'which stick axis moves which joint'
        is the single most common source of confusion when tuning directions."""
        labels = ("手柄roll", "手柄pitch", "手柄yaw", "摇杆前后", "摇杆左右", "R/摇杆按下")
        lines = [
            f"  {label} → {self.JOINT_NAMES[joint]}  ({mode}, scale={scale:g}, sign={sign:+g})"
            for label, joint, mode, scale, sign in zip(
                labels, self.mapper.joints, self.mapper.modes,
                self.mapper.scales, self.mapper.signs)
        ]
        self.get_logger().info("通道绑定:\n" + "\n".join(lines))

    # ── Joy-Con connection ────────────────────────────────────────────────────

    def connect(self, announce_failure=False):
        """Right Joy-Con first, left as fallback -- same priority as joyrebot_teleop."""
        if self.controller is not None:
            return True
        for side in ("right", "left"):
            try:
                # without_rest_init=False keeps the two-second rest calibration:
                # the wrist channels read the IMU, so the attitude estimate has to
                # settle before the first command.
                self.controller = JoyconRobotics(
                    side, without_rest_init=False, all_button_return=True,
                    gripper_open=1.0, gripper_close=0.0, enable_shoulder_translation=True)
                self.side = side
                self.last_report = None
                self.last_input_time = None
                self.previous = {name: False for name in EDGE_BUTTONS}
                self.mapper.clear()
                self.get_logger().info(f"{side.capitalize()} Joy-Con connected")
                return True
            except Exception as error:
                self.controller = None
                if announce_failure:
                    self.get_logger().warning(f"{side.capitalize()} Joy-Con unavailable: {error}")
        if announce_failure:
            self.get_logger().warning(
                "No Joy-Con connected. The node keeps running, holds the arm still and rescans.")
        return False

    def rescan(self):
        self.connect()

    def _drop(self):
        if self.controller is not None:
            try:
                self.controller.running = False
                self.controller.disconnnect()  # vendor spelling
            except Exception:
                pass
        self.controller = None
        self.side = None
        self.last_input_time = None
        self.mapper.clear()

    def _check_exclusive(self):
        """One-shot warning if joyrebot_teleop is driving the same command topics."""
        self.exclusivity_timer.cancel()
        others = self.count_publishers("/rebot/joint1/cmd_pos") - 1
        if others > 0:
            self.get_logger().error(
                f"{others} other publisher(s) on /rebot/joint1/cmd_pos -- joyrebot_teleop is "
                "probably running. Two controllers will fight over the arm; stop one of them.")

    # ── Input ─────────────────────────────────────────────────────────────────

    def _poll(self):
        """Returns (channel inputs, buttons, fresh). Inputs are zeroed unless fresh."""
        neutral = np.zeros(len(CHANNEL_NAMES))
        if self.controller is None:
            return neutral, {}, False
        bindings = SIDE_BINDINGS[self.side]
        joycon = self.controller.joycon
        try:
            # The vendored driver refreshes this buffer from its own daemon thread;
            # an unchanged report means the Joy-Con stopped talking to us.
            report = bytes(joycon._input_report)
            posture = self.controller.get_control()[0]
            raw_sticks = tuple(getattr(joycon, name)() for name in bindings["stick"])
            buttons = {key: bool(getattr(joycon, bindings[key])()) for key in BUTTON_KEYS}
        except Exception as error:
            self.get_logger().warning(f"Joy-Con read failed ({error}); dropping the connection")
            self._drop()
            return neutral, {}, False
        if not report_is_ready(report):
            return neutral, {}, False
        now = self.get_clock().now()
        if report != self.last_report:
            self.last_report = report
            self.last_input_time = now
        if ((now - self.last_input_time).nanoseconds * 1e-9
                > float(self.get_parameter("input_timeout").value)):
            return neutral, buttons, False
        self._handle_edges(buttons)
        center = float(self.get_parameter("stick_center").value)
        half_range = float(self.get_parameter("stick_half_range").value)
        signs = (float(self.get_parameter("stick_horizontal_sign").value),
                 float(self.get_parameter("stick_vertical_sign").value))
        horizontal, vertical = (normalize_axis(value, center, half_range) * sign
                                for value, sign in zip(raw_sticks, signs))
        roll, pitch, yaw = posture[3:6]
        # Channel order must match anthropomorphic.CHANNEL_NAMES.
        inputs = np.array([roll, pitch, yaw, vertical, horizontal,
                           button_axis(buttons["shoulder"], buttons["stick_press"])])
        return inputs, buttons, True

    def _handle_edges(self, buttons):
        """Rising-edge actions: gripper toggle, re-anchor, return home."""
        rising = {name: buttons.get(name, False) and not self.previous[name]
                  for name in EDGE_BUTTONS}
        self.previous = {name: buttons.get(name, False) for name in EDGE_BUTTONS}
        if rising["gripper"]:
            self.gripper = 0.0 if self.gripper > 0.5 else 1.0
        if rising["reanchor"]:
            self.mapper.clear()
            self.get_logger().info("Re-anchored to the current wrist pose")
        if rising["home"] and self.home_command is not None and not self.returning_home:
            self.returning_home = True
            self.mapper.clear()
            self.get_logger().info("Home requested; returning to the startup joint positions")

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in self.JOINT_NAMES):
            return
        self.q = np.asarray([positions[name] for name in self.JOINT_NAMES])
        if self.command is not None:
            return
        self.command = self.q.copy()
        self.home_command = self.configured_home.copy()
        outside = [name for name, value, lower, upper
                   in zip(self.JOINT_NAMES, self.q, self.lower, self.upper)
                   if value < lower or value > upper]
        if outside:
            self.get_logger().warning(
                f"Startup joints outside the soft limits: {', '.join(outside)}. "
                "They are clamped on the first command.")
        if (bool(self.get_parameter("move_home_on_startup").value)
                and not np.allclose(self.command, self.home_command, atol=1e-3)):
            self.returning_home = True
            self.get_logger().info("Joint feedback ready; moving to the Home pose first")
        else:
            self.get_logger().info("Joint feedback ready; control enabled")

    # ── Control ───────────────────────────────────────────────────────────────

    def control(self):
        if self.q is None or self.command is None:
            return
        previous_command = self.command.copy()
        inputs, buttons, fresh = self._poll()
        self.inputs = inputs
        self.clutch = bool(buttons.get("clutch")) and fresh
        if self.returning_home:
            self.status = "returning_home"
            max_step = float(self.get_parameter("max_joint_speed").value) * self.dt
            difference = self.home_command - self.command
            self.command = self.command + np.clip(difference, -max_step, max_step)
            if np.all(np.abs(difference) <= max_step):
                self.command = self.home_command.copy()
                self.returning_home = False
                self.get_logger().info("Home joint positions reached; control resumed")
        elif self.controller is None:
            self.status = "no_joycon"
        elif not fresh:
            self.status = "input_timeout"
        elif self.clutch:
            # Dropping the anchor here is what re-engages on release, so the
            # operator can re-centre their wrist without stepping the arm.
            self.mapper.clear()
            self.status = "clutch"
        else:
            if not self.mapper.engaged:
                self.mapper.engage(inputs, self.command)
            target = self.mapper.target(inputs, self.command, self.dt)
            tracking_step = float(self.get_parameter("max_tracking_speed").value) * self.dt
            self.command = np.clip(rate_limit(self.command, target, tracking_step),
                                   self.lower, self.upper)
            self.status = "tracking" if np.any(self.command != previous_command) else "hold"
        for name, value in zip(self.JOINT_NAMES, self.command):
            self.joint_pubs[name].publish(Float64(data=float(value)))
        closed = float(self.get_parameter("gripper_closed").value)
        opened = float(self.get_parameter("gripper_open").value)
        self.gripper_command = closed + self.gripper * (opened - closed)
        self.gripper_pub.publish(Float64(data=self.gripper_command))
        self.velocity = (self.command - previous_command) / self.dt
        self._write_data_log(self.command - previous_command)

    # ── Display and logging ───────────────────────────────────────────────────

    def print_status(self):
        if self.command is None:
            return
        print_dashboard({
            "side": "未连接" if self.side is None else f"{self.side.capitalize()} Joy-Con",
            "status": "clutch(冻结)" if self.clutch else self.status,
            "names": self.JOINT_NAMES,
            "command": self.command,
            "lower": self.lower, "upper": self.upper,
            "absolute": self.mapper.absolute_joints(),
            "inputs": self.inputs,
            "gripper": self.gripper,
            "gripper_command": self.gripper_command,
            "battery": self._battery_level(),
        })

    def _battery_level(self):
        try:
            return int(self.controller.joycon.get_battery_level())
        except Exception:
            return None

    def _open_data_log(self):
        if not bool(self.get_parameter("data_logging").value):
            return
        try:
            directory = Path(str(self.get_parameter("data_log_directory").value)).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"joint_teleop_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.log_file = path.open("w", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            columns = ["ros_time_s", "status", "clutch"]
            columns += [f"input_{name}" for name in CHANNEL_NAMES]
            for prefix in ("feedback", "velocity", "command", "command_delta"):
                columns += [f"{prefix}_joint{i}" for i in range(1, 7)]
            columns += ["gripper_normalized", "gripper_command"]
            self.log_writer.writerow(columns)
            self.log_file.flush()
            self.get_logger().info(f"Joint control data log: {path.resolve()}")
        except OSError as error:
            self.log_file = None
            self.log_writer = None
            self.get_logger().warning(f"Cannot create joint control data log: {error}")

    def _write_data_log(self, command_delta):
        if self.log_writer is None:
            return
        nan_joints = [float("nan")] * len(self.JOINT_NAMES)
        row = [self.get_clock().now().nanoseconds * 1e-9, self.status, int(self.clutch)]
        row += list(self.inputs)
        row += list(self.q if self.q is not None else nan_joints)
        row += list(self.velocity)
        row += list(self.command)
        row += list(command_delta)
        row += [self.gripper, self.gripper_command]
        self.log_writer.writerow(row)
        self.log_rows_since_flush += 1
        if self.log_rows_since_flush >= self.log_flush_every:
            self.log_file.flush()
            self.log_rows_since_flush = 0

    def destroy_node(self):
        self._drop()
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
            self.log_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

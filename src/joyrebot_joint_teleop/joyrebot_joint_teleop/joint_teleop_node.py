"""拟人化关节空间 Joy-Con 控制。

六个关节像人手一样同时运动。Joy-Con 恰好提供六个独立通道：三个 IMU 旋转、两个
摇杆轴和一对按键，因此每个关节都有独立输入，无需切换任何控制模式。

手腕 roll 和 pitch 以重力为参考一对一驱动机械臂腕部；yaw、摇杆和按键对则控制
关节速度。

本节点用于替代 `joyrebot_teleop` 的 `joycon_input + teleop_controller` 组合，不能与其
同时运行：Joy-Con HID 设备只能由一个节点独占，且二者会发布相同的
`/rebot/joint*/cmd_pos` 话题。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from joyrebot_teleop.kinematics import SerialChain

from .anthropomorphic import CHANNEL_NAMES, AnthropomorphicMap, button_axis, rate_limit
from .joint_data_logger import JointDataLogger
from .joint_display import print_dashboard
from .joycon_session import JoyconSession


# 保持 joyrebot_teleop 的按键习惯：扳机仍控制夹爪，Home/Capture 仍返回启动姿态。
# 不绑定 Plus/Minus，因为 vendor 驱动会将其用于约两秒的 IMU 重新标定。
EDGE_BUTTONS = ("gripper", "reanchor", "home")


class JointTeleop(Node):
    JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

    def __init__(self):
        super().__init__("joint_teleop")
        defaults = {
            "control_rate": 60.0, "input_timeout": 0.30,
            "display_rate": 10.0, "rescan_rate": 2.0, "terminal_display": True,
            "stick_center": 2048.0, "stick_half_range": 1400.0,
            # vertical = 摇杆前后 -> joint2；horizontal = 摇杆左右 -> joint5。
            "stick_horizontal_sign": -1.0, "stick_vertical_sign": 1.0,
            # 通道固定顺序为 (roll, pitch, yaw, stick_vertical, stick_horizontal, buttons)，
            # 每一项绑定一个关节下标：
            #   roll -> joint6 末端旋转；pitch -> joint4 腕俯仰。
            #   yaw -> joint1 底座回转；摇杆垂直 -> joint2 大臂。
            #   摇杆水平 -> joint5 腕滚转；R/摇杆按下 -> joint3 小臂。
            "channel_joint": [5, 3, 0, 1, 4, 2],
            "channel_mode": ["absolute", "absolute", "rate", "rate", "rate", "rate"],
            # yaw（第三项）刻意设置为最低灵敏度：它没有绝对参考且会漂移，
            # 因此使用低增益和较宽死区抑制误动作。
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

        self.session = JoyconSession(
            param("input_timeout"), param("stick_center"), param("stick_half_range"),
            param("stick_horizontal_sign"), param("stick_vertical_sign"),
            info=self.get_logger().info, warning=self.get_logger().warning)
        self.last_connection_generation = self.session.connection_generation
        self.q = self.command = self.home_command = None
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
        self.data_logger = JointDataLogger(
            param("data_logging"), param("data_log_directory"), rate,
            param("data_log_flush_interval"), CHANNEL_NAMES, self.JOINT_NAMES)
        self._open_data_log()
        self.session.connect(announce_failure=True)
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
        """启动时输出一次通道绑定，便于排查摇杆轴和关节方向配置。"""
        labels = ("手柄roll", "手柄pitch", "手柄yaw", "摇杆前后", "摇杆左右", "R/摇杆按下")
        lines = [
            f"  {label} → {self.JOINT_NAMES[joint]}  ({mode}, scale={scale:g}, sign={sign:+g})"
            for label, joint, mode, scale, sign in zip(
                labels, self.mapper.joints, self.mapper.modes,
                self.mapper.scales, self.mapper.signs)
        ]
        self.get_logger().info("通道绑定:\n" + "\n".join(lines))

    # ── Joy-Con 会话 ──────────────────────────────────────────────────────────

    def rescan(self):
        self.session.rescan()

    def _check_exclusive(self):
        """一次性检查 joyrebot_teleop 是否正在发布相同的关节命令话题。"""
        self.exclusivity_timer.cancel()
        others = self.count_publishers("/rebot/joint1/cmd_pos") - 1
        if others > 0:
            self.get_logger().error(
                f"{others} other publisher(s) on /rebot/joint1/cmd_pos -- joyrebot_teleop is "
                "probably running. Two controllers will fight over the arm; stop one of them.")

    # ── 输入 ─────────────────────────────────────────────────────────────────

    def _poll(self):
        """将一个标准化会话样本适配为映射器固定顺序的六通道输入。"""
        sample = self.session.poll()
        if sample.connection_generation != self.last_connection_generation:
            self.last_connection_generation = sample.connection_generation
            self.previous = {name: False for name in EDGE_BUTTONS}
            self.mapper.clear()
        neutral = np.zeros(len(CHANNEL_NAMES))
        if not sample.fresh:
            return neutral, sample.buttons or {}, False, sample
        self._handle_edges(sample.buttons)
        inputs = np.array([
            sample.roll, sample.pitch, sample.yaw,
            sample.stick_vertical, sample.stick_horizontal,
            button_axis(sample.buttons["shoulder"], sample.buttons["stick_press"]),
        ])
        return inputs, sample.buttons, True, sample

    def _handle_edges(self, buttons):
        """处理按键上升沿动作：切换夹爪、重新锚定和返回 Home。"""
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

    # ── 控制 ─────────────────────────────────────────────────────────────────

    def control(self):
        if self.q is None or self.command is None:
            return
        previous_command = self.command.copy()
        inputs, buttons, fresh, sample = self._poll()
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
        elif not sample.connected:
            self.status = "no_joycon"
        elif not fresh:
            self.status = "input_timeout"
        elif self.clutch:
            # 在此清除锚点，松开离合时会自动重新接合，操作者可回正手腕而不会带动机械臂跳变。
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
        self.data_logger.write(
            ros_time_s=self.get_clock().now().nanoseconds * 1e-9,
            status=self.status,
            clutch=self.clutch,
            inputs=self.inputs,
            feedback=self.q,
            velocity=self.velocity,
            command=self.command,
            command_delta=self.command - previous_command,
            gripper_normalized=self.gripper,
            gripper_command=self.gripper_command,
        )

    # ── 显示和日志 ────────────────────────────────────────────────────────────

    def print_status(self):
        if self.command is None:
            return
        print_dashboard({
            "side": "未连接" if self.session.side is None else f"{self.session.side.capitalize()} Joy-Con",
            "status": "clutch(冻结)" if self.clutch else self.status,
            "names": self.JOINT_NAMES,
            "command": self.command,
            "lower": self.lower, "upper": self.upper,
            "absolute": self.mapper.absolute_joints(),
            "inputs": self.inputs,
            "gripper": self.gripper,
            "gripper_command": self.gripper_command,
            "battery": self.session.battery_level(),
        })

    def _open_data_log(self):
        try:
            path = self.data_logger.open()
            if path is not None:
                self.get_logger().info(f"Joint control data log: {path.resolve()}")
        except OSError as error:
            self.get_logger().warning(f"Cannot create joint control data log: {error}")

    def destroy_node(self):
        self.session.close()
        self.data_logger.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

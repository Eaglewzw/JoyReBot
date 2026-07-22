"""ROS 2 interactive Joy-Con axis calibration assistant."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

from .axis_calibration import AXIS_NAMES, AxisCalibration, pose_components


STAGE_INSTRUCTIONS = (
    "演示你希望用于末端 +X 平移的摇杆方向（只推一个方向）",
    "演示你希望用于末端 +Y 平移的摇杆方向（只推一个方向）",
    "按住 R（左手柄为 L），演示末端 +Z 上升",
    "转动手柄，演示你希望对应末端绕 +X 的旋转",
    "转动手柄，演示你希望对应末端绕 +Y 的旋转",
    "转动手柄，演示你希望对应末端绕 +Z 的旋转",
)


class AxisCalibrationNode(Node):
    def __init__(self):
        super().__init__("axis_calibration")
        self.declare_parameter("minimum_position_motion", 0.01)
        self.declare_parameter("minimum_orientation_motion", 0.15)
        self.position_calibration = AxisCalibration(
            self.get_parameter("minimum_position_motion").value)
        self.orientation_calibration = AxisCalibration(
            self.get_parameter("minimum_orientation_motion").value)
        self.latest_pose = None
        self.anchor = None
        self.trigger_pressed = False
        self.stage = 0
        self.finished = False
        self.create_subscription(PoseStamped, "/joycon_input/pose", self.on_pose, 10)
        self.create_subscription(
            Bool, "/joycon_input/calibration_trigger", self.on_trigger, 10)
        self.get_logger().info(
            "坐标轴校准已就绪。校准过程中不要运行 teleop_controller。"
            "ZR/ZL 在此仅作为录制键。")
        self._print_instruction()

    def on_pose(self, message):
        self.latest_pose = pose_components(message)

    def on_trigger(self, message):
        pressed = bool(message.data)
        if self.finished or pressed == self.trigger_pressed:
            return
        if pressed:
            if self.latest_pose is None:
                self.get_logger().warning("尚未收到 Joy-Con 位姿，请稍后重试")
            else:
                position, rotation = self.latest_pose
                self.anchor = (position.copy(), rotation)
                self.get_logger().info(
                    "开始记录：执行提示动作，先松开动作按键/摇杆，最后松开 ZR/ZL")
        elif self.anchor is not None and self.latest_pose is not None:
            self._finish_trial()
        self.trigger_pressed = pressed

    def _finish_trial(self):
        start_position, start_rotation = self.anchor
        end_position, end_rotation = self.latest_pose
        if self.stage < 3:
            delta = end_position - start_position
            calibration = self.position_calibration
        else:
            # Same convention used by RelativePoseMapper.map().
            delta = Rotation.from_matrix(
                end_rotation.as_matrix() @ start_rotation.as_matrix().T).as_rotvec()
            calibration = self.orientation_calibration
        try:
            sample = calibration.add(delta, self.stage % 3)
        except ValueError as error:
            self.get_logger().warning(f"本次动作未通过：{error}，请重试")
            self.anchor = None
            self._print_instruction()
            return
        kind = "平移" if self.stage < 3 else "旋转"
        self.get_logger().info(
            f"已接受{kind}动作：识别为手柄 {AXIS_NAMES[sample.input_axis]} 轴，"
            f"符号 {sample.sign:+.0f}，幅度 {sample.magnitude:.3f}")
        self.stage += 1
        self.anchor = None
        if self.stage == 6:
            self._print_result()
            self.finished = True
        else:
            self._print_instruction()

    def _print_instruction(self):
        self.get_logger().info(
            f"步骤 {self.stage + 1}/6：按住 ZR（左手柄为 ZL），"
            f"{STAGE_INSTRUCTIONS[self.stage]}；完成后松开 ZR/ZL")

    def _print_result(self):
        position_map, position_sign = self.position_calibration.result()
        orientation_map, orientation_sign = self.orientation_calibration.result()
        self.get_logger().info(
            "校准完成。请将以下配置复制到 teleop.yaml 的 teleop_controller 节点：\n"
            f"position_axis_map: {position_map}\n"
            f"position_axis_sign: {position_sign}\n"
            f"orientation_axis_map: {orientation_map}\n"
            f"orientation_axis_sign: {orientation_sign}")


def main(args=None):
    rclpy.init(args=args)
    node = AxisCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

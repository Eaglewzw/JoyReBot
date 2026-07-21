import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float64


class JoyconInput(Node):
    def __init__(self):
        super().__init__("joycon_input")
        self.declare_parameter("device", "right")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("clutch_button", "sr")
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", 10)
        self.clutch_pub = self.create_publisher(Bool, "~/clutch", 10)
        self.gripper_pub = self.create_publisher(Float64, "~/gripper", 10)
        try:
            from .vendor.joyconrobotics import JoyconRobotics
            self.controller = JoyconRobotics(
                self.get_parameter("device").value,
                without_rest_init=False,
                all_button_return=True,
                gripper_open=1.0,
                gripper_close=0.0,
            )
        except Exception as error:
            raise RuntimeError(f"Could not initialize Joy-Con: {error}") from error
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.publish_input)
        self.get_logger().info("Joy-Con ready; hold the configured clutch button to move")

    def publish_input(self):
        posture, gripper, _ = self.controller.get_control()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "joycon"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = posture[:3]
        quaternion = Rotation.from_euler("xyz", posture[3:]).as_quat()
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        clutch_name = str(self.get_parameter("clutch_button").value)
        # listen_button consumes edge events; the tracked states are stable and suitable for a clutch.
        state_by_name = {
            # joycon-robotics 2025 names these two tracked fields in reverse.
            "sl": self.controller.joycon_button_sr,
            "sr": self.controller.joycon_button_sl,
            "zr": self.controller.joycon_button_zrl,
        }
        self.pose_pub.publish(pose)
        self.clutch_pub.publish(Bool(data=bool(state_by_name.get(clutch_name, 0))))
        self.gripper_pub.publish(Float64(data=float(gripper)))

    def destroy_node(self):
        if hasattr(self, "controller"):
            self.controller.running = False
            self.controller.disconnnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyconInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

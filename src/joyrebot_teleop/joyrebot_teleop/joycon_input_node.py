import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float64

if __package__:
    from .terminal_display import print_dashboard, snapshot
else:
    # Allow: python3 joyrebot_teleop/joyrebot_teleop/joycon_input_node.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from joyrebot_teleop.terminal_display import print_dashboard, snapshot


class JoyconInput(Node):
    def __init__(self):
        super().__init__("joycon_input")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("clutch_button", "sr")
        self.declare_parameter("terminal_display", True)
        self.declare_parameter("display_rate", 2.0)
        self.declare_parameter("rescan_rate", 2.0)
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", 10)
        self.clutch_pub = self.create_publisher(Bool, "~/clutch", 10)
        self.gripper_pub = self.create_publisher(Float64, "~/gripper", 10)
        if __package__:
            from .vendor.joyconrobotics import JoyconRobotics
        else:
            from joyrebot_teleop.vendor.joyconrobotics import JoyconRobotics
        self.controller_class = JoyconRobotics
        # Teleoperation priority is fixed: right Joy-Con first, left as fallback.
        self.monitors = {}
        self.controller = None
        for side in ("left", "right"):
            self.connect_side(side, announce_failure=True)
        if "right" in self.monitors:
            self.controller = self.monitors["right"]
        elif self.monitors:
            self.controller = self.monitors["left"]
            self.get_logger().warn(
                "Right Joy-Con is unavailable; using connected left Joy-Con for control")
        else:
            self.get_logger().warn(
                "No Joy-Con connected. The node will keep running and rescan both sides.")
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.publish_input)
        rescan_rate = max(0.2, float(self.get_parameter("rescan_rate").value))
        self.create_timer(1.0 / rescan_rate, self.rescan)
        if bool(self.get_parameter("terminal_display").value):
            display_rate = max(0.2, float(self.get_parameter("display_rate").value))
            self.create_timer(1.0 / display_rate, self.print_status)
        self.get_logger().info("Joy-Con monitor ready; hold the configured clutch button to move")

    def connect_side(self, side, announce_failure=False):
        if side in self.monitors:
            return True
        try:
            controller = self.controller_class(
                side,
                without_rest_init=False,
                all_button_return=True,
                gripper_open=1.0,
                gripper_close=0.0,
            )
            self.monitors[side] = controller
            self.get_logger().info(f"{side.capitalize()} Joy-Con connected")
            return True
        except Exception as error:
            if announce_failure:
                self.get_logger().warn(f"{side.capitalize()} Joy-Con unavailable: {error}")
            return False

    def rescan(self):
        for side in ("left", "right"):
            if side not in self.monitors:
                self.connect_side(side)
        selected = self.monitors.get("right") or self.monitors.get("left")
        if selected is not self.controller:
            selected_side = "right" if selected is self.monitors.get("right") else "left"
            self.controller = selected
            self.get_logger().info(f"Control switched to {selected_side} Joy-Con")

    def publish_input(self):
        if self.controller is None:
            return
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

    def print_status(self):
        data = {}
        for side, controller in self.monitors.items():
            posture = controller.get_control()[0]
            data[side] = snapshot(controller, posture)
        print_dashboard(left=data.get("left"), right=data.get("right"))

    def destroy_node(self):
        for controller in getattr(self, "monitors", {}).values():
            controller.running = False
            controller.disconnnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyconInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

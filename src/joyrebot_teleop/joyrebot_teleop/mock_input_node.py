import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class MockInput(Node):
    def __init__(self):
        super().__init__("mock_input")
        self.declare_parameter("amplitude", 0.03)
        self.declare_parameter("period", 8.0)
        self.pose_pub = self.create_publisher(PoseStamped, "/joycon_input/pose", 10)
        self.clutch_pub = self.create_publisher(Bool, "/joycon_input/clutch", 10)
        self.gripper_pub = self.create_publisher(Float64, "/joycon_input/gripper", 10)
        self.started = self.get_clock().now()
        self.create_timer(0.02, self.tick)

    def tick(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        angle = 2.0 * math.pi * elapsed / float(self.get_parameter("period").value)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "mock_controller"
        pose.pose.position.x = float(self.get_parameter("amplitude").value) * math.sin(angle)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.clutch_pub.publish(Bool(data=True))
        self.gripper_pub.publish(Float64(data=1.0))


def main(args=None):
    rclpy.init(args=args)
    node = MockInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

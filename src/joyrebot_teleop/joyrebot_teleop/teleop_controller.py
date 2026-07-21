from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from .kinematics import SerialChain
from .pose_mapping import RelativePoseMapper, pose_to_matrix


class TeleopController(Node):
    JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

    def __init__(self):
        super().__init__("teleop_controller")
        defaults = {
            "control_rate": 50.0, "input_timeout": 0.30,
            "position_scale": [1.0, 1.0, 1.0], "orientation_scale": [1.0, 1.0, 1.0],
            "position_axis_map": [0, 1, 2], "position_axis_sign": [1.0, 1.0, 1.0],
            "orientation_axis_map": [0, 1, 2], "orientation_axis_sign": [1.0, 1.0, 1.0],
            "workspace_min": [-0.55, -0.55, -0.05], "workspace_max": [0.55, 0.55, 0.70],
            "joint_margin": 0.02, "max_joint_speed": 0.7,
            "gripper_open": 0.05, "gripper_closed": 0.0,
            "ik_damping": 0.06, "ik_max_iterations": 120,
            "ik_position_tolerance": 0.004, "ik_orientation_tolerance": 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        urdf = Path(get_package_share_directory("joyrebot_gazebo_sim")) / "urdf/rebot_b601_rs.urdf"
        self.chain = SerialChain.from_urdf(urdf)
        margin = float(self.get_parameter("joint_margin").value)
        self.chain.lower += margin
        self.chain.upper -= margin
        param = lambda name: self.get_parameter(name).value
        self.mapper = RelativePoseMapper(param("position_scale"), param("orientation_scale"),
                                         param("position_axis_map"), param("position_axis_sign"),
                                         param("orientation_axis_map"), param("orientation_axis_sign"),
                                         param("workspace_min"), param("workspace_max"))
        self.joint_pubs = {
            name: self.create_publisher(Float64, f"/rebot/{name}/cmd_pos", 10)
            for name in self.JOINT_NAMES
        }
        self.gripper_pub = self.create_publisher(Float64, "/rebot/gripper/cmd_pos", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(PoseStamped, "/joycon_input/pose", self.on_pose, 10)
        self.create_subscription(Bool, "/joycon_input/clutch", self.on_clutch, 10)
        self.create_subscription(Float64, "/joycon_input/gripper", self.on_gripper, 10)
        self.q = self.command = None
        self.input_pose = None
        self.last_input_time = None
        self.clutch = False
        self.gripper = 1.0
        rate = float(param("control_rate"))
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.control)

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.JOINT_NAMES):
            self.q = np.asarray([positions[name] for name in self.JOINT_NAMES])
            if self.command is None:
                self.command = self.q.copy()
                self.get_logger().info("Joint feedback ready; teleoperation can be engaged")

    def on_pose(self, message):
        p, o = message.pose.position, message.pose.orientation
        self.input_pose = pose_to_matrix([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        self.last_input_time = self.get_clock().now()

    def on_clutch(self, message):
        requested = bool(message.data)
        if requested and not self.clutch and self.input_pose is not None and self.q is not None:
            self.mapper.engage(self.input_pose, self.chain.forward(self.q))
        elif not requested and self.clutch:
            self.mapper.clear()
        self.clutch = requested

    def on_gripper(self, message):
        self.gripper = float(np.clip(message.data, 0.0, 1.0))

    def control(self):
        if self.q is None or self.command is None:
            return
        fresh = self.last_input_time is not None and (
            self.get_clock().now() - self.last_input_time).nanoseconds * 1e-9 <= float(self.get_parameter("input_timeout").value)
        if self.clutch and fresh and self.mapper.input_anchor is not None:
            target = self.mapper.map(self.input_pose)
            solution, success = self.chain.inverse(
                target, self.command,
                damping=float(self.get_parameter("ik_damping").value),
                max_iterations=int(self.get_parameter("ik_max_iterations").value),
                position_tolerance=float(self.get_parameter("ik_position_tolerance").value),
                orientation_tolerance=float(self.get_parameter("ik_orientation_tolerance").value))
            if success and np.all(np.isfinite(solution)):
                max_step = float(self.get_parameter("max_joint_speed").value) * self.dt
                self.command += np.clip(solution - self.command, -max_step, max_step)
        for name, value in zip(self.JOINT_NAMES, self.command):
            self.joint_pubs[name].publish(Float64(data=float(value)))
        closed = float(self.get_parameter("gripper_closed").value)
        opened = float(self.get_parameter("gripper_open").value)
        self.gripper_pub.publish(Float64(data=closed + self.gripper * (opened - closed)))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

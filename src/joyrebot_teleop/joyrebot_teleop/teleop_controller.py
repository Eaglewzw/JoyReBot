import csv
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float64

from .kinematics import SerialChain
from .pose_mapping import RelativePoseMapper, pose_to_matrix


class TeleopController(Node):
    JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

    def __init__(self):
        super().__init__("teleop_controller")
        defaults = {
            "control_rate": 60.0, "input_timeout": 0.30,
            "position_scale": [1.0, 1.0, 1.0], "orientation_scale": [1.0, 1.0, 1.0],
            "position_axis_map": [0, 1, 2], "position_axis_sign": [1.0, 1.0, 1.0],
            "orientation_axis_map": [0, 1, 2], "orientation_axis_sign": [1.0, 1.0, 1.0],
            "orientation_limit": [2.09, 0.23, 0.86],
            "workspace_min": [-0.55, -0.55, -0.05], "workspace_max": [0.70, 0.55, 0.70],
            "joint_margin": 0.02, "max_joint_speed": 0.7,
            "home_joint_positions": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
            "move_home_on_startup": True,
            "gripper_open": 0.05, "gripper_closed": 0.0,
            "ik_solver": "dls",
            "ik_damping": 0.06, "ik_max_iterations": 120,
            "ik_position_tolerance": 0.004, "ik_orientation_tolerance": 0.015,
            "ik_position_weight": 100.0, "ik_orientation_weight": 0.35,
            "ik_manipulability_weight": 0.0, "ik_qp_substeps": 1,
            "data_logging": True, "data_log_directory": "teleop_logs",
            "data_log_flush_interval": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        urdf = Path(get_package_share_directory("joyrebot_teleop")) / "config/rebot_b601_kinematics.urdf"
        solver_kind = str(param("ik_solver")).strip().lower()
        rate = float(param("control_rate"))
        margin = float(param("joint_margin"))
        max_joint_speed = float(param("max_joint_speed"))
        if solver_kind == "placo":
            from .placo_solver import PlacoChain
            self.chain = PlacoChain(
                urdf,
                dt=1.0 / rate,
                position_weight=float(param("ik_position_weight")),
                orientation_weight=float(param("ik_orientation_weight")),
                manipulability_weight=float(param("ik_manipulability_weight")),
                joint_margin=margin,
                max_joint_speed=max_joint_speed,
                solve_iterations=int(param("ik_qp_substeps")))
            self.get_logger().info(
                "IK solver: PlaCo constrained incremental QP "
                f"({int(param('ik_qp_substeps'))} substep(s), "
                f"position/orientation weights "
                f"{float(param('ik_position_weight'))}:"
                f"{float(param('ik_orientation_weight'))})")
        elif solver_kind == "dls":
            self.chain = SerialChain.from_urdf(urdf)
            self.get_logger().info("IK solver: damped least squares (DLS)")
            self.chain.lower += margin
            self.chain.upper -= margin
        else:
            raise ValueError(
                f"Unsupported ik_solver {solver_kind!r}; expected 'dls' or 'placo'")
        configured_home = np.asarray(param("home_joint_positions"), dtype=float)
        if configured_home.shape != (6,):
            raise ValueError("home_joint_positions must contain exactly six values")
        if np.any(configured_home < self.chain.lower) or np.any(configured_home > self.chain.upper):
            raise ValueError("home_joint_positions must be inside the IK soft joint limits")
        self.configured_home = configured_home
        self.mapper = RelativePoseMapper(param("position_scale"), param("orientation_scale"),
                                         param("position_axis_map"), param("position_axis_sign"),
                                         param("orientation_axis_map"), param("orientation_axis_sign"),
                                         param("orientation_limit"),
                                         param("workspace_min"), param("workspace_max"))
        self.joint_pubs = {
            name: self.create_publisher(Float64, f"/rebot/{name}/cmd_pos", 10)
            for name in self.JOINT_NAMES
        }
        self.gripper_pub = self.create_publisher(Float64, "/rebot/gripper/cmd_pos", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(PoseStamped, "/joycon_input/pose", self.on_pose, 10)
        self.create_subscription(Float64, "/joycon_input/gripper", self.on_gripper, 10)
        self.create_subscription(Bool, "/joycon_input/reset", self.on_reset, 10)
        self.q = self.command = None
        self.home_command = None
        self.input_pose = None
        self.last_input_time = None
        self.reset_pressed = False
        self.returning_home = False
        self.gripper = 1.0
        self.dt = 1.0 / rate
        self.log_file = None
        self.log_writer = None
        self.log_rows_since_flush = 0
        self.log_flush_every = max(
            1, int(rate * float(param("data_log_flush_interval"))))
        self._open_data_log()
        self.create_timer(self.dt, self.control)

    @staticmethod
    def _pose_values(transform):
        if transform is None:
            return [float("nan")] * 6
        return [*transform[:3, 3],
                *Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")]

    def _open_data_log(self):
        if not bool(self.get_parameter("data_logging").value):
            return
        try:
            directory = Path(
                str(self.get_parameter("data_log_directory").value)).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = directory / f"teleop_{stamp}.csv"
            self.log_file = path.open("w", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            columns = [
                "ros_time_s", "status", "input_age_s", "ik_success",
                "ik_target_reached", "ik_position_error_m",
                "ik_orientation_error_rad", "ik_at_joint_limit",
                "ik_velocity_limited",
            ]
            columns += [f"input_{name}" for name in ("x", "y", "z", "roll", "pitch", "yaw")]
            columns += [f"target_{name}" for name in ("x", "y", "z", "roll", "pitch", "yaw")]
            for prefix in ("feedback", "ik", "command", "command_delta"):
                columns += [f"{prefix}_joint{i}" for i in range(1, 7)]
            columns += ["gripper_normalized", "gripper_command"]
            self.log_writer.writerow(columns)
            self.log_file.flush()
            self.get_logger().info(f"Teleoperation data log: {path.resolve()}")
        except OSError as error:
            self.log_file = None
            self.log_writer = None
            self.get_logger().warning(f"Cannot create teleoperation data log: {error}")

    def _write_data_log(self, status, input_age, ik_success, target,
                        solution, command_delta, gripper_command):
        if self.log_writer is None:
            return
        nan_joints = [float("nan")] * 6
        diagnostics = (getattr(self.chain, "last_diagnostics", None)
                       if ik_success is not None else None)
        row = [self.get_clock().now().nanoseconds * 1e-9, status, input_age,
               "" if ik_success is None else int(ik_success)]
        row += ([
            int(diagnostics.target_reached),
            diagnostics.position_error,
            diagnostics.orientation_error,
            int(diagnostics.at_joint_limit),
            int(diagnostics.velocity_limited),
        ] if diagnostics is not None else [
            "", float("nan"), float("nan"), "", "",
        ])
        row += self._pose_values(self.input_pose)
        row += self._pose_values(target)
        row += list(self.q if self.q is not None else nan_joints)
        row += list(solution if solution is not None else nan_joints)
        row += list(self.command if self.command is not None else nan_joints)
        row += list(command_delta)
        row += [self.gripper, gripper_command]
        self.log_writer.writerow(row)
        self.log_rows_since_flush += 1
        if self.log_rows_since_flush >= self.log_flush_every:
            self.log_file.flush()
            self.log_rows_since_flush = 0

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.JOINT_NAMES):
            self.q = np.asarray([positions[name] for name in self.JOINT_NAMES])
            if self.command is None:
                self.command = self.q.copy()
                self.home_command = self.configured_home.copy()
                outside = [
                    name for name, value, lower, upper in zip(
                        self.JOINT_NAMES, self.q, self.chain.lower, self.chain.upper)
                    if value < lower or value > upper
                ]
                if outside:
                    self.get_logger().warning(
                        "Startup joints outside IK soft limits: "
                        f"{', '.join(outside)}. Consider using a home pose farther "
                        "from the mechanical limits.")
                if (bool(self.get_parameter("move_home_on_startup").value)
                        and not np.allclose(self.command, self.home_command, atol=1e-3)):
                    self.returning_home = True
                    self.get_logger().info(
                        "Joint feedback ready; moving to the configured Home pose before teleoperation")
                else:
                    self._engage_mapping()
                    self.get_logger().info("Joint feedback ready; teleoperation enabled")

    def on_pose(self, message):
        p, o = message.pose.position, message.pose.orientation
        self.input_pose = pose_to_matrix([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        self.last_input_time = self.get_clock().now()
        if self.command is not None and self.mapper.input_anchor is None and not self.returning_home:
            self._engage_mapping()

    def _engage_mapping(self):
        if self.input_pose is not None and self.command is not None:
            self.mapper.engage(self.input_pose, self.chain.forward(self.command))

    def on_gripper(self, message):
        self.gripper = float(np.clip(message.data, 0.0, 1.0))

    def on_reset(self, message):
        pressed = bool(message.data)
        if pressed and not self.reset_pressed and self.home_command is not None:
            self.returning_home = True
            self.mapper.clear()
            self.get_logger().info(
                "Home requested; returning to the startup joint positions")
        self.reset_pressed = pressed

    def control(self):
        if self.q is None or self.command is None:
            return
        previous_command = self.command.copy()
        target = None
        solution = None
        ik_success = None
        status = "hold"
        max_step = float(self.get_parameter("max_joint_speed").value) * self.dt
        if self.returning_home:
            status = "returning_home"
            difference = self.home_command - self.command
            self.command += np.clip(difference, -max_step, max_step)
            if np.all(np.abs(difference) <= max_step):
                self.command = self.home_command.copy()
                self.returning_home = False
                self._engage_mapping()
                self.get_logger().info(
                    "Startup joint positions reached; teleoperation resumed")
        input_age = ((self.get_clock().now() - self.last_input_time).nanoseconds * 1e-9
                     if self.last_input_time is not None else float("nan"))
        fresh = (self.last_input_time is not None
                 and input_age <= float(self.get_parameter("input_timeout").value))
        if (not self.returning_home and fresh and self.mapper.input_anchor is not None):
            target = self.mapper.map(self.input_pose)
            solution, success = self.chain.inverse(
                target, self.command,
                damping=float(self.get_parameter("ik_damping").value),
                max_iterations=int(self.get_parameter("ik_max_iterations").value),
                position_tolerance=float(self.get_parameter("ik_position_tolerance").value),
                orientation_tolerance=float(self.get_parameter("ik_orientation_tolerance").value))
            ik_success = bool(success and np.all(np.isfinite(solution)))
            if ik_success:
                diagnostics = getattr(self.chain, "last_diagnostics", None)
                status = ("tracking_limited"
                          if diagnostics is not None
                          and diagnostics.at_joint_limit
                          and not diagnostics.target_reached
                          else "tracking")
                self.command += np.clip(solution - self.command, -max_step, max_step)
                self.command = np.clip(
                    self.command, self.chain.lower, self.chain.upper)
            else:
                status = "ik_failed"
                diagnostics = getattr(self.chain, "last_diagnostics", None)
                detail = (f" ({diagnostics.failure})"
                          if diagnostics is not None and diagnostics.failure
                          else "")
                self.get_logger().warning(
                    "IK produced no safe command; holding the last valid "
                    f"joint command{detail}",
                    throttle_duration_sec=2.0)
        for name, value in zip(self.JOINT_NAMES, self.command):
            self.joint_pubs[name].publish(Float64(data=float(value)))
        closed = float(self.get_parameter("gripper_closed").value)
        opened = float(self.get_parameter("gripper_open").value)
        gripper_command = closed + self.gripper * (opened - closed)
        self.gripper_pub.publish(Float64(data=gripper_command))
        self._write_data_log(
            status, input_age, ik_success, target, solution,
            self.command - previous_command, gripper_command)

    def destroy_node(self):
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
            self.log_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

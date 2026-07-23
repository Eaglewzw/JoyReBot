"""Joint-space jogging. Starts one node only.

This replaces joyrebot_teleop's teleop.launch.py -- do not run both, they own the
same Joy-Con and publish the same /rebot/joint*/cmd_pos topics.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("joyrebot_joint_teleop"))
                 / "config/joint_teleop.yaml")
    return LaunchDescription([
        Node(package="joyrebot_joint_teleop", executable="joint_teleop", name="joint_teleop",
             parameters=[config], output="screen"),
    ])

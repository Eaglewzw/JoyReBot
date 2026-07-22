from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("joyrebot_teleop")) / "config/teleop.yaml")
    return LaunchDescription([
        Node(
            package="joyrebot_teleop",
            executable="joycon_input",
            name="joycon_input",
            parameters=[config, {"terminal_display": False}],
            output="screen",
        ),
        Node(
            package="joyrebot_teleop",
            executable="axis_calibration",
            name="axis_calibration",
            output="screen",
        ),
    ])

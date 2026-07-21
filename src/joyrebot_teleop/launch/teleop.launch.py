from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("joyrebot_teleop")) / "config/teleop.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        Node(package="joyrebot_teleop", executable="joycon_input", name="joycon_input",
             parameters=[config],
             output="screen",
             condition=UnlessCondition(LaunchConfiguration("mock"))),
        Node(package="joyrebot_teleop", executable="mock_input", name="mock_input",
             output="screen", condition=IfCondition(LaunchConfiguration("mock"))),
        Node(package="joyrebot_teleop", executable="teleop_controller", name="teleop_controller",
             parameters=[config], output="screen"),
    ])

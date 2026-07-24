"""关节空间遥操启动文件，只启动一个节点。

本启动文件替代 `joyrebot_teleop` 的 `teleop.launch.py`。两者会独占同一个 Joy-Con，
并发布相同的 `/rebot/joint*/cmd_pos` 话题，因此不能同时运行。
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(get_package_share_directory("joyrebot_gazebo_sim"))
    ros_gz_dir = Path(get_package_share_directory("ros_gz_sim"))
    world = package_dir / "worlds" / "rebot_b601.sdf"
    urdf = package_dir / "urdf" / "rebot_b601_rs.urdf"
    bridge = package_dir / "config" / "bridge.yaml"
    robot_description = urdf.read_text(encoding="utf-8")

    gazebo_launch = PythonLaunchDescriptionSource(
        str(ros_gz_dir / "launch" / "gz_sim.launch.py")
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", str(package_dir.parent)
        ),
        IncludeLaunchDescription(
            gazebo_launch,
            launch_arguments={
                "gz_args": ["-r -v 3 ", str(world)],
                "gz_version": "8",
            }.items(),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        IncludeLaunchDescription(
            gazebo_launch,
            launch_arguments={
                "gz_args": ["-s -r -v 3 ", str(world)],
                "gz_version": "8",
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "rebot_b601_rs", "-file", str(urdf), "-z", "0.001"],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": str(bridge), "use_sim_time": True}],
            output="screen",
        ),
    ])

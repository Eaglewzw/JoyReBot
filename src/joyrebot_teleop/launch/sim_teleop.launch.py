from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = Path(get_package_share_directory("joyrebot_gazebo_sim")) / "launch/sim.launch.py"
    teleop = Path(get_package_share_directory("joyrebot_teleop")) / "launch/teleop.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("mock", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim)),
                                 launch_arguments={"gui": LaunchConfiguration("gui")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop)),
                                 launch_arguments={"mock": LaunchConfiguration("mock")}.items()),
    ])

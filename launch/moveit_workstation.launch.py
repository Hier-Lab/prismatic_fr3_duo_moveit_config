import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch MoveIt and RViz on a workstation connected to the real-time host."""
    package_share = get_package_share_directory("prismatic_fr3_duo_moveit_config")
    default_config = os.path.join(package_share, "config", "bringup.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "setup.launch.py")
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("config_file"),
                    "profile": "moveit_workstation",
                }.items(),
            ),
        ]
    )

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_rail_mounting_pose():
    """Return the world-to-rail XYZ/RPY values from the description package."""
    description_share = get_package_share_directory(
        "prismatic_fr3_duo_description"
    )
    kinematics_path = os.path.join(description_share, "urdf", "kinematics.yaml")
    with open(kinematics_path, encoding="utf-8") as kinematics_file:
        mounting = yaml.safe_load(kinematics_file)["rail_mounting_point"][
            "kinematic"
        ]

    # Keep the argument order expected by static_transform_publisher.
    return [
        str(mounting[key])
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    ]


def generate_launch_description():
    rail_mounting_pose = _load_rail_mounting_pose()

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher0",
                output="log",
                arguments=[
                    "--x",
                    rail_mounting_pose[0],
                    "--y",
                    rail_mounting_pose[1],
                    "--z",
                    rail_mounting_pose[2],
                    "--roll",
                    rail_mounting_pose[3],
                    "--pitch",
                    rail_mounting_pose[4],
                    "--yaw",
                    rail_mounting_pose[5],
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "rail_link",
                ],
            )
        ]
    )

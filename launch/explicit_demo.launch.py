import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


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
    """Launch the MoveIt demo with every process declared in this file."""
    moveit_config = (
        MoveItConfigsBuilder(
            "prismatic_fr3_duo",
            package_name="prismatic_fr3_duo_moveit_config",
        ).to_moveit_configs()
    )

    db = LaunchConfiguration("db")
    debug = LaunchConfiguration("debug")
    reset = LaunchConfiguration("reset")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_monitored_planning_scene = LaunchConfiguration(
        "publish_monitored_planning_scene"
    )
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    rail_mounting_pose = _load_rail_mounting_pose()

    launch_arguments = [
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        ),
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        ),
        DeclareBooleanLaunchArg("use_rviz", default_value=True),
        DeclareBooleanLaunchArg(
            "use_sim_time",
            default_value=False,
            description="Use simulation time published on the /clock topic",
        ),
        DeclareLaunchArgument("publish_frequency", default_value="60.0"),
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
        DeclareBooleanLaunchArg(
            "publish_monitored_planning_scene", default_value=True
        ),
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        ),
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities[
                "disable_capabilities"
            ],
        ),
        DeclareBooleanLaunchArg("monitor_dynamics", default_value=False),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        ),
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        ),
        DeclareBooleanLaunchArg("reset", default_value=False),
        DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"),
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost"),
    ]

    static_virtual_joint_tf = Node(
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
        parameters=[sim_time_parameter],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
            sim_time_parameter,
        ],
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration(
            "allow_trajectory_execution"
        ),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": publish_monitored_planning_scene,
        "publish_geometry_updates": publish_monitored_planning_scene,
        "publish_state_updates": publish_monitored_planning_scene,
        "publish_transforms_updates": publish_monitored_planning_scene,
        "monitor_dynamics": False,
        **sim_time_parameter,
    }
    move_group_parameters = [moveit_config.to_dict(), move_group_configuration]
    move_group_environment = {"DISPLAY": os.environ.get("DISPLAY", "")}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
        additional_env=move_group_environment,
        condition=UnlessCondition(debug),
    )
    move_group_debug = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
        additional_env=move_group_environment,
        prefix=[
            "gdb "
            f"-x {moveit_config.package_path / 'launch/gdb_settings.gdb'} "
            "--ex run --args"
        ],
        arguments=["--debug"],
        condition=IfCondition(debug),
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        sim_time_parameter,
    ]
    rviz = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=rviz_parameters,
                condition=UnlessCondition(debug),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                prefix=["gdb --ex run --args"],
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=rviz_parameters,
                condition=IfCondition(debug),
            ),
        ],
    )

    warehouse = GroupAction(
        condition=IfCondition(db),
        actions=[
            Node(
                package="warehouse_ros_mongo",
                executable="mongo_wrapper_ros.py",
                parameters=[
                    {
                        "overwrite": False,
                        "database_path": LaunchConfiguration(
                            "moveit_warehouse_database_path"
                        ),
                        "warehouse_port": LaunchConfiguration(
                            "moveit_warehouse_port"
                        ),
                        "warehouse_host": LaunchConfiguration(
                            "moveit_warehouse_host"
                        ),
                        "warehouse_exec": "mongod",
                        "warehouse_plugin": (
                            "warehouse_ros_mongo::MongoDatabaseConnection"
                        ),
                        **sim_time_parameter,
                    }
                ],
            ),
            Node(
                package="moveit_ros_warehouse",
                executable="moveit_init_demo_warehouse",
                output="screen",
                parameters=[sim_time_parameter],
                condition=IfCondition(reset),
            ),
        ],
    )

    # ROS2 Control Management Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            sim_time_parameter,
        ],
    )

    # Spawn the joint state broadcaster
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    #     parameters=[sim_time_parameter],
    # )

    # Spawn the controllers as defined in the ros2_controllers.yaml file
    # Only activate mutually compatible controllers; alternatives remain available via setup.launch.py.
    controller_names = [
        "joint_state_broadcaster",
        "dual_arm_controller",
        "base_controller",
        "left_hand_controller",
        "right_hand_controller",
    ]
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name],
            output="screen",
            parameters=[sim_time_parameter],
        )
        for controller_name in controller_names
    ]

    return LaunchDescription(
        launch_arguments
        + [
            static_virtual_joint_tf,
            robot_state_publisher,
            move_group,
            move_group_debug,
            rviz,
            # warehouse,
            ros2_control_node,
            *controller_spawners,
            # joint_state_broadcaster_spawner,
        ]
    )

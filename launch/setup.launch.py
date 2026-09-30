import os
import resource

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE_NAME = "prismatic_fr3_duo_moveit_config"


def _read_yaml(path):
    """Load the central bring-up file and reject an empty document."""
    with open(path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise RuntimeError(f"Bring-up configuration is empty or invalid: {path}")
    return config


def _load_rail_mounting_pose():
    """Return the world-to-rail transform maintained by the description package."""
    description_share = get_package_share_directory(
        "prismatic_fr3_duo_description"
    )
    kinematics_path = os.path.join(description_share, "urdf", "kinematics.yaml")
    with open(kinematics_path, encoding="utf-8") as kinematics_file:
        mounting = yaml.safe_load(kinematics_file)["rail_mounting_point"][
            "kinematic"
        ]
    return [str(mounting[key]) for key in ("x", "y", "z", "roll", "pitch", "yaw")]


def _robot_config(hardware_mode, config, real_moveit=False):
    """Build one robot description for simulation, hardware, or MoveIt-only use."""
    real = config["real_hardware"]
    prefixes = {
        "left": config["robots"]["left"]["prefix"],
        "right": config["robots"]["right"]["prefix"],
    }
    if prefixes != {"left": "left_", "right": "right_"}:
        raise RuntimeError(
            "Robot prefixes must remain left_ and right_ to match the combined URDF"
        )
    mappings = {
        "hardware_mode": hardware_mode,
        "left_robot_ip": config["robots"]["left"]["ip"],
        "right_robot_ip": config["robots"]["right"]["ip"],
        "left_arm_prefix": prefixes["left"],
        "right_arm_prefix": prefixes["right"],
        "is_async": str(real["hardware_is_async"]).lower(),
        "thread_priority": str(real["hardware_thread_priority"]),
    }
    builder = MoveItConfigsBuilder(
        "prismatic_fr3_duo", package_name=PACKAGE_NAME
    ).robot_description(mappings=mappings)
    if real_moveit:
        builder = builder.trajectory_execution(
            file_path="config/moveit_controllers_real.yaml"
        )
    return builder.to_moveit_configs()


def _static_rail_tf(use_sim_time):
    """Publish the SRDF virtual-joint transform from the description source."""
    pose = _load_rail_mounting_pose()
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_rail",
        output="log",
        arguments=[
            "--x", pose[0], "--y", pose[1], "--z", pose[2],
            "--roll", pose[3], "--pitch", pose[4], "--yaw", pose[5],
            "--frame-id", "world", "--child-frame-id", "rail_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def _controller_spawner(controller_names, inactive=False):
    """Load a configured controller set, activating it as one group when requested."""
    if not controller_names:
        return None
    arguments = list(controller_names)
    arguments.append("--inactive" if inactive else "--activate-as-group")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
    )


def _moveit_nodes(moveit_config, profile):
    """Create the planning and optional visualization processes for one host."""
    use_sim_time = bool(profile["use_sim_time"])
    monitored = bool(profile["publish_monitored_planning_scene"])
    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "use_sim_time": use_sim_time,
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": bool(profile["allow_trajectory_execution"]),
            "publish_planning_scene": monitored,
            "publish_geometry_updates": monitored,
            "publish_state_updates": monitored,
            "publish_transforms_updates": monitored,
        },
    ]

    # Select the debugger at configuration time so launch parameters stay in YAML.
    prefix = None
    arguments = []
    if profile.get("debug", False):
        prefix = [
            "gdb ",
            f"-x {moveit_config.package_path / 'launch/gdb_settings.gdb'} ",
            "--ex run --args",
        ]
        arguments = ["--debug"]
    nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_parameters,
            prefix=prefix,
            arguments=arguments,
        )
    ]
    if profile.get("use_rviz", True):
        rviz_prefix = ["gdb --ex run --args"] if profile.get("debug", False) else None
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                prefix=rviz_prefix,
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
            )
        )
    return nodes


def _simulation_nodes(config, package_share):
    """Run ros2_control, MoveIt, state publication, and RViz on one simulation host."""
    profile = config["simulation"]
    hardware_mode = profile["hardware_mode"]
    if hardware_mode not in ("fake", "isaac"):
        raise RuntimeError("simulation.hardware_mode must be 'fake' or 'isaac'")
    moveit_config = _robot_config(hardware_mode, config)
    use_sim_time = bool(profile["use_sim_time"])
    controller_parameters = {
        "use_sim_time": use_sim_time,
        "update_rate": int(profile["controller_manager_update_rate"]),
    }

    # Start state producers before planning consumers and controller spawners.
    nodes = [
        _static_rail_tf(use_sim_time),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"use_sim_time": use_sim_time, "publish_frequency": 60.0},
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                os.path.join(package_share, "config", "ros2_controllers.yaml"),
                controller_parameters,
            ],
            on_exit=Shutdown(),
        ),
    ]
    nodes.extend(_moveit_nodes(moveit_config, {**config["moveit"], **profile}))
    nodes.extend(
        node
        for node in (
            _controller_spawner(profile["active_controllers"]),
            _controller_spawner(profile["inactive_controllers"], inactive=True),
        )
        if node is not None
    )
    return nodes


def _validate_real_ips(config):
    """Stop before opening hardware when either configured robot address is a placeholder."""
    for side in ("left", "right"):
        robot_ip = str(config["robots"][side]["ip"]).strip()
        if not robot_ip or robot_ip.startswith("CHANGE_ME"):
            raise RuntimeError(
                f"Set robots.{side}.ip in config/bringup.yaml before real-hardware launch"
            )


def _realtime_cpu_affinity(profile):
    """Validate RT permissions and return the CPUs to pin the update thread to."""
    requested_priority = int(profile["controller_manager_thread_priority"])
    if profile.get("hardware_is_async", False):
        # The per-arm I/O threads talk to the robots, so they must outrank the update thread.
        hardware_priority = int(profile["hardware_thread_priority"])
        if hardware_priority <= requested_priority:
            raise RuntimeError(
                "real_hardware.hardware_thread_priority must be higher than "
                "controller_manager_thread_priority when hardware_is_async is true"
            )
        requested_priority = hardware_priority
    rt_priority_limit = resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
    if rt_priority_limit != resource.RLIM_INFINITY and rt_priority_limit < requested_priority:
        raise RuntimeError(
            "The real-hardware controller manager requests FIFO priority "
            f"{requested_priority}, but the process RLIMIT_RTPRIO is {rt_priority_limit}. "
            "Log in again after configuring the realtime group and verify `ulimit -r`."
        )

    realtime_flag = "/sys/kernel/realtime"
    if os.path.exists(realtime_flag):
        with open(realtime_flag, encoding="utf-8") as flag_file:
            realtime_kernel = flag_file.read().strip() == "1"
    else:
        kernel_identity = f"{os.uname().release} {os.uname().version}".upper()
        realtime_kernel = "PREEMPT_RT" in kernel_identity or "PREEMPT RT" in kernel_identity
    if not realtime_kernel:
        raise RuntimeError(
            "Real hardware requires a PREEMPT_RT kernel; /sys/kernel/realtime is not 1 "
            "and the running kernel does not report PREEMPT_RT."
        )

    cpu_affinity = profile.get("controller_manager_cpu_affinity", [])
    if not cpu_affinity:
        return []
    if not isinstance(cpu_affinity, list) or any(
        not isinstance(cpu, int) or isinstance(cpu, bool) for cpu in cpu_affinity
    ):
        raise RuntimeError(
            "real_hardware.controller_manager_cpu_affinity must be a list of CPU numbers"
        )
    cpu_count = os.cpu_count()
    if cpu_count is not None and any(cpu < 0 or cpu >= cpu_count for cpu in cpu_affinity):
        raise RuntimeError(
            "real_hardware.controller_manager_cpu_affinity contains a CPU outside the "
            f"available range 0..{cpu_count - 1}"
        )
    # Pinning is only meaningful once the listed CPUs are isolated from the scheduler.
    with open("/proc/cmdline", encoding="utf-8") as cmdline_file:
        if "isolcpus=" not in cmdline_file.read():
            print(
                "WARNING: real_hardware.controller_manager_cpu_affinity is set, but the "
                "kernel was booted without isolcpus. Pinning the update thread to a subset "
                "of shared CPUs usually increases jitter; consider leaving it empty."
            )
    return [int(cpu) for cpu in cpu_affinity]


def _gripper_node(side, robot_ip, publish_rate):
    """Start one physical Franka Hand server with prefixed joint-state names."""
    gripper_config = os.path.join(
        get_package_share_directory("franka_gripper"),
        "config",
        "franka_gripper_node.yaml",
    )
    return Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name=f"{side}_franka_gripper",
        output="screen",
        parameters=[
            gripper_config,
            {
                "robot_ip": robot_ip,
                "joint_names": [
                    f"{side}_fr3_finger_joint1",
                    f"{side}_fr3_finger_joint2",
                ],
                "state_publish_rate": publish_rate,
            },
        ],
    )


def _real_hardware_nodes(config, package_share):
    """Run only latency-sensitive hardware, controllers, grippers, and robot TF."""
    _validate_real_ips(config)
    profile = config["real_hardware"]
    if profile["hardware_mode"] != "real":
        raise RuntimeError("real_hardware.hardware_mode must be 'real'")
    cpu_affinity = _realtime_cpu_affinity(profile)
    moveit_config = _robot_config(profile["hardware_mode"], config)
    publish_rate = int(profile["joint_state_publish_rate"])
    ros2_joint_states = "ros2_control/joint_states"
    sources = [ros2_joint_states]
    start_grippers = bool(profile.get("start_grippers", True))
    if start_grippers:
        sources.extend(
            [
                "left_franka_gripper/joint_states",
                "right_franka_gripper/joint_states",
            ]
        )

    # ros2_control_node applies these to the update thread itself, unlike a taskset
    # prefix, which would also drag the DDS executor and service threads along.
    realtime_parameters = {
        "use_sim_time": False,
        "update_rate": int(profile["controller_manager_update_rate"]),
        "thread_priority": int(profile["controller_manager_thread_priority"]),
        # mlockall() keeps the 1 kHz loop from taking page faults.
        "lock_memory": bool(profile.get("controller_manager_lock_memory", True)),
    }
    # An empty list has no inferable parameter type, so only send it when set.
    if cpu_affinity:
        realtime_parameters["cpu_affinity"] = cpu_affinity

    # Without the gripper servers nothing publishes the four prismatic finger
    # joints, so let the URDF supply defaults for them. Source topics still win
    # for every joint they cover.
    joint_state_parameters = [] if start_grippers else [moveit_config.robot_description]
    joint_state_parameters.append(
        {
            "source_list": sources,
            "rate": publish_rate,
            "use_robot_description": not start_grippers,
            "use_sim_time": False,
        }
    )

    # Keep the real-time controller manager free of planning and visualization work.
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description, {"use_sim_time": False}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                os.path.join(package_share, "config", "ros2_controllers.yaml"),
                realtime_parameters,
            ],
            remappings=[("joint_states", ros2_joint_states)],
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=joint_state_parameters,
        ),
    ]
    if start_grippers:
        nodes.extend(
            [
                _gripper_node("left", config["robots"]["left"]["ip"], publish_rate),
                _gripper_node("right", config["robots"]["right"]["ip"], publish_rate),
            ]
        )
    nodes.extend(
        node
        for node in (
            _controller_spawner(profile["active_controllers"]),
            _controller_spawner(profile["inactive_controllers"], inactive=True),
        )
        if node is not None
    )
    return nodes


def _moveit_workstation_nodes(config):
    """Run MoveIt and RViz while consuming joint states and TF from the RT host."""
    profile = config["moveit"]
    if profile.get("use_sim_time", False):
        raise RuntimeError("moveit.use_sim_time must remain false for real hardware")
    # Fake ros2_control tags preserve the kinematic model without loading hardware plugins here.
    moveit_config = _robot_config("fake", config, real_moveit=True)
    nodes = [_static_rail_tf(False)]
    nodes.extend(_moveit_nodes(moveit_config, profile))
    return nodes


def _launch_setup(context):
    """Dispatch the selected YAML-backed deployment profile."""
    package_share = get_package_share_directory(PACKAGE_NAME)
    config_path = LaunchConfiguration("config_file").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    config = _read_yaml(config_path)
    if profile == "simulation":
        return _simulation_nodes(config, package_share)
    if profile == "real_hardware":
        return _real_hardware_nodes(config, package_share)
    if profile == "moveit_workstation":
        return _moveit_workstation_nodes(config)
    raise RuntimeError(f"Unknown bring-up profile: {profile}")


def generate_launch_description():
    """Expose only deployment selection and the central YAML file as launch arguments."""
    package_share = get_package_share_directory(PACKAGE_NAME)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(package_share, "config", "bringup.yaml"),
                description="YAML file containing all deployment parameters",
            ),
            DeclareLaunchArgument("profile", default_value="simulation"),
            OpaqueFunction(function=_launch_setup),
        ]
    )

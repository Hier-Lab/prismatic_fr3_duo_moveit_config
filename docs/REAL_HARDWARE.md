# Dual FR3 real-hardware bring-up

The real deployment is intentionally split across two computers. The real-time
computer owns both Franka connections, `ros2_control`, the physical gripper
servers, joint-state aggregation, and `robot_state_publisher`. The workstation
runs MoveIt, RViz, and the fixed `world` to `rail_link` transform.

## Prerequisites

1. Build the Humble branch of `franka_ros2` on the real-time computer. Source
   its install space before the dual-arm workspace. Both `franka_hardware` and
   `franka_gripper` must be discoverable with `ros2 pkg prefix`.
2. Install and configure a PREEMPT_RT kernel. Give the controller user
   permission for FIFO scheduling (`rtprio` up to 99 and unlimited `memlock`).
   Confirm the launch no longer prints `Could not enable FIFO RT scheduling`.
3. Put each robot on a suitable wired interface, unlock both brakes, enable
   FCI in Desk, and verify that both robot IP addresses are reachable from the
   real-time computer. Use independent interfaces/subnets when the robots have
   overlapping factory network settings.
4. Use the same `ROS_DOMAIN_ID` and compatible DDS/RMW configuration on both
   computers. Synchronize their system clocks with NTP or PTP. Real hardware
   never uses `/clock` or `use_sim_time`.

This machine currently has the `franka_ros2` source at
`/home/hier-tony/Projects/franka_ros2`, but it is not built. Its current
`rosdep check` also reports missing Franka/libfranka and lint dependencies, so
the physical-hardware path cannot be tested until those dependencies are
installed.

## Configure and build

Edit `config/bringup.yaml` and replace both `CHANGE_ME_*` values with the two
robot IP addresses. Keep the prefixes `left_` and `right_`; they match the
joint names in the combined robot description. Controller rates, RT
priorities, active/inactive controller sets, gripper startup, RViz, debug, and
simulation mode are all maintained in this one file.

Then build and source each workspace:

```bash
source /opt/ros/humble/setup.bash
cd /home/hier-tony/Projects/franka_ros2
colcon build --symlink-install --packages-select-up-to \
  franka_hardware franka_gripper franka_robot_state_broadcaster
source install/setup.bash
cd /home/hier-tony/Projects/dual_arm_ws
colcon build --symlink-install --packages-select prismatic_fr3_duo_moveit_config
source install/setup.bash
```

## Start the two-host system

On the real-time computer, after sourcing both workspaces:

```bash
ros2 launch prismatic_fr3_duo_moveit_config real_hardware.launch.py
```

On the workstation, after sourcing the dual-arm workspace:

```bash
ros2 launch prismatic_fr3_duo_moveit_config moveit_workstation.launch.py
```

Before commanding motion, verify the state and action endpoints:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 action list | grep -E 'follow_joint_trajectory|gripper_action'
```

Per-arm diagnostics are published below
`/left_franka_robot_state_broadcaster` and
`/right_franka_robot_state_broadcaster`.

The default arm controller is `dual_arm_controller`. The individual arm,
Cartesian, and whole-body controllers are loaded and configured but inactive.
Switch mutually exclusive controllers in one strict request. For example:

```bash
ros2 control switch_controllers --strict \
  --deactivate dual_arm_controller \
  --activate left_arm_controller right_arm_controller

ros2 control switch_controllers --strict \
  --deactivate left_arm_controller right_arm_controller base_controller \
  --activate whole_body_controller
```

Use the inverse switch before sending a trajectory through another controller.
Never activate overlapping effort controllers at the same time. The physical
base is represented by mock hardware for now and `base_controller` is inactive
in the real profile; do not treat it as a physical rail command.

The real hands use the Franka-supported action servers, not ros2_control hand
controllers:

- `/left_franka_gripper/gripper_action`
- `/right_franka_gripper/gripper_action`

MoveIt maps both endpoints as `GripperCommand` controllers. The simulated hands
continue to use `FollowJointTrajectory` because the Isaac bridge publishes one
master joint per mimic-finger pair.

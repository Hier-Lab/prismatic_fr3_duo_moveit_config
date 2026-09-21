# Prismatic Dual FR3 MoveIt Configuration

MoveIt 2 and ros2_control configuration for two Franka FR3 arms mounted on a
shared prismatic rail. The package supports:

- fake ros2_control hardware;
- Isaac Sim through `topic_based_ros2_control`;
- two physical FR3 robots, with controllers on a real-time computer and
  MoveIt/RViz on a workstation.

Runtime settings, including robot IPs, hardware mode, controller sets, and
simulation time, are in [`config/bringup.yaml`](config/bringup.yaml).

## Build

```bash
cd ~/Projects/dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select prismatic_fr3_duo_moveit_config
source install/setup.bash
```

Rebuild and source the workspace after changing installed configuration files.

## Fake hardware

Set the following values in `config/bringup.yaml`:

```yaml
simulation:
  hardware_mode: "fake"
  use_sim_time: false
```

Then launch the complete stack:

```bash
ros2 launch prismatic_fr3_duo_moveit_config simulation.launch.py
```

## Isaac Sim

Set:

```yaml
simulation:
  hardware_mode: "isaac"
  use_sim_time: true
```

In Isaac Sim, configure the ROS graph to:

- publish `/clock` and `/isaac_joint_states`;
- subscribe to `/isaac_joint_commands`;
- exchange all 17 controlled joints: the rail, two gripper master joints, and
  fourteen arm joints.

Start the Isaac timeline, then launch:

```bash
ros2 launch prismatic_fr3_duo_moveit_config simulation.launch.py
```

## Real hardware

Install and build `franka_ros2` on the real-time computer first. In
`config/bringup.yaml`, enter both robot IPs and keep real time disabled:

```yaml
robots:
  left:
    ip: "LEFT_ROBOT_IP"
  right:
    ip: "RIGHT_ROBOT_IP"

real_hardware:
  hardware_mode: "real"
  use_sim_time: false

moveit:
  use_sim_time: false
```

Use the same `ROS_DOMAIN_ID` and DDS configuration on both computers.

On the real-time computer, source `franka_ros2` before this workspace and run:

```bash
source ~/Projects/franka_ros2/install/setup.bash
source ~/Projects/dual_arm_ws/install/setup.bash
ros2 launch prismatic_fr3_duo_moveit_config real_hardware.launch.py
```

On the MoveIt workstation, run:

```bash
source ~/Projects/dual_arm_ws/install/setup.bash
ros2 launch prismatic_fr3_duo_moveit_config moveit_workstation.launch.py
```

The physical rail currently uses mock hardware and its controller is inactive.
See [`REAL_HARDWARE.md`](docs/REAL_HARDWARE.md) for real-time setup, validation, and
controller-switching details.

## Quick checks

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 action list
```

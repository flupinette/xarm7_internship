# xarm_moveit_servo Directory

This directory contains a modified version of the `xarm_moveit_servo` package from [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2), enhanced to:
- Enable control of the xArm7 linear rail and gripper through `moveit_servo` using keyboard inputs
- Add support for Orbbec Astra camera integration with new launch options

**Note:** For detailed information about the camera integration (`add_orbbec_astra` and `add_orbbec_astra_links` options), please refer to the `xarm_description` directory in this repository.

The robot can be used in simulation mode, but you won't be able to move the gripper or the rail.

## Modified Files

### Core Control Modifications
- `src/xarm_ros2/xarm_moveit_servo/include/xarm_moveit_servo/xarm_keyboard_input.h`
- `src/xarm_ros2/xarm_moveit_servo/src/xarm_keyboard_input.cpp`
- `src/xarm_ros2/xarm_moveit_servo/CMakeLists.txt`

### Camera Integration Modifications
- `src/xarm_ros2/xarm_moveit_servo/launch/_robot_moveit_servo_fake.launch.py`
- `src/xarm_ros2/xarm_moveit_servo/launch/_robot_moveit_servo_realmove.launch.py`
- `src/xarm_ros2/xarm_moveit_servo/launch/_robot_moveit_servo.launch.py`

## Installation

To use this package, replace the original `xarm_moveit_servo` directory from [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) with this modified version.

**Important:** The xArm repository may have been updated since these modifications were made. Please verify the current structure and adapt the changes accordingly before use.

## Usage

### Basic Operation

In the first terminal, launch the moveit_servo node:

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.240 dof:=7 add_gripper:=True
```

In the second terminal, run the keyboard input interface:

```bash
ros2 run xarm_moveit_servo xarm_keyboard_input
```

## Motion Recording and Replay

Currently, the linear rail and gripper are controlled via services, not topics.
In ROS 2 Humble, ros2 bag only records topics, so rail and gripper motions cannot be replayed.

In ROS 2 Jazzy, ros2 bag supports recording services, which, in theory, should allow full replay (arm + rail + gripper) but, in practice, it doesn't seem to work.
The changes made in this repository mirror what would be needed for Jazzy, but since the official Jazzy branch differs from Humble, do not blindly merge this package without reviewing the differences.

For partial recording (arm movements only):
```bash
ros2 bag record -o demo   /xarm7_traj_controller/joint_trajectory   /xarm/joint_states   /xarm/robot_states   /tf   /tf_static
```
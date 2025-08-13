# xarm_description Directory

This directory contains a modified version of the `xarm_description` package from [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2), enhanced to add Orbbec Astra camera integration support. The camera mesh (`astra_with_stand.stl`) originates from the [gazebo_sensor_collection](https://github.com/Hlwy/gazebo_sensor_collection) repository.

## Modified Files and Added Files

### Added Files
- `src/xarm_ros2/xarm_description/meshes/camera/orbbec`
- `src/xarm_ros2/xarm_description/urdf/camera/orbbec_astra.urdf.xacro`
- `src/xarm_ros2/xarm_description/urdf/camera/orbbec.gazebo.xacro`

### Modified Files
- `src/xarm_ros2/xarm_description/urdf/xarm_device_macro.xacro`
- `src/xarm_ros2/xarm_description/urdf/xarm_device.urdf.xacro`
- `src/xarm_ros2/xarm_description/launch/_robot_description.launch.py`
- `src/xarm_ros2/xarm_description/launch/_robot_joint_state.launch.py`
- `src/xarm_ros2/xarm_description/launch/_robot_rviz_display.launch.py`

## Installation

To use this package, replace the original `xarm_description` directory from [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) with this modified version.

Throughout the rest of the [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) package:
  - Replace all instances of `add_realsense_d435i` parameter with `add_orbbec_astra`
  - Replace all instances of `add_d435i_links` parameter with `add_orbbec_astra_links`
  - Alternatively, to support both cameras, duplicate the relevant lines and keep both parameter names

**Important:** The xArm repository may have been updated since these modifications were made. Please verify the current structure and adapt the changes accordingly before use.

## Usage

To enable the Orbbec Astra camera in your launches, add the `add_orbbec_astra:=true` parameter to your launch commands.

Exemples :
```bash
ros2 launch xarm_description xarm7_rviz_display.launch.py add_gripper:=true add_orbbec_astra:=true
```
```bash
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py robot_ip:=192.168.1.240 add_gripper:=true add_orbbec_astra:=true
```

## Camera Mesh Notes

The current `astra_with_stand.stl` mesh still contains the D435i camera model, but this doesn't affect functionality. The D435i mesh can be safely removed if not needed. The mesh should be adjusted to match your physical setup.

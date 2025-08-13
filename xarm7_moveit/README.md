# Programs to Control the xArm7 Robot

This folder contains all programs developed to control the xArm7 robotic arm using MoveIt2. Note that in MoveIt2, the linear rail cannot be simulated and will not appear in the RViz2 window. To control the rail, the script `xarm7_linear_motor.py` was created to directly use the robot's services.

## Attention
Before executing any program on the real robot, ensure that:
1. The environmental conditions (such as the position of the table and the rail) match the actual setup.
2. All programs are first tested in simulation mode to avoid potential issues.

## Usage of the Programs in the xarm7_moveit Folder

All programs can be used in simulation mode and with the real robot. However, if you are using a program that involves a real camera, launch and use it as if it were on the simulated robot to ensure proper functionality (e.g., if using `xarm7_cam_pick_pour`, place a bowl under the camera).

### In Simulation
To launch MoveIt2 + RViz2 in simulation mode, use the following command:

```bash
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py [add_gripper:=true] [add_orbbec_astra:=true]
```

### With the Real Robot
To launch MoveIt2 + RViz2 with the real robot, use the following command:

```bash
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py robot_ip:=192.168.1.240 [add_gripper:=true] [add_orbbec_astra:=true]
```


### Running the Programs in the xarm7_moveit Folder
For most programs, simply run:

```bash
ros2 run xarm7_moveit <PROGRAM_NAME>
```

The code will execute immediately. If the rail does not move, it might be because it is not enabled. Try executing the following commands to enable and reset the rail:

```bash
ros2 service call /xarm/set_linear_track_enable xarm_msgs/srv/SetInt16 "{data: 1}"
ros2 service call /xarm/set_linear_track_back_origin xarm_msgs/srv/LinearTrackBackOrigin "{}"
```

Then, try running the program again.

If the program uses the Astra Pro Camera, launch the camera first with the following command:

```bash
ros2 launch astra_camera astra_pro.launch.xml
```

**Note:** Always read the top comment in each program file, as it may contain specific instructions or prerequisites.

## Octomap

To use Octomap for planning the robot's movements, install Octomap with:

```bash
sudo apt-get install liboctomap-dev octomap-tools
sudo apt-get install ros-humble-octomap ros-humble-octomap-mapping ros-humble-octomap-msgs ros-humble-octomap-ros ros-humble-octomap-server
```

Add the following line to your `src/xarm_ros2/xarm_moveit_config/package.xml` file:
```xml
<depend>octomap</depend>
```

Then, modify the `src/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_common2.launch.py` file by replacing:

```Python
    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )
```
 
with:

 ```Python
    # Adapt those parameters depending on your situation
    sensor_manager_parameters = {
        'sensors': ['ros'],
        'octomap_resolution': 0.02,
        'ros.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
        'ros.point_cloud_topic': '/camera/depth/points',
        'ros.max_range': 4.0,
        'ros.point_subsample': 1,
        'ros.padding_offset': 0.1,
        'ros.padding_scale': 1.0,
        'ros.max_update_rate': 1.0,
        'ros.filtered_cloud_topic': 'filtered_cloud',
     }
    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            sensor_manager_parameters,
            {'use_sim_time': use_sim_time},
            {'octomap_frame': 'world', 'octomap_resolution': 0.02, 'max_range': 4.0}, # Adapt those parameters depending on your situation
        ],
    )
 ```

## Videos
For many programs, videos demonstrating their real-life or simulated applications are provided in the `videos` folder. To check if a program has an associated video, refer to the top comment in the program file.
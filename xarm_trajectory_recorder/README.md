# Recording and Re-playing a Trajectory in Manual Mode in ROS2 with the xarm_trajectory_recoder Folder

This package allows you to convert a ROS bag file that has recorded the robot's positions on the topic `/xarm/joint_states` into commands to replay the recorded trajectory using the command `ros2 bag record [...]`.

This package was developed to replay a trajectory performed in manual mode. It has been tested only in this scenario but should work under other circumstances as well.

## Adapting the Code to Your Situation

Change the `bag_path` variable in the `trajectory_recorder.py` file to match your specific setup.

## Usage

First, launch the following command to record the trajectory:

```bash
ros2 bag record -o freedrive_record /xarm/joint_states
```

Perform the desired movement with the robot. Once finished, stop the recording by pressing CTRL+C.

To communicate with the robot, launch the following command:

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.240 dof:=7 add_gripper:=True
```

To convert the ROS bag file into a usable YAML file, run:

```bash
ros2 run xarm7_moveit trajectory_recorder
```

Finally, to replay the trajectory, launch:

```bash
ros2 run xarm7_moveit trajectory_planner
```

Once finished, press CTRL+C.
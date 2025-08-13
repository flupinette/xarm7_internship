#!/usr/bin/env python3
"""
Inspired by https://github.com/AndrejOrsula/pymoveit2/blob/main/examples/ex_gripper.py
Example of interacting with the gripper.
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_gripper.py --ros-args -p action:="toggle"
- ros2 run xarm7_moveit xarm7_gripper.py --ros-args -p action:="open"
- ros2 run xarm7_moveit xarm7_gripper.py --ros-args -p action:="close"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import GripperInterface

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_gripper_xarm7")

    # Declare parameter for gripper action
    node.declare_parameter(
        "action",
        "toggle",
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create gripper interface for xArm7
    gripper_interface = GripperInterface(
        node=node,
        gripper_joint_names=["drive_joint"],  # Name of the gripper joint
        open_gripper_joint_positions=[0.0],  # Open positions of your gripper
        closed_gripper_joint_positions=[0.85],  # Closed positions of your gripper
        gripper_group_name="xarm_gripper",  # Name of the gripper group
        callback_group=callback_group,
        gripper_command_action_name="gripper_action",  # Name of the gripper command action
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Sleep a while in order to get the first joint state
    node.create_rate(10.0).sleep()

    # Get parameter
    action = node.get_parameter("action").get_parameter_value().string_value

    # Perform gripper action
    node.get_logger().info(f'Performing gripper action "{action}"')
    if "open" == action:
        gripper_interface.open()
        gripper_interface.wait_until_executed()
    elif "close" == action:
        gripper_interface.close()
        gripper_interface.wait_until_executed()
    else:
        # Default action is toggle
        gripper_interface()
        gripper_interface.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()

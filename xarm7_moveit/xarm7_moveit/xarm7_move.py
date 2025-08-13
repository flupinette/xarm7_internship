#!/usr/bin/env python3
"""
Demonstration script for moving the xArm7 using joint positions.
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_move
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class XArm7Controller(Node):
    def __init__(self):
        super().__init__('xarm7_move_node')

        # Initialize MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint1", "joint2", "joint3",
                "joint4", "joint5", "joint6", "joint7"
            ],
            base_link_name="link_base",
            end_effector_name="link_eef",
            group_name="xarm7"
        )

        self.get_logger().info("Initialization in progress...")
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Add collision objects to the planning scene
        self.add_table()
        self.add_rail()
        self.add_cylinders()
        
        # Define joint positions for the robot arm
        joint_positions = [
            -1.309,      # joint1
            -0.768,     # joint2
            0.855,      # joint3
            0.524,     # joint4
            0.611,      # joint5
            1.152,      # joint6
            -0.890       # joint7
        ]

        self.get_logger().info("Planning...")
        self.moveit2.max_velocity = 0.1  # 10% of max velocity
        self.moveit2.max_acceleration = 0.1
        self.moveit2.move_to_configuration(joint_positions)
        
        self.moveit2.wait_until_executed()
        self.get_logger().info("Second pause")
        
        """
        # WARNING ! The following pose is a bit extreme for the robot. Please adjust it according to your needs.
        # If you want to use the pose below, make sure the robot is in a safe position before executing it.
        # Define a target pose for the end effector
        target_pose = PoseStamped()
        target_pose.header.frame_id = "link_base"
        target_pose.pose.position.x = -0.277
        target_pose.pose.position.y = 0.003
        target_pose.pose.position.z = 0.411
        target_pose.pose.orientation.x = 0.847
        target_pose.pose.orientation.y = 0.526
        target_pose.pose.orientation.z = -0.07
        target_pose.pose.orientation.w = -0.015

        self.get_logger().info("Planning...")
        self.moveit2.max_velocity = 0.8
        self.moveit2.max_acceleration = 0.8
        self.moveit2.move_to_pose(pose=target_pose)
        
        self.moveit2.wait_until_executed()
        self.get_logger().info("Movement completed.") """
        

    def add_table(self):
        """ Add a collision object representing a table to the planning scene. """
        table = CollisionObject()
        table.id = "table"
        table.header.frame_id = "world"

        # Define the box for the table
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 2.0, 0.02]  # Dimensions of the table

        # Position of the table
        table_pose = Pose()
        table_pose.position.x = 0.5
        table_pose.position.y = 0.0
        table_pose.position.z = -0.1  # Approximately where the table is located (with the captor it's -0.04)

        table.primitives.append(box)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD

        # Publish the collision object
        planning_scene = self.create_publisher(CollisionObject, "/collision_object", 10)
        planning_scene.publish(table)

    def add_rail(self):
        """ Add a collision object representing the linear rail under the robot. """
        rail = CollisionObject()
        rail.id = "rail"
        rail.header.frame_id = "world"

        # Define the box for the rail
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.15, 2.0, 0.24]  # width, length, height of the rail

        # Position of the rail
        rail_pose = Pose()
        rail_pose.position.x = 0.0    # Centered under the robot
        rail_pose.position.y = 0.0
        rail_pose.position.z = -0.06 # with the captor it's 0

        rail.primitives.append(box)
        rail.primitive_poses.append(rail_pose)
        rail.operation = CollisionObject.ADD

        # Publish the collision object
        planning_scene = self.create_publisher(CollisionObject, "/collision_object", 10)
        planning_scene.publish(rail)
        
    
    def add_cylinders(self):
        """ Add two upright cylinders as collision objects. """
        planning_scene = self.create_publisher(CollisionObject, "/collision_object", 10)

        for i, (x_mm, y_mm) in enumerate([(460, -410), (470, 360)], start=1):
            cylinder = CollisionObject()
            cylinder.id = f"cylinder_{i}"
            cylinder.header.frame_id = "world"

            # Create the cylinder primitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [0.40, 0.03]  # [height, radius]

            # Create the pose
            pose = Pose()
            pose.position.x = x_mm / 1000.0  # convert mm to m
            pose.position.y = y_mm / 1000.0
            pose.position.z = 0.1

            # Orientation: no rotation needed since the default is upright along Z
            pose.orientation.w = 1.0

            cylinder.primitives.append(primitive)
            cylinder.primitive_poses.append(pose)
            cylinder.operation = CollisionObject.ADD

            planning_scene.publish(cylinder)


def main():
    rclpy.init()
    node = XArm7Controller()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

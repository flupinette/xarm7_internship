#!/usr/bin/env python3
"""
Example of circular movement on xArm7 with joint positions (without servo).
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_circle
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from pymoveit2 import MoveIt2

from math import cos, sin, pi

class XArm7CartesianCircle(Node):
    def __init__(self):
        super().__init__('xarm7_cartesian_circle')

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6', 'joint7'
            ],
            base_link_name='link_base',
            end_effector_name='link_eef',
            group_name='xarm7',
            callback_group=self.callback_group,
        )

        self.get_logger().info("Initialization complete, waiting for MoveIt2...")

        self.execute_cartesian_circle()
    
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

    def execute_slow_cartesian(self, pose, speed_factor=0.5): # Do not set below 0.4 otherwise the robot will do everything but a circle
        """ Execute a cartesian trajectory to the given pose with slowing down. This function makes the robot shake if the speed_factor value is too low."""
        trajectory = self.moveit2.plan(
            pose = pose,
            cartesian=True,
            max_step=0.01,  
        )
        
        if trajectory:
            # Slow down the trajectory
            for point in trajectory.points:
                # Convert existing time to seconds (float)
                original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

                # Calculate the new time
                new_time = original_time * (1.0 / speed_factor)

                # Decompose into sec and nanosec
                point.time_from_start.sec = int(new_time)
                point.time_from_start.nanosec = int((new_time - int(new_time)) * 1e9)

            # Execute the trajectory
            self.moveit2.execute(trajectory)

    def execute_cartesian_circle(self):
        self.get_logger().info("Creating cartesian circular trajectory...")
        
        self.add_table()
        self.add_rail()
        self.add_cylinders()

        radius = 0.1
        center_x = 0.4
        center_y = 0.0
        z = 0.3
        num_points = 100 #Nombre de points dans la trajectoire circulaire
        while True: # Boucle infinie pour répéter la trajectoire
            for i in range(num_points + 1): 
                angle = 2 * pi * i / num_points
                pose = PoseStamped()
                pose.header.frame_id = 'link_base'  
                pose.pose.position.x = center_x + radius * cos(angle)
                pose.pose.position.y = center_y + radius * sin(angle)
                pose.pose.position.z = z
                # Fixed orientation
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 1.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0

                self.get_logger().info(f"Planning and executing to point {i} / {num_points}")
                
                self.execute_slow_cartesian(pose)
                
                """                # Uncomment this block if you want to use the original cartesian trajectory planning
                trajectory = self.moveit2.plan(
                pose = pose,
                cartesian=True,
                max_step=0.01,  
                )
        
                self.moveit2.execute(trajectory)
                """
                self.moveit2.wait_until_executed()

            self.get_logger().info("Circular trajectory completed. Restarting!")

def main():
    rclpy.init()
    node = XArm7CartesianCircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
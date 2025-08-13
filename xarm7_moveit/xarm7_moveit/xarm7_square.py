#!/usr/bin/env python3
"""
Example of square movement on xArm7. It is really similar to the circle scenario.
- ros2 run xarm7_moveit xarm7_square
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from pymoveit2 import MoveIt2

class XArm7CartesianSquare(Node):
    def __init__(self):
        super().__init__('xarm7_cartesian_square')

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

        self.execute_cartesian_square()
        
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

    def execute_cartesian(self, pose):
        """Execute a Cartesian movement"""
        trajectory = self.moveit2.plan(
            pose=pose,
            cartesian=True,
            max_step=0.0001,  
        )
        
        if trajectory:
            self.moveit2.execute(trajectory)

    def execute_cartesian_square(self):
        self.get_logger().info("Creating cartesian square trajectory...")

        self.add_table()
        self.add_rail()
        self.add_cylinders()

         # Square parameters
        side_length = 0.2  # Side length in meters
        center_x = 0.4     # Center of the square in x
        center_y = 0.0     # Center of the square in y
        z = 0.3            # Constant height

        # Square points (corner1, corner2, corner3, corner4, return to corner1)
        square_points = [
            (center_x - side_length/2, center_y - side_length/2),  # bottom left corner
            (center_x + side_length/2, center_y - side_length/2),  # bottom right corner
            (center_x + side_length/2, center_y + side_length/2),  # top right corner
            (center_x - side_length/2, center_y + side_length/2),  # top left corner
            (center_x - side_length/2, center_y - side_length/2)   # return to start
        ]

        n = 0
        while n<2:
            for i, (x, y) in enumerate(square_points):
                pose = PoseStamped()
                pose.header.frame_id = 'link_base'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                # Fixed orientation
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 1.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0

                self.get_logger().info(f"Planning and executing towards point {i+1} / {len(square_points)}")

                self.execute_cartesian(pose)
                self.moveit2.wait_until_executed()
            n += 1

            self.get_logger().info("Square trajectory completed. Restarting... Maybe")

def main():
    rclpy.init()
    node = XArm7CartesianSquare()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
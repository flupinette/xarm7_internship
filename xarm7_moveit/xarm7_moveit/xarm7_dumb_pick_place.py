#!/usr/bin/env python3
"""
Inspired by https://github.com/AndrejOrsula/gz_moveit2_examples/blob/master/examples/py/ex_throw_object.py
A pick and place scenario where everything is hard coded.
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_dumb_pick_place
"""
from copy import deepcopy
import math
from threading import Thread

import rclpy
import time 
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from pymoveit2 import MoveIt2, MoveIt2Gripper
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation as R
from xarm_msgs.srv import LinearTrackSetPos, LinearTrackBackOrigin
from xarm_msgs.srv import SetInt16
from moveit_msgs.msg import PlanningScene, ObjectColor
from std_msgs.msg import ColorRGBA


from xarm7_linear_motor import LinearTrackController

class MoveItPickPlace(Node):
    def __init__(self):
        super().__init__("ex_pick_place")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6', 'joint7'
            ],
            base_link_name='link_base',
            end_effector_name='link_eef',
            group_name='xarm7',
            callback_group=self._callback_group,
        )
        
        # Use upper joint velocity and acceleration limits
        self._moveit2.max_velocity = 0.5
        self._moveit2.max_acceleration = 0.5

        # Create MoveIt 2 interface for gripper
        self._moveit2_gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=["drive_joint"],  # Name of the gripper joint
            open_gripper_joint_positions=[0.00],  # Open positions of your gripper
            closed_gripper_joint_positions=[0.85],  # Closed positions of your gripper
            gripper_group_name="xarm_gripper",  # Name of the gripper group
            callback_group=self._callback_group,
        )
        
        self._collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        
        self.linear_track = LinearTrackController()  # Create linear track controller
        
        # Check if the linear track service is available to determine simulation mode
        self.simulation_mode = not self.linear_track.enable_client.wait_for_service(timeout_sec=1.0)
        if self.simulation_mode:
            self.get_logger().warn("Simulation detected: the rail will not be controlled.")
        else:
            self.get_logger().info("Real mode detected: rail control enabled.")
            self.linear_track.call_service_sync(self.linear_track.enable_client, SetInt16.Request(data=1), "enable")
            self.linear_track.call_service_sync(
                    self.linear_track.home_client,
                    LinearTrackBackOrigin.Request(wait=True),
                    "back_origin"
                )

        self.get_logger().info("Initialization successful.")
    
    
    def execute_slow_cartesian(self, pose, speed_factor=0.9): # Do not set below 0.4 otherwise the robot will do everything but what you want it to do
        """ Execute a cartesian trajectory to the given pose with slowing down. This function makes the robot shake if the speed_factor value is too low."""
        trajectory = self._moveit2.plan(
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
            self._moveit2.execute(trajectory)

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

    def add_object(self):
        """ Add a collision object representing the item to be picked up. """
        obj = CollisionObject()
        obj.id = "pick_object"
        obj.header.frame_id = "world"
        
        # Define the box for the object (4 cm cube)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.04, 0.04, 0.04]  # 4 cm on each side

        # Position of the object on the table
        obj_pose = Pose()
        obj_pose.position.x = 0.5
        obj_pose.position.y = -0.1
        obj_pose.position.z = -0.07  # on the table (thickness = 0.02)

        obj.primitives.append(box)
        obj.primitive_poses.append(obj_pose)
        obj.operation = CollisionObject.ADD

        # Publish the collision object
        self._collision_pub.publish(obj)
        
        # Color for the object
        color = ObjectColor()
        color.id = "pick_object"
        color.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red

        scene = PlanningScene()
        scene.is_diff = True
        scene.object_colors.append(color)

        scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        scene_pub.publish(scene)


    def add_cylinders(self):
        """ Add two upright cylinders as collision objects. """
        for i, (x_mm, y_mm) in enumerate([(460, -410)], start=1): # The robot can not reach the second cylinder when the linear track is between 0 mm and 200 mm
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

            self._collision_pub.publish(cylinder)

    def pickplace(self):
        """ Perform a pick and place operation. """
        
        # Log the start of the pick and place operation
        self.get_logger().info("Starting pick and place...")

        # Add collision objects
        self.add_table()
        self.add_rail()
        self.add_cylinders()
        if self.simulation_mode:
            self.add_object()
        
        # Orientation downward (gripper pointing downward)
        quat = R.from_euler('xyz', [math.pi, 0, 0]).as_quat()  # downward
        downward_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Positions
        pick_position = Point(x=0.5, y=-0.1, z=-0.04)
        place_position = Point(x=0.5, y=0.4, z=-0.04)

        # Intermediate positions above the targets
        position_above_pick = deepcopy(pick_position)
        position_above_pick.z += 0.4
        
        position_above_place = deepcopy(place_position)
        position_above_place.z += 0.4
        
        position_pick = deepcopy(pick_position)
        position_pick.z += 0.13
        
        position_place = deepcopy(place_position)
        position_place.z += 0.13


        # 1. Open the gripper
        self._moveit2_gripper.open()

        # 2. Move above the pick position
        pose = Pose(position=position_above_pick, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()
        
        # 3. Descend to the pick position
        pose = Pose(position=position_pick, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()
        self._moveit2_gripper.wait_until_executed()
        
        # 4. Close the gripper
        if self.simulation_mode:
            self._moveit2.attach_collision_object(id="pick_object",link_name="link_eef",touch_links=["link_eef", "xarm_gripper_link", "xarm_gripper_base_link"] )
            self._moveit2_gripper.move_to_position(position=0.5)
        else:
            self.get_logger().info("Closing gripper...")
            self._moveit2_gripper.move_to_position(position=0.5)
        self._moveit2_gripper.wait_until_executed()
        time.sleep(1.0) 

        # 5. Move up
        pose = Pose(position=position_above_pick, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()

        # 5.5. Move the rail to 200 mm
        if not self.simulation_mode:
            self.linear_track.call_service_sync(self.linear_track.speed_client, SetInt16.Request(data=90), "set_speed")
            move_req = LinearTrackSetPos.Request()
            move_req.pos = 200
            move_req.wait = True
            self.linear_track.call_service_sync(self.linear_track.move_client, move_req, "move_to_150")

        # 6. Move above the place position
        pose = Pose(position=position_above_place, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()

        # 7. Descend to the place position
        pose = Pose(position=position_place, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()

        # 8. Open the gripper
        self._moveit2_gripper.open()
        if self.simulation_mode:
            self._moveit2.detach_collision_object(id="pick_object")
        time.sleep(1.0)  

        # 9. Move up
        pose = Pose(position=position_above_place, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()
        self._moveit2_gripper.wait_until_executed()

        # 10. Return to the home position

        joint_positions = [
            0.0,      # joint1
            -0.253,     # joint2
            0.0,      # joint3
            -0.042,     # joint4
            0.0,      # joint5
            0.127,      # joint6
            0.0       # joint7
        ]

        self.get_logger().info("Moving to home position...")
        self._moveit2.move_to_configuration(joint_positions)

        # 11. Close the gripper
        self._moveit2_gripper.close()
        self._moveit2_gripper.wait_until_executed()
        self._moveit2.wait_until_executed()

        # 11.5. Move the rail back to position 0
        if not self.simulation_mode:
            move_req = LinearTrackSetPos.Request()
            move_req.pos = 0
            move_req.wait = True
            self.linear_track.call_service_sync(self.linear_track.move_client, move_req, "move_to_0")

        self.get_logger().info("Pick and place completed.")

def main(args=None):
    rclpy.init(args=args)

    pick_placer = MoveItPickPlace()

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(pick_placer)
    executor.add_node(pick_placer.linear_track)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Wait for everything to setup
    sleep_duration_s = 2.0
    if rclpy.ok():
        pick_placer.create_rate(1 / sleep_duration_s).sleep()

    pick_placer.pickplace()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()

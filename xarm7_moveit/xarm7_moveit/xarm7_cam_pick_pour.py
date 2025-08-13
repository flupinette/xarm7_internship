#!/usr/bin/env python3
"""
A pick and place scenario where the robot arm picks up a bowl from a position that isn't hardcoded and needs to be detected using camera vision and then pour it in a target bowl. The target bowl also needs to be detected with the camera. Everything else is hardcoded.
You should have calibrated your camera before running this code. Please do not forget to do an hand to eye calibration too.  See the calib.launch.py script for an example of how to use your calibration afterwards.
You will need to launch an equivalent of the calib.launch.py.
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_cam_pick_pour
"""

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener
from pymoveit2 import MoveIt2, MoveIt2Gripper
import tf_transformations
from xarm7_linear_motor import LinearTrackController
from xarm_msgs.srv import SetInt16
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import quaternion_from_euler
from xarm_msgs.srv import LinearTrackSetPos, LinearTrackBackOrigin
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import time 
from threading import Thread
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
import math

class BowlPicker(Node):
    def __init__(self):
        super().__init__("bowl_picker")
        # Bowl constants
        self.source_bowl_radius_real = 0.0655  # Reel radius of the source bowl in meters
        self.target_bowl_radius_real = 0.0835  # Reel radius of the target bowl in meters
        self.initial_position = Point(x=0.5, y=-0.05, z=0.4) # First position to move to
        self.above_pour_position = Point(x=0.5, y=0.35, z=0.4) # Position above the pour position
        # self.pour_position = Point(x=0.5, y=0.35, z=0.22)  # Pouring position (before changing position)
        self.grasp_height = 0.11  # Height to grasp the bowl
        self.pour_height = 0.22  # Height to pour
        self.lift_height = 0.4  # Height to lift the bowl after grasping (or pouring)
        self.move_step = 0.05  # Movement step in meters
        self.processed = False  # Add a flag to indicate if the image has been processed


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
            closed_gripper_joint_positions=[0.835],  # Closed positions of your gripper
            gripper_group_name="xarm_gripper",  # Name of the gripper group
            callback_group=self._callback_group,
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        
        self._collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10) # Publisher for collision objects
        
        self.linear_track = LinearTrackController()  # Create linear track controller
        
        # Check if the linear track service is available
        self.simulation_mode = not self.linear_track.enable_client.wait_for_service(timeout_sec=1.0) 

        if self.simulation_mode:
            self.get_logger().warn("Simulation détectée : le rail ne sera pas contrôlé.")
        else:
            self.get_logger().info("Mode réel détecté : contrôle du rail activé.")
            self.linear_track.call_service_sync(self.linear_track.enable_client, SetInt16.Request(data=1), "enable")
            self.linear_track.call_service_sync(
                    self.linear_track.home_client,
                    LinearTrackBackOrigin.Request(wait=True),
                    "back_origin"
                )


        # Variables
        self.current_image_data = None
        self.image_width = 640
        self.image_height = 480
        self.camera_matrix = None
        self.dist_coeffs = None

        # Add collision objects
        self.add_cylinders()
        self.add_table()
        self.add_rail()
        
        self.get_logger().info("Initialization successful.")
        
        
    def execute_slow_cartesian(self, pose, speed_factor=0.9): # DO NOT set below 0.4 otherwise the robot will do everything but what you want it to do
        """ Execute a cartesian trajectory to the given pose with slowing down. This function makes the robot shake if the speed_factor value is too low."""
        trajectory = self._moveit2.plan(
            pose = pose,
            cartesian=True,
            max_step=0.01,  
        )
        
        if trajectory is None:
            self.get_logger().error("Cartesian plan to grasp pose failed!")
            return
        
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


    def add_cylinders(self):
        """ Add two upright cylinders as collision objects. """
        for i, (x_mm, y_mm) in enumerate([(460, -410),(470, 770)], start=1): # The robot can not reach the second cylinder when the linear track is between 0 mm and 200 mm
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
        
    def camera_info_callback(self, msg):
        """Retrieve the intrinsic parameters of the camera"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info("Camera parameters received")

    def image_callback(self, msg: Image):
        """Callback for camera images - manual conversion"""
        if self.processed:
            return

        encoding = msg.encoding
        height = msg.height
        width = msg.width
        step = msg.step

        try:
            if encoding == 'rgb8': 
                channels = 3
            elif encoding == 'bgr8':
                channels = 3
            elif encoding == 'mono8':
                channels = 1
            else:
                self.get_logger().warn(f'Unsupported encoding: {encoding}')
                return

            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            img_2d = np_arr.reshape((height, step))

            img = img_2d[:, :width*channels]

            if channels > 1:
                img = img.reshape((height, width, channels))
            else:
                img = img.reshape((height, width))

            if encoding == 'rgb8':
                img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
            self.current_image_data = img
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def detect_bowl(self, img):
        """Bowl detection function"""
        if img is None:
            return False, None, None, None
        
        # Convert to grayscale and blur
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)

        # Circle detection with HoughCircles
        circles = cv.HoughCircles(
            gray, cv.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=100, param2=30, minRadius=20, maxRadius=300
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            # I chose to only keep the biggest circle found : it's what worked the best for me, could be changed depending on the setup
            x,y,r = circles[0]
            # Display the circle found to the user
            display_img = img.copy()
            cv.circle(display_img, (x,y), r, (0,255,0), 4)
            cv.circle(display_img, (x,y), 2, (0,0,255), 3)
            cv.imshow("Is this the right bowl?", display_img)
            cv.waitKey(0)
            cv.destroyAllWindows()
            return True, (x, y), r, display_img
        else:
            print("No circle detected")
            cv.imshow("Is this the right bowl?", img)
            cv.waitKey(0)
            cv.destroyAllWindows()
            return False, None, None, img

    def pixel_to_world_offset(self, pixel_x, pixel_y, radius, is_target_bowl=False):
        """
        Calculate the world offset in coordinates relative to the current position
        """
        if self.camera_matrix is None:
            return None, None

        # Center of the image
        center_x = self.image_width // 2
        center_y = self.image_height // 2

        # Offset in pixels from the center
        pixel_offset_x = pixel_x - center_x
        pixel_offset_y = pixel_y - center_y

        # Intrinsic parameters
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]

        # Estimated depth
        if is_target_bowl:
            estimated_depth = (self.target_bowl_radius_real * fx) / radius
        else:
            estimated_depth = (self.source_bowl_radius_real * fx) / radius
        
        # Convert to camera coordinates
        x_cam = pixel_offset_x * estimated_depth / fx
        y_cam = pixel_offset_y * estimated_depth / fy
        z_cam = estimated_depth
        
        try:
            if not self.tf_buffer.can_transform('link_eef', 'camera_color_optical_frame', rclpy.time.Time()):
                self.get_logger().error("TF transform not available")
                return None, None

            # Transform from camera to end effector frame
            transform = self.tf_buffer.lookup_transform(
                'link_eef', 'camera_color_optical_frame', rclpy.time.Time()
            )

            # Create a point in the camera frame
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_color_optical_frame'
            point_cam.point.x = x_cam
            point_cam.point.y = y_cam
            point_cam.point.z = 0.0  # We only want the XY offset

            # Transform to end effector frame and then to world frame
            try:
                # Camera -> End-effector
                point_eef = tf2_geometry_msgs.do_transform_point(point_cam, transform)
                
                # End-effector -> World
                transform_world = self.tf_buffer.lookup_transform(
                    'world', 'link_eef', rclpy.time.Time()
                )
                point_eef_stamped = PointStamped()
                point_eef_stamped.header.frame_id = 'link_eef'
                point_eef_stamped.point = point_eef.point
                
                point_world = tf2_geometry_msgs.do_transform_point(point_eef_stamped, transform_world)
                
                return point_world.point.x, point_world.point.y
                
            except Exception as e:
                self.get_logger().error(f'Error in world transform: {e}')
                return None, None
                
        except Exception as e:
            self.get_logger().error(f'Error in camera transform: {e}')
            return None, None

    def show_image_and_ask_confirmation(self, image):
        """Ask confirmation for the detected bowl"""
        cv.waitKey(1)
        
        while True:
            response = input("Do you confirm the bowl detection? ([Y]/n): ").lower().strip()
            if response in ['y', 'yes', 'oui','']:
                cv.destroyAllWindows()
                return True
            elif response in ['n', 'no', 'non']:
                cv.destroyAllWindows()
                return False
            else:
                print("Please respond with 'y' or 'n'")

    def ask_movement_direction(self):
        """Ask for the movement direction"""
        while True:
            direction = input("Direction (x+, x-, y+, y-): ").lower().strip()
            if direction in ['x+', 'x-', 'y+', 'y-']:
                return direction
            else:
                print("Please respond with 'x+', 'x-', 'y+', or 'y-'")

    def move_relative(self, direction, current_pose):
        """Move the robot according to the given direction"""
        new_pose = deepcopy(current_pose)
        
        if direction == 'x+':
            new_pose.position.x += self.move_step
        elif direction == 'x-':
            new_pose.position.x -= self.move_step
        elif direction == 'y+':
            new_pose.position.y += self.move_step
        elif direction == 'y-':
            new_pose.position.y -= self.move_step
        
        self.execute_slow_cartesian(new_pose)
        self._moveit2.wait_until_executed()
        return new_pose
    
    def detect_and_locate_bowl(self, current_pose, bowl_type="source"):
        """
        Generic function to detect and locate a bowl
        bowl_type: "source" or "target"
        """
        is_target = (bowl_type == "target")
        bowl_found = False
        quat = R.from_euler('xyz', [math.pi, 0, 0]).as_quat()
        downward_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Detection loop
        while not bowl_found:
            self.get_logger().info(f"Taking photo and detecting {bowl_type} bowl...")
            
            if self.current_image_data is None:
                self.get_logger().warn("No image available, waiting...")
                time.sleep(1)
                continue

            # Detect the bowl
            detected, center, radius, result_image = self.detect_bowl(self.current_image_data)
            
            if detected:
                self.get_logger().info(f"{bowl_type.capitalize()} bowl detected at pixel ({center[0]}, {center[1]}) with radius {radius}")

                # Show the picture and ask for confirmation
                if self.show_image_and_ask_confirmation(result_image):
                    self.get_logger().info(f"{bowl_type.capitalize()} bowl confirmed by user!")

                    # Automatic offset calculation
                    offset_x, offset_y = self.pixel_to_world_offset(center[0], center[1], radius, is_target)
                    
                    if offset_x is not None and offset_y is not None:
                        self.get_logger().info(f"Calculated offset: x={offset_x:.3f}, y={offset_y:.3f}")

                        # Center the robot on the bowl
                        centered_pose = deepcopy(current_pose)
                        
                        if is_target:
                            # For the target bowl, position with offset to pour
                            centered_pose.position.x = offset_x - self.target_bowl_radius_real*1.7 # 1.7 is an empirical factor : it was what seemed to work the best but can be adapted depending on the setup
                            centered_pose.position.y = offset_y + self.target_bowl_radius_real
                        else:
                            # For the source bowl, position at center + offset to catch the edge
                            centered_pose.position.x = offset_x
                            centered_pose.position.y = offset_y + self.source_bowl_radius_real
                        
                        self.get_logger().info(f"Centering on {bowl_type} bowl...")
                        self.execute_slow_cartesian(centered_pose)
                        self._moveit2.wait_until_executed()
                        
                        current_pose = centered_pose
                        bowl_found = True
                    else:
                        self.get_logger().warn("Could not calculate position automatically")
                        bowl_found = True
                else:
                    self.get_logger().info(f"{bowl_type.capitalize()} bowl not confirmed by user")
            else:
                self.get_logger().info(f"No {bowl_type} bowl detected")
                cv.imshow(f"No {bowl_type} bowl detected", self.current_image_data)
                cv.waitKey(1)
            
            if not bowl_found:
                direction = self.ask_movement_direction()
                self.get_logger().info(f"Moving {direction}...")
                current_pose = self.move_relative(direction, current_pose)

        cv.destroyAllWindows()
        return current_pose

    def bowl_grasping_sequence(self):
        """Main sequence for grasping the bowl"""
        self.get_logger().info("Starting bowl grasping sequence...")

        # Downward orientation
        quat = R.from_euler('xyz', [math.pi, 0, 0]).as_quat()
        downward_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # 1. Initial position
        initial_pose = Pose(position=self.initial_position, orientation=downward_quat)
        self.get_logger().info("Moving to initial position...")
        self.execute_slow_cartesian(initial_pose)
        self._moveit2.wait_until_executed()

        # 2. Open the gripper
        self._moveit2_gripper.open()
        self._moveit2_gripper.wait_until_executed()

        # 3. Grasping sequence
        current_pose = self.detect_and_locate_bowl(initial_pose,"source")
        
        self.get_logger().info("Starting grasping sequence...")
        # Position to catch the bowl
        grasp_pose = deepcopy(current_pose)
        print(current_pose.position.z)
        grasp_pose.position.z = self.grasp_height
        self.get_logger().info("Descending to bowl...")
        self.execute_slow_cartesian(grasp_pose)
        self._moveit2.wait_until_executed()


        # 4. Close the gripper
        self.get_logger().info("Closing gripper...")
        self._moveit2_gripper.close()
        self._moveit2_gripper.wait_until_executed()
        time.sleep(1.0)

        # 5. Lift
        lift_pose = deepcopy(current_pose)
        lift_pose.position.z = self.lift_height
        
        self.get_logger().info("Lifting bowl...")
        self.execute_slow_cartesian(lift_pose)
        self._moveit2.wait_until_executed()
        
        
        # 5.5. Move the rail to 250 mm
        if not self.simulation_mode:
            self.linear_track.call_service_sync(self.linear_track.speed_client, SetInt16.Request(data=90), "set_speed")
            move_req = LinearTrackSetPos.Request()
            move_req.pos = 275
            move_req.wait = True
            self.linear_track.call_service_sync(self.linear_track.move_client, move_req, "move_to_250")

        # 6. Move above pour position
        pose = Pose(position=self.above_pour_position, orientation=downward_quat)
        self.execute_slow_cartesian(pose)
        self._moveit2.wait_until_executed()

        # 7. Descend to pour position
        above_pour_pose = self.detect_and_locate_bowl(pose, "target")
        soon_pour_pose = deepcopy(above_pour_pose)
        soon_pour_pose.position.z = self.pour_height  # Adjust height for pouring
        self.get_logger().info("Descending to pour position...")
        self.execute_slow_cartesian(soon_pour_pose)
        self._moveit2.wait_until_executed()

        # 8. Pour
        pour_quat = R.from_euler('xyz', [math.pi, -math.pi/1.7, 0]).as_quat() #R.from_euler('xyz',[-math.pi/2.3, 0, 0]).as_quat() # Adjust orientation for pouring
        pour_pose = Pose(position=soon_pour_pose.position, orientation=Quaternion(x=pour_quat[0], y=pour_quat[1], z=pour_quat[2], w=pour_quat[3]))
        self.execute_slow_cartesian(pour_pose)
        self._moveit2.wait_until_executed()
        time.sleep(5.0)  
        
        # 9. Return to previous position
        self.execute_slow_cartesian(soon_pour_pose)
        self._moveit2.wait_until_executed()
        
        # 10. Move back up
        self.execute_slow_cartesian(above_pour_pose)
        self._moveit2.wait_until_executed()
        self._moveit2_gripper.wait_until_executed()
        
        # 16.5. Move the rail to position 0
        if not self.simulation_mode:
            move_req = LinearTrackSetPos.Request()
            move_req.pos = 0
            move_req.wait = True
            self.linear_track.call_service_sync(self.linear_track.move_client, move_req, "move_to_0")

        # 11. Move above the pick position
        self.execute_slow_cartesian(lift_pose)
        self._moveit2.wait_until_executed()

        # 12. Descend to pick position
        self.execute_slow_cartesian(grasp_pose)
        self._moveit2.wait_until_executed()
        self._moveit2_gripper.wait_until_executed()

        # 13. Open the gripper
        self.get_logger().info("Opening gripper...")
        self._moveit2_gripper.open()
        self._moveit2_gripper.wait_until_executed()
        time.sleep(1.0)

        # 14. Move up
        self.execute_slow_cartesian(lift_pose)
        self._moveit2.wait_until_executed()

        # 15. Return to home position
        
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

        # 16. Close the gripper
        self._moveit2_gripper.close()
        self._moveit2_gripper.wait_until_executed()
        self._moveit2.wait_until_executed()

        self.get_logger().info("Pick and pour completed.")
        
        
        self.get_logger().info("Bowl grasping sequence completed successfully!")

def main(args=None):
    rclpy.init(args=args)

    bowl_robot = BowlPicker()

    # Spin the node in background thread
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(bowl_robot)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Wait for setup
    sleep_duration_s = 3.0
    if rclpy.ok():
        bowl_robot.create_rate(1 / sleep_duration_s).sleep()

    try:
        bowl_robot.bowl_grasping_sequence()
    except KeyboardInterrupt:
        bowl_robot.get_logger().info("Interrupted by user")
    finally:
        cv.destroyAllWindows()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
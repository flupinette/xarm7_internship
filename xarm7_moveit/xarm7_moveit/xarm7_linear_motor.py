"""
Example of controlling the linear track on xArm7 using ROS2 services.
See the videos folder for a demonstration video.
- ros2 run xarm7_moveit xarm7_linear_motor
"""
import rclpy
from rclpy.node import Node

from xarm_msgs.srv import SetInt16, GetInt16
from xarm_msgs.srv import LinearTrackSetPos, LinearTrackBackOrigin


class LinearTrackController(Node):
    def __init__(self):
        super().__init__('linear_track_controller')

        # Declare service clients with the correct types
        self.enable_client = self.create_client(SetInt16, '/xarm/set_linear_track_enable')
        self.home_client = self.create_client(LinearTrackBackOrigin, '/xarm/set_linear_track_back_origin')
        self.speed_client = self.create_client(SetInt16, '/xarm/set_linear_track_speed')
        self.move_client = self.create_client(LinearTrackSetPos, '/xarm/set_linear_track_pos')
        self.pos_client = self.create_client(GetInt16, '/xarm/get_linear_track_pos')

    def call_service_sync(self, client, request, name="service"):
        # Check if the service is available
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {name} not available.')
            return None
        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # Check the result
        if future.result() is not None:
            self.get_logger().info(f'{name} → return: {future.result().ret} | message: {future.result().message}')
            return future.result()
        else:
            self.get_logger().error(f'Error calling {name}')
            return None

    def run_sequence(self):
        # 1. Activate the linear track
        self.call_service_sync(self.enable_client, SetInt16.Request(data=1), "enable")

        # 2. Return to the origin
        self.call_service_sync(self.home_client, LinearTrackBackOrigin.Request(wait=True), "back_origin")

        # 3. Set the speed (in mm/s)
        self.call_service_sync(self.speed_client, SetInt16.Request(data=50), "set_speed")

        # 4. Move the linear track
        move_req = LinearTrackSetPos.Request()
        move_req.pos = 150
        move_req.wait = True
        self.call_service_sync(self.move_client, move_req, "set_position")

        # 5. Read the current position
        result = self.call_service_sync(self.pos_client, GetInt16.Request(), "get_position")
        if result:
            self.get_logger().info(f"Current position of the rail: {result.data} mm")


def main(args=None):
    rclpy.init(args=args)
    controller = LinearTrackController()
    controller.run_sequence()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

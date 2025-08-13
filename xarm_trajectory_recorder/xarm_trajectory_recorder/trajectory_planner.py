import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import os

class TrajectoryPlayer(Node):
    def __init__(self, filename):
        super().__init__('trajectory_player')
        self.pub = self.create_publisher(JointTrajectory, '/xarm7_traj_controller/joint_trajectory', 10) # Topic to publish the trajectory in ROS2 xarm_moveit_servo
        self.filename = filename
        self.timer = self.create_timer(2.0, self.publish_trajectory)  # Publish after 2 seconds

    def publish_trajectory(self):
        """Publish the trajectory from the YAML file."""
        self.timer.cancel()
        with open(self.filename, 'r') as f:
            data = yaml.safe_load(f)

        traj = JointTrajectory()
        traj.joint_names = data['joint_names']
        for p in data['points']:
            point = JointTrajectoryPoint()
            point.positions = p['positions']
            point.time_from_start.sec = p['time_from_start']['sec']
            point.time_from_start.nanosec = p['time_from_start']['nanosec']
            traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f"Trajectory {self.filename} published to /xarm7_traj_controller/joint_trajectory")

def main(args=None):
    rclpy.init(args=args)
    yaml_file = os.path.abspath('freedrive_trajectory.yaml')
    node = TrajectoryPlayer(yaml_file)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

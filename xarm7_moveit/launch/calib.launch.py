""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link_eef -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link_eef",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.0963286",
                "--y",
                "0.0331063",
                "--z",
                "-0.0545427",
                "--qx",
                "0.0440578",
                "--qy",
                "-0.0446277",
                "--qz",
                "0.798276",
                "--qw",
                "0.599018",
                # "--roll",
                # "0.124371",
                # "--pitch",
                # "0.0168758",
                # "--yaw",
                # "1.85304",
            ],
        ),
    ]
    return LaunchDescription(nodes)

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
import yaml
import os

def read_joint_states_from_bag(bag_path, topic_name='/xarm/joint_states'):
    """Read joint states from a rosbag file."""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    joint_states = []
    start_time = None

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg = JointState()
            msg = deserialize_message(data, JointState)
            if start_time is None:
                start_time = t
            joint_states.append((msg, t))

    return joint_states, start_time

def joint_states_to_trajectory(joint_states, start_time, joint_names):
    """Convert joint states to a JointTrajectory."""
    traj = JointTrajectory()
    traj.joint_names = joint_names
    for msg, timestamp in joint_states:
        point = JointTrajectoryPoint()
        positions = []
        for jn in joint_names:
            try:
                idx = msg.name.index(jn)
                positions.append(msg.position[idx])
            except ValueError:
                positions.append(0.0)
        point.positions = positions

        time_from_start = (timestamp - start_time) / 1e9
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - point.time_from_start.sec) * 1e9)
        traj.points.append(point)

    return traj

def save_trajectory_to_yaml(traj, filename):
    """Save JointTrajectory to a YAML file."""
    data = {
        'joint_names': traj.joint_names,
        'points': []
    }
    for p in traj.points:
        data['points'].append({
            'positions': list(p.positions),
            'time_from_start': {
                'sec': p.time_from_start.sec,
                'nanosec': p.time_from_start.nanosec
            }
        })

    with open(filename, 'w') as f:
        yaml.dump(data, f)
    print(f"Trajectory saved to {filename}")
    
def clean_trajectory(traj): 
    """Remove points where all positions are zero."""
    cleaned_points = []
    for p in traj.points:
        # If every position is zero, skip this point -> it's not a valid point
        if all(abs(x) == 0 for x in p.positions):
            continue
        cleaned_points.append(p)
    traj.points = cleaned_points
    return traj


def main():
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    bag_path = '/home/e25vanac/bag_files/freedrive_record' # Bag file path
    output_yaml = 'freedrive_trajectory.yaml'

    print("Reading joint_states from rosbag...")
    joint_states, start_time = read_joint_states_from_bag(bag_path)

    print(f"Converting {len(joint_states)} joint states to trajectory...")
    traj = joint_states_to_trajectory(joint_states, start_time, joint_names)
    
    print(f"Trajectory has {len(traj.points)} points before cleaning.")
    traj = clean_trajectory(traj)
    print(f"Trajectory has {len(traj.points)} points after cleaning.")

    if not traj.points:
        print("No valid trajectory points found. Exiting.")
        return

    print("Saving trajectory to YAML file...")
    save_trajectory_to_yaml(traj, output_yaml)

if __name__ == '__main__':
    main()

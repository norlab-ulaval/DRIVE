import datetime
import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# TODO: Change this to your robot's name
robot_name = "castor"

# TODO: Change these topics according to your setup.
localization_topic = "mapping/pose"  # PoseStamped
localization_topic_type = "PoseStamped"  # "PoseStamped" or "Odometry"

drive_cmd_vel_topic = "doughnut_cmd_vel"
cmd_vel_topic_type = "Twist" # "Twist" or "TwistStamped"

deadman_pressed_topic = "lock_autonomy"  # Bool


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config", robot_name)

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]

    root_datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]
    
    # List folders in datasets_directory and get the most recent one by modification time
    dataset_folders = [d for d in os.listdir(root_datasets_directory) if os.path.isdir(os.path.join(root_datasets_directory, d))]
    if dataset_folders:
        latest_dir = max(dataset_folders, key=lambda d: os.path.getmtime(os.path.join(root_datasets_directory, d)))
    else:
        latest_dir = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S")
    dataset_directory = os.path.join(root_datasets_directory, latest_dir, "node_data")

    print(f"Saving in: {dataset_directory}")

    # Drive ros bridge
    drive_ros_node = Node(
        package="drive_ros",
        executable="drive_ros_bridge.py",
        parameters=[drive_ros_config_path, {"dataset_directory": dataset_directory, "localization_topic_type": localization_topic_type, "cmd_vel_topic_type": cmd_vel_topic_type}],
        remappings=[
            ("pose", localization_topic),
            ("cmd_drive", drive_cmd_vel_topic),
            ("pause_drive", deadman_pressed_topic),
        ],
    )

    return LaunchDescription([drive_ros_node])

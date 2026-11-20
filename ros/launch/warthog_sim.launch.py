import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from drive_ros.dataset_utils import resolve_dataset_directory

# TODO: Change this to your robot's name
DRIVE_ROBOT_NAME = "warthog_sim"

# TODO: Change these topics according to your setup.
localization_topic = "/mapping/icp_odom"  # PoseStamped or Odometry
localization_topic_type = "Odometry"  # "PoseStamped" or "Odometry"

drive_cmd_vel_topic = "/controller/cmd_vel"
cmd_vel_topic_type = "Twist"  # "Twist" or "TwistStamped"

deadman_pressed_topic = "/teleop/lock_autonomy"  # Bool


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config", DRIVE_ROBOT_NAME)

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]
    dataset_directory = resolve_dataset_directory(datasets_directory)

    print(f"Saving in: {dataset_directory}")

    # Drive ros bridge
    drive_ros_node = Node(
        package="drive_ros",
        executable="drive_ros_bridge.py",
        parameters=[
            drive_ros_config_path,
            {
                "dataset_directory": dataset_directory,
                "localization_topic_type": localization_topic_type,
                "cmd_vel_topic_type": cmd_vel_topic_type,
            },
        ],
        remappings=[
            ("pose", localization_topic),
            ("cmd_drive", drive_cmd_vel_topic),
            ("pause_drive", deadman_pressed_topic),
        ],
    )

    return LaunchDescription([drive_ros_node])

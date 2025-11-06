import datetime
import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# TODO: Change this to your robot's name
robot_name = "husky"

# TODO: Change these topics according to your setup.
localization_topic = "/mapping/icp_odom"
localization_topic_type = "Odometry"  # "PoseStamped" or "Odometry"

drive_cmd_vel_topic = "/controller/cmd_vel" 
cmd_vel_topic_type = "TwistStamped" # "Twist" or "TwistStamped"

deadman_pressed_topic = "/teleop/lock_autonomy"  # Bool


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config", robot_name)

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]
    dataset_name = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S")

    # Drive ros bridge
    drive_ros_node = Node(
        package="drive_ros",
        executable="drive_ros_bridge.py",
        parameters=[drive_ros_config_path, {"dataset_name": dataset_name, "localization_topic_type": localization_topic_type, "cmd_vel_topic_type": cmd_vel_topic_type}],
        remappings=[
            ("pose", localization_topic),
            ("cmd_drive", drive_cmd_vel_topic),
            ("pause_drive", deadman_pressed_topic),
        ],
    )

    return LaunchDescription([drive_ros_node])

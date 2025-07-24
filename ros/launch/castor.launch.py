import datetime
import os
import pathlib

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config", "castor")

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]
    dataset_name = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S")

    localization_topic = "mapping/pose" # PoseStamped
    drive_cmd_vel_topic = "doughnut_cmd_vel" # Twist
    controller_cmd_vel_topic = "nav_vel" # Twist
    pause_drive_topic = "lock_autonomy" # Bool

    # Drive ros bridge
    drive_ros_node = Node(
        package="drive_ros",
        executable="drive_ros_bridge.py",
        parameters=[drive_ros_config_path, {"dataset_name": dataset_name}],
        remappings=[("pose", localization_topic),
                    ("cmd_drive", drive_cmd_vel_topic),
                    ("pause_drive", pause_drive_topic),
        ]
    )

    # Controller
    p_controller = Node(package="drive_ros", executable="p_controller.py", remappings=[
            ("pose", localization_topic), 
            ("cmd_controller", controller_cmd_vel_topic),
            ("pause_drive", pause_drive_topic),
        ])

    # Starting rosbag
    topics_file = os.path.join(config_folder, "rosbag_topics.yaml")
    topics_list = yaml.safe_load(open(topics_file, "r"))["topics"]
    command = ["ros2", "bag", "record", "-o", f"{datasets_directory}/{dataset_name}"]
    command.extend(topics_list)

    rosbag_record_process = ExecuteProcess(name="rosbag_record", cmd=command, output="screen")

    return LaunchDescription([drive_ros_node, p_controller, rosbag_record_process])
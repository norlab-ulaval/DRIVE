import datetime
import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# TODO: Change this to your robot's name
robot_name = "warthog_sim"

# TODO: Change these topics according to your setup.
localization_topic = "/mapping/icp_odom"  # PoseStamped or Odometry
localization_topic_type = "Odometry"  # "PoseStamped" or "Odometry"

drive_cmd_vel_topic = "/controller/cmd_vel" 
cmd_vel_topic_type = "Twist" # "Twist" or "TwistStamped"

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

    # Starting rosbag
    topics_file = os.path.join(config_folder, "rosbag_topics.yaml")
    topics_list = yaml.safe_load(open(topics_file, "r"))["topics"]
    command = ["ros2", "bag", "record", "-s", "mcap", "-o", f"{datasets_directory}/{dataset_name}", "--topics"]
    command.extend(topics_list)

    rosbag_record_process = ExecuteProcess(name="rosbag_record", cmd=command, output="screen")

    return LaunchDescription([rosbag_record_process, drive_ros_node])

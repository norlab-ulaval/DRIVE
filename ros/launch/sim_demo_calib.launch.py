import datetime
import os
import pathlib

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config")

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["/**"]["ros__parameters"]["datasets_directory"]
    dataset_name = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S")

    # Twist mux
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux_node",
        parameters=[os.path.join(config_folder, "twist_mux.yaml")],
        remappings=[("cmd_vel_out", "cmd_vel")],
    )

    # Drive ros bridge
    calib_node = Node(
        package="drive_ros",
        executable="calibration_node.py",
        parameters=[drive_ros_config_path, {"dataset_name": dataset_name}],
        remappings=[("/left_motor_encoder", "/left_wheel_encoder"),
                    ("/right_motor_encoder", "/right_wheel_encoder"),
                    ("/left_motor_cmd", "/left_wheel_cmd"),
                    ("/right_motor_cmd", "/right_wheel_cmd")],
    )

    
    # Diff drive sim
    diff_drive_sim_node = Node(package="drive_ros", executable="point_mass_sim.py")

    # Controller
    p_controller = Node(
        package="drive_ros", executable="p_controller.py", parameters=[os.path.join(config_folder, "p_controller.yaml")]
    )

    # Starting rosbag
    topics_file = os.path.join(config_folder, "rosbag_topics.yaml")
    topics_list = yaml.safe_load(open(topics_file, "r"))["topics"]
    command = ["ros2", "bag", "record", "-o", f"{datasets_directory}/{dataset_name}"]
    command.extend(topics_list)

    rosbag_record_process = ExecuteProcess(name="rosbag_record", cmd=command, output="screen")

    return LaunchDescription([rosbag_record_process, twist_mux_node, calib_node, diff_drive_sim_node, p_controller])

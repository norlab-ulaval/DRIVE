import datetime
import os
import pathlib

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    config_folder = os.path.join(get_package_share_directory("drive_ros"), "config", "warthog_sim")

    drive_ros_config_path = os.path.join(config_folder, "drive_ros_bridge.yaml")
    drive_ros_config = yaml.safe_load(open(drive_ros_config_path, "r"))

    datasets_directory = drive_ros_config["drive_ros_bridge"]["ros__parameters"]["datasets_directory"]
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
    drive_ros_node = Node(
        package="drive_ros",
        executable="drive_ros_bridge.py",
        parameters=[drive_ros_config_path, {"dataset_name": dataset_name}],
    )

    # Diff drive sim
    diff_drive_sim_node = Node(package="drive_ros", executable="point_mass_sim.py")

    # Controller
    p_controller = Node(
        package="drive_ros", executable="p_controller.py", parameters=[os.path.join(config_folder, "p_controller.yaml")]
    )

    return LaunchDescription([twist_mux_node, drive_ros_node, diff_drive_sim_node, p_controller])

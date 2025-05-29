from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drive_ros_node = Node(package="drive_ros", executable="drive_ros_bridge")

    diff_drive_sim_node = Node(package="drive_ros", executable="diff_drive_sim")

    return LaunchDescription([drive_ros_node, diff_drive_sim_node])

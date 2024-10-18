import os, yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    share_folder = get_package_share_directory('norlab_robot')
    include_folder = os.path.join(share_folder, 'launch', 'include')

    icp_mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(include_folder, 'icp_mapper.launch.py')
        ])
    )

    imu_and_wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(include_folder, 'imu_and_wheel_odom.launch.py')
        ])
    )

    pcl_deskew_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(include_folder, 'pcl_deskew.launch.py')
        ])
    )

    return LaunchDescription([
        icp_mapper_launch,
        imu_and_wheel_odom_launch,
        pcl_deskew_launch
    ])
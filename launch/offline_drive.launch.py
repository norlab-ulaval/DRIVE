import os, yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    share_folder = get_package_share_directory('drive')
    launch_folder = os.path.join(share_folder, 'launch')

    icp_mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_folder, 'offline_mapping.launch.py')
        ])
    )

    logger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_folder, 'offline_logger_warthog.launch.py')
        ])
    )

    return LaunchDescription([
        icp_mapper_launch,
        logger_launch
    ])
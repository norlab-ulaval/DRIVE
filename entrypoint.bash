#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ws/install/setup.bash

screen -wipe > /dev/null 2>&1
killall screen > /dev/null 2>&1

if [ -z "$DRIVE_ROBOT_NAME" ]; then
  echo "Error: DRIVE_ROBOT_NAME is not set. Please set it in your environment or .env file."
  exit 1
fi

if [ -d /home/ws/drive_datasets ]; then
    sudo chown -R $(id -u):$(id -g) /home/ws/drive_datasets
fi

# Uncomment if foxglove is not launched on the host robot
# screen -S foxglove -dm bash -c 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'

ros2 launch drive_ros $DRIVE_ROBOT_NAME.launch.py
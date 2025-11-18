#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ws/install/setup.bash

screen -wipe > /dev/null 2>&1
killall screen > /dev/null 2>&1

if [ -z "$ROBOT_NAME" ]; then
  echo "Error: ROBOT_NAME is not set. Please set it in your environment or .env file."
  exit 1
fi

if [ -d /home/ws/drive_datasets ]; then
    sudo chown -R $(id -u):$(id -g) /home/ws/drive_datasets
fi


echo "DRIVE ROS container ready in GUI mode. Waiting for commands from interface..."
while true; do sleep 1; done

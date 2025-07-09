#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ws/install/setup.bash

screen -wipe > /dev/null 2>&1
killall screen > /dev/null 2>&1

screen -S foxglove -dm bash -c 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'

echo "ROS screens launched, container is ready"
while true; do sleep 1; done
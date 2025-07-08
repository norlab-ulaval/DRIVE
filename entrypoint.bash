#!/bin/bash

screen -wipe > /dev/null 2>&1
killall screen > /dev/null 2>&1

screen -S foxglove -dm bash -c 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
screen -S drive -dm bash -c 'ros2 launch drive_ros sim_demo.launch.py'
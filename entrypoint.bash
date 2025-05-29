#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /home/root/ros2_ws/install/setup.bash ]
then
    source /home/root/ros2_ws/install/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"
#!/bin/bash

# Script pour lancer le protocole DRIVE complet depuis le conteneur drive_gui
# Ce script exécute les commandes dans le conteneur drive_ros

ROBOT_NAME=${1:-warthog}
EXPERIENCE_NAME=${2:-""}
DEPLOYMENT_NAME=${3:-""}

echo "========================================="
echo "Launching DRIVE Protocol"
echo "Robot: $ROBOT_NAME"
echo "Experience: $EXPERIENCE_NAME"
echo "Deployment: $DEPLOYMENT_NAME"
echo "========================================="

# Vérifier que le conteneur drive_ros existe et est en cours d'exécution
if ! docker ps | grep -q drive_ros; then
    echo "Error: drive_ros container is not running!"
    exit 1
fi

# Lancer Foxglove Bridge dans une session screen
echo "Starting Foxglove Bridge..."
docker exec drive_ros bash -c "screen -S foxglove -dm bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && source /home/ws/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml'"

sleep 2

# Lancer le launch file du robot dans une session screen
echo "Launching DRIVE for robot: $ROBOT_NAME..."
docker exec drive_ros bash -c "screen -S drive -dm bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && source /home/ws/install/setup.bash && ros2 launch drive_ros ${ROBOT_NAME}.launch.py'"

sleep 2

echo "========================================="
echo "DRIVE Protocol launched successfully!"
echo "Foxglove Bridge: http://localhost:8765"
echo ""
echo "To check running nodes:"
echo "  docker exec drive_ros bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && ros2 node list'"
echo ""
echo "To view screen sessions:"
echo "  docker exec -it drive_ros screen -ls"
echo "========================================="

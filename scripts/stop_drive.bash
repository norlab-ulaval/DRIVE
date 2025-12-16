#!/bin/bash

# Script pour arrêter le protocole DRIVE

echo "========================================="
echo "Stopping DRIVE Protocol"
echo "========================================="

# Vérifier que le conteneur drive_ros existe et est en cours d'exécution
if ! docker ps | grep -q drive_ros; then
    echo "Error: drive_ros container is not running!"
    exit 1
fi

# Arrêter les sessions screen
echo "Stopping DRIVE nodes..."
docker exec drive_ros bash -c "screen -X -S drive quit" 2>/dev/null
docker exec drive_ros bash -c "screen -X -S foxglove quit" 2>/dev/null

# Nettoyer les processus ROS restants
echo "Cleaning up ROS processes..."
docker exec drive_ros bash -c "pkill -f 'ros2 launch'" 2>/dev/null
docker exec drive_ros bash -c "pkill -f 'foxglove'" 2>/dev/null

sleep 1

echo "========================================="
echo "DRIVE Protocol stopped successfully!"
echo "========================================="

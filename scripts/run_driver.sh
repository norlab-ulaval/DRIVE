#!/bin/bash

echo -n "Enter experiment name, if empty the name will be the current date and time: "
read experiment_name
if [[ -z "$var" ]]; then
   experiment_name="$(date '+%Y-%m-%d-%s')"
fi
echo "Killing old screens"
screen -S "sensors" -X stuff $'\003'
screen -S "mapping" -X stuff $'\003'
screen -S "drive" -X stuff $'\003'


echo "Starting sensors"
screen -dmS sensors ros2 launch warthog_mapping sensors.launch.xml
echo "Sensors started, access it with screen -r sensors"

echo "Starting mapping"
screen -dmS mapping ros2 launch warthog_mapping realtime_mapping.launch.xml
echo "Mapping started, access it with screen -r mapping"
sleep 5


echo "Launch drive in a screen name drive"
screen -dmS drive ros2 launch drive warthog.launch.xml 

screen -r drive

#sleep 2
#read -p "When dataset gathering is done, press Enter to save"
#
#current_directory=$PWD
#export_path=""$PWD"/../calib_data/"$experiment_name"/"
#echo -n $export_path
#
#if [ ! -d $export_path ]; then
#  mkdir -p $export_path;
#fi
#
#ros2 service call /logger_node/export_data norlab_controllers_msgs/srv/ExportData "export_path:
#  data: '"$export_path"/raw_dataframe.pkl'"
#sleep 5
#
#killall drive_node
#killall logger_node

#TODO: Complete integration with model_training module and automate model training

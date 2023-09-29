#!/bin/bash

echo -n "Enter experiment name, if empty the name will be the current date and time: "
read experiment_name
if [[ -z "$var" ]]; then
   experiment_name="$(date '+%Y-%m-%d-%s')"
fi


ros2 launch drive drive.launch.xml &
sleep 2
read -p "When dataset gathering is done, press Enter to save"

current_directory=$PWD
export_path=""$PWD"/../calib_data/"$experiment_name"/"
echo -n $export_path

if [ ! -d $export_path ]; then
  mkdir -p $export_path;
fi

ros2 service call /logger_node/export_data norlab_controllers_msgs/srv/ExportData "export_path:
  data: '"$export_path"/raw_dataframe.pkl'"
sleep 5

killall drive_node
killall logger_node

#TODO: Complete integration with model_training module and automate model training

#!/bin/bash

echo -n "Enter experiment name: "
read experiment_name

ros2 launch drive drive.launch.xml &
sleep 2
read -p "When dataset gathering is done, press Enter to save"

export_path="${PWD}/../${experiment_name}/"

if [ ! -d $export_path ]; then
  mkdir -p $export_path;
fi

srv_string="export_path: data: '${export_path}raw_dataframe.pkl'"
echo -n "${srv_string}"

ros2 service call /logger_node/export_data norlab_controllers_msgs/srv/ExportData "${srv_string}"

sleep 5

killall drive_node
killall logger_node

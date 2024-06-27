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
screen -S "records" -X stuff $'\003'

echo "Starting sensors"
#screen -dmS sensors ros2 launch norlab_robot sensors.launch.py
screen -dmS records ros2 launch warthog_mapping sensors.launch.xml 
echo "Sensors started, access it with screen -r sensors"
sleep 2

echo "Starting mapping"
#screen -dmS mapping ros2 launch norlab_robot mapping.launch.py
screen -dmS mapping ros2 launch warthog_mapping realtime_mapping.launch.xml 

echo "Mapping started, access it with screen -r mapping"
screen -dmS records ros2 launch
sleep 2

echo "Starting the record in the screen records"
#screen -dmS records ros2 launch norlab_robot rosbag_record.launch.py config:=nicolas_samson


sleep 2

echo "Launch drive in a screen name drive"
#screen -dmS drive ros2 launch drive warthog.launch.xml 
sleep 2 

echo "All screen has been started, click ENTER to enter the screen drive and monitor Drive.
   Once done, exit the screen by using (ctrl+a) then d."
read continue

screen -r drive

drive_done = 0
while [ drive_done = 1 ]
do
   echo "If you have finished DRIVE write 'finish' and click enter to continue to the training step:"
   read start_drive
   if [ $start_drive = 'finish' ]; then
      drive_done = 1
   sleep 30
fi
done



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

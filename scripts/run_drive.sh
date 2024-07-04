#!/bin/bash
test=2
echo -n "Enter experiment name, if empty the name will be the current date and time: "
read experiment_name
if [[ -z "$experiment_name" ]]; then
   experiment_name="$(date '+%Y-%m-%d-%s')"
fi


echo "Killing old screens"
screen -S "sensors" -X quit
screen -S "mapping" -X quit
screen -S "drive" -X quit
screen -S "records" -X quit
screen -S "theodolite" -X quit
screen -S "icp_visualization" -X quit




echo "Starting sensors"
screen -dmS sensors ros2 launch norlab_robot sensors.launch.py
#screen -dmS records ros2 launch warthog_mapping sensors.launch.xml 
echo "Sensors started, access it with screen -r sensors"
sleep 2

echo "Starting mapping"
screen -dmS mapping ros2 launch norlab_robot mapping.launch.py
#screen -dmS mapping ros2 launch warthog_mapping realtime_mapping.launch.xml 

echo "Mapping started, access it with screen -r mapping"
sleep 2

echo "Launch drive in a screen name drive"
screen -dmS drive ros2 launch drive warthog.launch.xml 
sleep 2 



echo "Starting theodolite"
screen -dmS theodolite ros2 launch theodolite_pose theodolite_pose.launch.py
echo "Theodolite started, access it with screen -r theodolite"

echo "Starting icp_visualization"
screen -dmS visualization ros2 launch theodolite_pose icp_pose.launch.py
echo "Visualization started, access it with screen -r icp_visualization"



echo "Starting the record in the screen records"
screen -dmS records ros2 launch norlab_robot rosbag_record.launch.py config:=nicolas_samson
sleep 2







echo "All screen have been started, open another terminal and enter the screen drive to monitor Drive.
    Once done, press enter."
read continue

#screen -r drive
screen -r records

drive_done=0
while [ "$drive_done" -eq 0 ]
do
   
   echo "If you have finished DRIVE write 'finish' and click enter to save the resulting dataset"
   read start_drive
   if [ $start_drive = "finish" ]; then drive_done=1; echo "$drive_done";fi
done


current_directory=$PWD
export_path=""$PWD"/../calib_data/"$experiment_name"/"
echo -n $export_path

if [ ! -d $export_path ]; then
  mkdir -p $export_path;
fi
ros2 service call /export_data norlab_controllers_msgs/srv/ExportData "export_path:
  data: '"$export_path"/raw_dataframe.pkl'"

sleep 5



killall drive_node
killall logger_node
#sleep 2
#read -p "When dataset gathering is done, press Enter to save"

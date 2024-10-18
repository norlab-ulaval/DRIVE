#!/bin/bash

# Check if a path argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path_to_rosbag>"
    exit 1
fi

# Define the path to the rosbag and the result folder
ROSBAG_PATH="$1"
BASENAME=$(basename "$ROSBAG_PATH")  # Get the base name without extension
RESULT_FOLDER="$(dirname "$ROSBAG_PATH")/${BASENAME}_without_mapping"
YAML_FILE="$RESULT_FOLDER/out.yaml"

# Create the result folder if it doesn't exist
mkdir -p "$RESULT_FOLDER"

# Write to the YAML file
cat <<EOL > "$YAML_FILE"
output_bags:
- uri: "$RESULT_FOLDER/${BASENAME}_to_remap"
  topics: [/vn100/vectornav/raw/ins,/vn100/vectornav/raw/imu,/vn100/vectornav/raw/gps2,/vn100/vectornav/raw/gps, /vn100/vectornav/raw/attitude, /left_drive/status/battery_current_corrected, /basler/driver/camera_info, /controller/ref_path, /drive/good_calib_step, /audio/audio, /audio/audio_info, /drive/imediate_path, /drive/operator_action_calibration, /vn100/data_unbiased, /vn100/vectornav/raw/time, /drive/calib_step, /basler/driver/status, /left_drive/velocity, /left_drive/status/battery_current, /mti30/mag, /vn100/time_gps, /warthog_velocity_controller/odom, /tf_static, /mti30/velocity, /vn100/temperature, /mti30/data_raw, /parameter_events, /vn100/vectornav/raw/common, /mti30/time_reference, /vn100/gnss, /vn100/time_startup, /vn100/data, /vn100/bias, /mti30/data, /mti30/bias, /mti30/imu_data_str, /rosout, /diagnostics, /right_drive/status/battery_voltage, /right_drive/status/speed, /robot_description, /vn100/data_raw, /left_drive/status/battery_voltage, /right_drive/status/battery_current_corrected, /rslidar128/points, /vn100/velocity_body, /vn100/time_syncin, /warthog_velocity_controller/cmd_vel, /vn100/time_pps, /vn100/magnetic, /drive/model_trainer_node_status, /doughnut_cmd_vel, /drive/calib_state, /vn100/pose, /drive/maestro_status, /vn100/pressure, /drive/path_to_reapeat, /basler/driver/image_raw, /controller/target_path]
  storage_id: "mcap"
EOL

echo "YAML file '$YAML_FILE' created successfully."


### Executing the filtering

source ~/ros2_ws/install/setup.bash 

ros2 bag convert -i $ROSBAG_PATH -o $YAML_FILE

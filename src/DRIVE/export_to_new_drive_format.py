import csv
import logging
import os
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from DRIVE.models import WARTHOG_MODEL

path = "../../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/raw_dataframe.pkl"
export_path = "../../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/new_drive_format"
step_duration_s = 6.0
expected_freq_hz = 20.0
nb_index_for_step = int(step_duration_s * expected_freq_hz)

# Columns: ['ros_time', 'joy_switch', 'icp_index',
#    'calib_state', 'calib_step',
#    'meas_left_vel', 'meas_right_vel',
#    'cmd_vel_x', 'cmd_vel_omega',
#    'icp_pos_x', 'icp_pos_y', 'icp_pos_z',
#    'icp_quat_x', 'icp_quat_y', 'icp_quat_z', 'icp_quat_w',
#    'imu_x', 'imu_y', 'imu_z',
#    'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z',
#    'left_wheel_voltage', 'right_wheel_voltage', 'left_wheel_current', 'right_wheel_current']

# calib_state: idle, drive_finished, calib

data = pd.read_pickle(path)

data["ros_time"] = data["ros_time"].astype("double")
data["calib_step"] = data["calib_step"].astype(float)

data["icp_pos_x"] = data["icp_pos_x"].astype(float)
data["icp_pos_y"] = data["icp_pos_y"].astype(float)
data["icp_pos_z"] = data["icp_pos_z"].astype(float)

data["icp_quat_x"] = data["icp_quat_x"].astype(float)
data["icp_quat_y"] = data["icp_quat_y"].astype(float)
data["icp_quat_z"] = data["icp_quat_z"].astype(float)
data["icp_quat_w"] = data["icp_quat_w"].astype(float)

data["imu_x"] = data["imu_x"].astype(float)
data["imu_y"] = data["imu_y"].astype(float)
data["imu_z"] = data["imu_z"].astype(float)
data["imu_acceleration_x"] = data["imu_acceleration_x"].astype(float)
data["imu_acceleration_y"] = data["imu_acceleration_y"].astype(float)
data["imu_acceleration_z"] = data["imu_acceleration_z"].astype(float)

data["cmd_vel_x"] = data["cmd_vel_x"].astype(float)
data["cmd_vel_omega"] = data["cmd_vel_omega"].astype(float)

data["meas_left_vel"] = data["meas_left_vel"].astype(float)
data["meas_right_vel"] = data["meas_right_vel"].astype(float)

data = data[data["calib_state"] == "calib"]

positions = []
accelerations = []
velocities = []
steps = []
encoders = []

model = WARTHOG_MODEL

for step_id, group in data.groupby("calib_step"):
    if len(group) < nb_index_for_step:
        logging.warning(
            f"Skipping step {step_id} due to insufficient data points. ({len(group)} < {nb_index_for_step})"
        )
        continue

    group = group.tail(nb_index_for_step)

    xs = group["icp_pos_x"].to_numpy()
    ys = group["icp_pos_y"].to_numpy()
    zs = group["icp_pos_z"].to_numpy()

    quats_x = group["icp_quat_x"].to_numpy()
    quats_y = group["icp_quat_y"].to_numpy()
    quats_z = group["icp_quat_z"].to_numpy()
    quats_w = group["icp_quat_w"].to_numpy()

    timestamps = group["ros_time"].to_numpy()

    imu_xs = group["imu_x"].to_numpy()
    imu_ys = group["imu_y"].to_numpy()
    imu_zs = group["imu_z"].to_numpy()
    imu_acc_xs = group["imu_acceleration_x"].to_numpy()
    imu_acc_ys = group["imu_acceleration_y"].to_numpy()
    imu_acc_zs = group["imu_acceleration_z"].to_numpy()

    cmd_vel_xs = group["cmd_vel_x"].to_numpy()
    cmd_vel_omegas = group["cmd_vel_omega"].to_numpy()

    left_wheel_velocities = group["meas_left_vel"].to_numpy()
    right_wheel_velocities = group["meas_right_vel"].to_numpy()

    cmd_vel_x = cmd_vel_xs[0]
    cmd_vel_omega = cmd_vel_omegas[0]
    start_ts = timestamps[0]
    end_ts = timestamps[-1]
    steps.append([step_id, start_ts, end_ts, cmd_vel_x, cmd_vel_omega, "completed"])

    for i in range(len(xs)):
        timestamp = timestamps[i]

        x = xs[i]
        y = ys[i]
        z = zs[i]

        quat_x = quats_x[i]
        quat_y = quats_y[i]
        quat_z = quats_z[i]
        quat_w = quats_w[i]

        imu_x = imu_xs[i]
        imu_y = imu_ys[i]
        imu_z = imu_zs[i]
        imu_acc_x = imu_acc_xs[i]
        imu_acc_y = imu_acc_ys[i]
        imu_acc_z = imu_acc_zs[i]

        roll, pitch, yaw = R.from_quat([quat_x, quat_y, quat_z, quat_w]).as_euler("xyz", degrees=True)

        left_angular_vel = left_wheel_velocities[i]
        right_angular_vel = right_wheel_velocities[i]

        dstate = model.forward_kinematics(left_angular_vel, right_angular_vel)
        vel_x = dstate[0]
        yaw_rate = dstate[2]

        positions.append([timestamp, step_id, x, y, z, roll, pitch, yaw])
        velocities.append([timestamp, step_id, vel_x, 0.0, 0.0, 0.0, 0.0, yaw_rate])
        accelerations.append([timestamp, step_id, imu_acc_x, imu_acc_y, imu_acc_z, imu_x, imu_y, imu_z])
        encoders.append([timestamp, step_id, left_angular_vel, right_angular_vel])

os.makedirs(export_path, exist_ok=True)

with open(os.path.join(export_path, "positions.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "step_id", "x", "y", "z", "roll", "pitch", "yaw"])
    writer.writerows(positions)

with open(os.path.join(export_path, "velocities.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "step_id", "speed_x", "speed_y", "speed_z", "speed_roll", "speed_pitch", "speed_yaw"])
    writer.writerows(velocities)

with open(os.path.join(export_path, "accelerations.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "step_id", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])
    writer.writerows(accelerations)

with open(os.path.join(export_path, "steps.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(
        [
            "id",
            "start_timestamp",
            "end_timestamp",
            "commanded_linear_velocity",
            "commanded_angular_velocity",
            "completion_status",
        ]
    )
    writer.writerows(steps)

with open(os.path.join(export_path, "encoders.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(
        [
            "timestamp",
            "step_id",
            "left_wheel_angular_velocity",
            "right_wheel_angular_velocity",
        ]
    )
    writer.writerows(encoders)

with open(os.path.join(export_path, "state_transitions.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "step_id", "from_state", "to_state"])

with open(os.path.join(export_path, "geofence.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])

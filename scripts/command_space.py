from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

from shapely.geometry import MultiPoint
from DRIVE.input_space import InputSpace
from DRIVE.models import WARTHOG_MODEL, IdealDiffDriveModel


def get_data_between_timestamps(
    dataframe: pd.DataFrame, timestamp_field_name: str, start_timestamp: float, end_timestamp: float
):
    timestamp = dataframe[timestamp_field_name]

    mask = (timestamp >= start_timestamp) & (timestamp <= end_timestamp)

    return dataframe.loc[mask].reset_index(drop=True)


data_dir = Path("../drive_datasets/warthog/micro_drive_grass")

left_wheel_vel = pd.read_csv(data_dir / "bag_extracted/left_wheel_vel/left_wheel_vel.csv")
left_wheel_cmd = pd.read_csv(data_dir / "bag_extracted/left_wheel_cmd/left_wheel_cmd.csv")
left_wheel_current = pd.read_csv(data_dir / "bag_extracted/left_wheel_current/left_wheel_current.csv")

right_wheel_vel = pd.read_csv(data_dir / "bag_extracted/right_wheel_vel/right_wheel_vel.csv")
right_wheel_cmd = pd.read_csv(data_dir / "bag_extracted/right_wheel_cmd/right_wheel_cmd.csv")
right_wheel_current = pd.read_csv(data_dir / "bag_extracted/right_wheel_current/right_wheel_current.csv")

steps = pd.read_csv(data_dir / "steps.csv")

completed_steps = steps[steps["completion_status"] == "completed"]

v_x = completed_steps["commanded_linear_velocity"]
omega_z = completed_steps["commanded_angular_velocity"]

encoders_left_vel = []
encoders_right_vel = []
encoders_body = []
command_wheel = []

for i, step in completed_steps.iterrows():

    end_timestamp = step["end_timestamp"]
    start_timestamp = end_timestamp - 2 * 1e9

    step_left_wheel_vel = get_data_between_timestamps(left_wheel_vel, "ros_time", start_timestamp, end_timestamp)
    step_right_wheel_vel = get_data_between_timestamps(right_wheel_vel, "ros_time", start_timestamp, end_timestamp)

    mean_left_vel = step_left_wheel_vel["data"].mean()
    mean_right_vel = step_right_wheel_vel["data"].mean()

    encoders_left_vel.append(mean_left_vel)
    encoders_right_vel.append(mean_right_vel)
    encoders_body.append(WARTHOG_MODEL.forward_kinematics(mean_left_vel, mean_right_vel))
    command_wheel.append(
        WARTHOG_MODEL.inverse_kinematics(step["commanded_linear_velocity"], step["commanded_angular_velocity"])
    )


encoders_left_vel = np.array(encoders_left_vel)
encoders_right_vel = np.array(encoders_right_vel)
encoders_body = np.array(encoders_body)
command_wheel = np.array(command_wheel)

min_lin_speed = -0.2
max_lin_speed = 0.2
min_ang_speed = -0.5
max_ang_speed = 0.5

min_wheel_speed = -13.3
max_wheel_speed = 13.3

input_space = InputSpace(
    WARTHOG_MODEL, min_lin_speed, max_lin_speed, min_ang_speed, max_ang_speed, min_wheel_speed, max_wheel_speed
)

fig, (ax_wheel, ax_body) = plt.subplots(2, 1, figsize=(10, 8))

# ======= Wheel Space =======
shape = input_space.actuator_input_space()
x, y = shape.exterior.xy
ax_wheel.plot(y, x, color="black", label="Experiment Input space")

# Encoders
multipoint = MultiPoint(np.column_stack((encoders_left_vel, encoders_right_vel)))
polygon = multipoint.convex_hull
x, y = polygon.exterior.xy
ax_wheel.fill(y, x, color="green", alpha=0.2)
ax_wheel.plot(y, x, color="green")
ax_wheel.scatter(encoders_right_vel, encoders_left_vel, color="green", label="Encoders")

# Commands
ax_wheel.scatter(command_wheel[:, 1], command_wheel[:, 0], color="gold", label="Commanded")

ax_wheel.set_xlabel("Right Wheel Velocity (rad/s)")
ax_wheel.set_ylabel("Left Wheel Velocity (rad/s)")
ax_wheel.set_title("Wheel Space")
ax_wheel.axis("equal")
ax_wheel.legend()

# ======= Body Space =======

shape = input_space.body_input_space()
x, y = shape.exterior.xy
ax_body.plot(y, x, color="black", label="Experiment Input space")

# Encoders
multipoint = MultiPoint(np.column_stack((encoders_body[:, 1], encoders_body[:, 0])))
polygon = multipoint.convex_hull
x, y = polygon.exterior.xy
ax_body.fill(x, y, color="blue", alpha=0.2)
ax_body.plot(x, y, color="blue")
ax_body.scatter(encoders_body[:, 1], encoders_body[:, 0], label="Encoders", color="blue")


# Commands
ax_body.scatter(omega_z, v_x, label="Commanded", color="gold")

ax_body.axis("equal")
ax_body.set_xlim(-1.0, 1.0)
ax_body.set_xlabel("Angular Velocity (rad/s)")
ax_body.set_ylabel("Linear Velocity (m/s)")
ax_body.set_title("Body Space")
ax_body.legend()

plt.tight_layout()
# plt.savefig("command_space.png")
plt.show()

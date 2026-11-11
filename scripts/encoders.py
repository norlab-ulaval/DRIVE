from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

from DRIVE.models import WARTHOG_MODEL


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

for i, step in completed_steps.iterrows():

    start_timestamp = step["start_timestamp"]
    end_timestamp = step["end_timestamp"]

    v_x = step["commanded_linear_velocity"]
    omega_z = step["commanded_angular_velocity"]

    u = WARTHOG_MODEL.inverse_kinematics(v_x, omega_z)

    u_left = u[0]
    u_right = u[1]

    delayed_start = start_timestamp - 2 * 1e9
    delayed_start_current = start_timestamp - 3 * 1e9  # Current data is only 1Hz

    step_left_wheel_vel = get_data_between_timestamps(left_wheel_vel, "ros_time", delayed_start, end_timestamp)
    step_left_wheel_cmd = get_data_between_timestamps(left_wheel_cmd, "ros_time", delayed_start, end_timestamp)
    step_left_wheel_current = get_data_between_timestamps(
        left_wheel_current, "ros_time", delayed_start_current, end_timestamp
    )

    step_right_wheel_vel = get_data_between_timestamps(right_wheel_vel, "ros_time", delayed_start, end_timestamp)
    step_right_wheel_cmd = get_data_between_timestamps(right_wheel_cmd, "ros_time", delayed_start, end_timestamp)
    step_right_wheel_current = get_data_between_timestamps(
        right_wheel_current, "ros_time", delayed_start_current, end_timestamp
    )

    fig, (ax_left, ax_right, ax_current) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax_left.plot((step_left_wheel_vel["ros_time"] - start_timestamp) / 1e9, step_left_wheel_vel["data"], "o-", color="g", label=f"Encoder")  # type: ignore
    ax_left.plot((step_left_wheel_cmd["ros_time"] - start_timestamp) / 1e9, step_left_wheel_cmd["data"], "--", color="orange", label=f"Commanded")  # type: ignore
    # ax_left.axhline(y=u_left, color='r', linestyle='--', label=f"IDD") # type: ignore
    ax_left.set_title("Left wheel velocity")
    ax_left.set_xlabel("Time since current step started (s)")
    ax_left.set_ylabel("Left wheel velocity (rad/s)")
    ax_left.set_xlim(-2.0, 6.0)
    ax_left.legend()

    ax_right.plot((step_right_wheel_vel["ros_time"] - start_timestamp) / 1e9, step_right_wheel_vel["data"], "o-b", label=f"Encoder")  # type: ignore
    ax_right.plot((step_right_wheel_cmd["ros_time"] - start_timestamp) / 1e9, step_right_wheel_cmd["data"], "--", color="orange", label=f"Commanded")  # type: ignore
    # ax_right.axhline(y=u_right, color='r', linestyle='--', label=f"IDD") # type: ignore
    ax_right.set_title("Right wheel velocity")
    ax_right.set_xlabel("Time since current step started (s)")
    ax_right.set_ylabel("Right wheel velocity (rad/s)")
    # ax_right.set_ylim(-0.6, 0.6)
    ax_right.legend()

    ax_current.plot((step_left_wheel_current["ros_time"] - start_timestamp) / 1e9, step_left_wheel_current["data"], "o-g", label="Left wheel current")  # type: ignore
    ax_current.plot((step_right_wheel_current["ros_time"] - start_timestamp) / 1e9, step_right_wheel_current["data"], "o-b", label="Right wheel current")  # type: ignore
    ax_current.set_ylim(0, 20)
    ax_current.set_title("Wheel current")
    ax_current.set_xlabel("Time since current step started (s)")
    ax_current.set_ylabel("Current (A)")
    ax_current.legend()

    for ax in [ax_left, ax_right, ax_current]:
        ax.tick_params(labelbottom=True)

    fig.suptitle(f"Step {i} ($v_x={v_x:.2f}$ m/s, $\\omega_z={omega_z:.2f}$ rad/s)")
    fig.tight_layout()
    fig.savefig(f"figs/encoders/step_{i}.png")
    plt.close(fig)

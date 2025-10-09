from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from motion_models.model import IdealDiffDrive, rk4, predict

def get_data_between_timestamps(
    dataframe: pd.DataFrame, timestamp_field_name: str, start_timestamp: float, end_timestamp: float
):
    timestamp = dataframe[timestamp_field_name]

    mask = (timestamp >= start_timestamp) & (timestamp <= end_timestamp)

    return dataframe.loc[mask].reset_index(drop=True)


data_dir = Path("../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/new_drive_format")
# data_dir = Path("../drive_datasets/warthog/micro_drive_grass")

positions = pd.read_csv(data_dir / "positions.csv")
steps = pd.read_csv(data_dir / "steps.csv")

completed_steps = steps[steps["completion_status"] == "completed"]

for i, step in completed_steps.iterrows():
    start_timestamp = step["start_timestamp"]
    end_timestamp = step["end_timestamp"]

    positions_icp = get_data_between_timestamps(positions, "timestamp", start_timestamp, end_timestamp)

    x_0 = positions_icp.iloc[0][["x", "y", "yaw"]].to_numpy().reshape((3,1))

    v = step["commanded_linear_velocity"]
    omega = step["commanded_angular_velocity"]
    u = np.array([v, omega])

    dt = 0.1
    N = int(6.0 / dt)

    u_arr = np.column_stack([u for _ in range(N)])

    model = IdealDiffDrive(wheelbase=1.08, wheel_radius=0.3)
    x_arr = predict(model, x_0, u_arr, dt, rk4)

    plt.scatter(positions_icp["x"], positions_icp["y"], label=f"step {i}")
    plt.scatter(x_arr[0, :], x_arr[1, :], label=f"model {i}", color='red')
    plt.scatter(x_0[0], x_0[1], color='green', label="start", s=100)
    plt.legend()
    plt.axis("equal")
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.show()
    

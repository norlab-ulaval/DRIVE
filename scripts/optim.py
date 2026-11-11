from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from motion_models.model import IdealDiffDrive, rk4, predict
from scipy.optimize import minimize

def get_data_between_timestamps(
    dataframe: pd.DataFrame, timestamp_field_name: str, start_timestamp: float, end_timestamp: float
):
    timestamp = dataframe[timestamp_field_name]

    mask = (timestamp >= start_timestamp) & (timestamp <= end_timestamp)

    return dataframe.loc[mask].reset_index(drop=True)


# data_dir = Path(
#     "../drive_datasets/old_drive/warthog/wheels/sand/warthog_wheels_sand_2024_9_25_14h51s17/model_training_datasets/new_drive_format"
# )
# data_dir = Path(
#     "../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h27s52/model_training_datasets/new_drive_format"
# )
data_dir = Path("../drive_datasets/warthog/micro_drive_grass")
# data_dir = Path("../drive_datasets/2025-10-09_16-05-04")

positions = pd.read_csv(data_dir / "positions.csv")
steps = pd.read_csv(data_dir / "steps.csv")

completed_steps = steps[steps["completion_status"] == "completed"]

nb_seconds = 2
dt = 0.1
nb_samples = int(nb_seconds / dt)
nb_steps = completed_steps.shape[0]

X_ref = np.zeros((nb_steps, 3, nb_samples))
U_ref = np.zeros((nb_steps, 2))
valid_idx = 0  # Counter for valid steps that we actually store

for i, step in completed_steps.iterrows():
    start_timestamp = step["end_timestamp"] - nb_seconds * 1e9
    end_timestamp = step["end_timestamp"]

    # Extract reference trajectory and initial state
    positions_icp = get_data_between_timestamps(positions, "timestamp", start_timestamp, end_timestamp)
    x_ref = positions_icp[["x", "y", "yaw"]].to_numpy().T
    x_0 = positions_icp.iloc[0][["x", "y", "yaw"]].to_numpy().reshape((3, 1))

    # Extract control inputs
    v = step["commanded_linear_velocity"]
    omega = step["commanded_angular_velocity"]
    body_u = np.array([v, omega])
    model = IdealDiffDrive(wheelbase=1.08, wheel_radius=0.3)

    u = (model.inv_J @ body_u.T).flatten()

    if abs(v) > 3 or abs(omega) > 3:
        print(f"Skipping step {i} with v={v:.2f} m/s, omega={omega:.2f} rad/s")
        continue

    if len(x_ref.T) < nb_samples:
        print(f"Skipping step {i} with only {len(x_ref.T)} samples")
        continue

    X_ref[valid_idx, :, :] = x_ref[:, :nb_samples] # type: ignore
    U_ref[valid_idx, :] = u # type: ignore
    valid_idx += 1

X_ref = X_ref[:valid_idx, :, :]
U_ref = U_ref[:valid_idx, :]

def idd(x_0, u_arr, theta):
    wheelbase, wheel_radius = theta
    current_model = IdealDiffDrive(wheelbase, wheel_radius)
    return predict(current_model, x_0, u_arr, dt, rk4)

def objective(theta):
    total_error = 0.0
    for i in range(valid_idx):  # Only iterate over valid entries
        x_ref = X_ref[i, :, :]
        x_0 = x_ref[:, 0].reshape((3, 1))  # Ensure proper shape

        u = U_ref[i, :]
        u_arr = np.column_stack([u for _ in range(nb_samples)])

        x_pred = idd(x_0, u_arr, theta)

        error = np.linalg.norm(x_ref - x_pred[:, 1:], axis=0)
        total_error += np.sum(error)
    
    return total_error


result = minimize(
    objective,
    x0=(1.0, 0.3),
    bounds=[(0.1, 3.0), (0.1, 3.0)],
    method="L-BFGS-B",
)

print(result.x)

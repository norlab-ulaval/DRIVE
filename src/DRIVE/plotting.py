import pathlib
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import logging

from DRIVE.analysis import DriveDataset, is_step_completed, read_dataset


def draw_geofence(ax, dataset: DriveDataset):
    if dataset.geofence.empty:
        logging.warning(f"Geofence is empty in this dataset")
        return

    ax.plot(dataset.geofence["x"], dataset.geofence["y"], color="#00f034")


def draw_step(ax, dataset: DriveDataset, step_id: int):
    pos_df = dataset.positions[dataset.positions["step_id"] == step_id]
    if pos_df.empty:
        logging.warning(f"Positions are empty for step {step_id}")
        return

    ax.plot(pos_df["x"], pos_df["y"], label=f"Step {step_id}")


def draw_predicted_step(ax, dataset: DriveDataset, step_id: int):
    step_df = dataset.steps[dataset.steps["id"] == step_id]
    if step_df.empty:
        logging.warning(f"Step {step_id} not found in dataset.")
        return

    pos_df = dataset.positions[dataset.positions["step_id"] == step_id]
    if pos_df.empty:
        logging.warning(f"Positions are empty for step {step_id}")
        return

    step = step_df.iloc[0]
    v_x = step["commanded_linear_velocity"]
    omega_z = step["commanded_angular_velocity"]
    start_t = step["start_timestamp"] / 1e9
    end_t = step["end_timestamp"] / 1e9

    x = pos_df["x"].iloc[0]
    y = pos_df["y"].iloc[0]
    yaw = pos_df["yaw"].iloc[0]
    dt = 1.0 / 20.0  # s
    t = start_t

    poses = []
    while t <= end_t:
        x += v_x * dt * np.cos(yaw)
        y += v_x * dt * np.sin(yaw)
        yaw += omega_z * dt

        poses.append([x, y, yaw])

        t += dt

    poses = np.array(poses)

    ax.plot(poses[:, 0], poses[:, 1], linestyle="--", color="gray", alpha=0.5)


if __name__ == "__main__":
    # path = "../../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h27s52/model_training_datasets/new_drive_format"
    path = "../../drive_datasets/2025-07-24_17-02-43"
    dataset = read_dataset(pathlib.Path(path))

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Geofence")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    draw_geofence(ax, dataset)

    for i in range(20):
        # if not is_step_completed(dataset, i):
        #     continue

        draw_step(ax, dataset, step_id=i)
        draw_predicted_step(ax, dataset, step_id=i)

    fig.tight_layout()
    plt.legend()
    plt.show()

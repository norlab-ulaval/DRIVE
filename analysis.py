import matplotlib
import numpy as np

matplotlib.use("TkAgg")

from pathlib import Path
from matplotlib import pyplot as plt
import pandas as pd

dataset_folder = Path("/home/ws/drive_datasets/2025-06-03_21-11-54_drive-test")
v_x_max = 0.5
v_omega_max = 1.0


geofence = pd.read_csv(dataset_folder / "geofence.csv")
positions = pd.read_csv(dataset_folder / "positions.csv")
state_transitions = pd.read_csv(dataset_folder / "state_transitions.csv")
steps = pd.read_csv(dataset_folder / "steps.csv")


def get_closest_from_timestamp(dataframe, target_timestamp: int):
    closest_index = np.argmin(abs(dataframe["timestamp"] - target_timestamp))
    return dataframe.iloc[closest_index]


if __name__ == "__main__":
    fig, axs = plt.subplots(2, 1)

    axs[0].plot(geofence["x"], geofence["y"], color="#00f034")

    nb_skipped_step = 0
    nb_step = 0
    for step_id, group in positions.groupby("step_id"):
        # -1 => Points before runnning state
        if step_id == -1:
            continue

        # No match in step, don't know why this is happening, need to investigate
        filtered = steps[steps["step_id"] == step_id]
        if filtered.empty:
            continue

        # Don't consider skipped step
        step = filtered.iloc[0]
        if not step["is_completed"]:
            nb_skipped_step += 1
            continue

        # We keep only positions after the start timestamp (Vehicle might come back to center when it goes off)
        start_timestamp = step["step_start_timestamp"]
        filtered_group = group[group["timestamp"] >= start_timestamp]

        axs[0].plot(filtered_group["x"], filtered_group["y"], label=f"Step {nb_step+1}")

        v = step["commanded_linear_velocity"]
        angular_v = step["commanded_angular_velocity"]
        axs[1].scatter(v, angular_v, label=f"Step {nb_step+1}")

        nb_step += 1

    axs[0].legend()
    axs[0].set_xlabel("x (m)")
    axs[0].set_ylabel("y (m)")
    axs[0].set_title("DRIVE Steps Trajectories")

    axs[1].legend()
    axs[1].set_xlim(-v_x_max, v_x_max)
    axs[1].set_xlabel("Linear Speed $v_x$ (m/s)")
    axs[1].set_ylim(-v_omega_max, v_omega_max)
    axs[1].set_ylabel("Angular Speed $\\omega_z$ (rad/s)")
    axs[1].set_title("Sampled Input Space")

    time_spent_in_each_state = {}
    last_state = state_transitions.iloc[0]["from_state"]
    last_timestamp = state_transitions.iloc[0]["timestamp"]
    for _, transition in state_transitions.iloc[1:].iterrows():
        state = transition["from_state"]
        timestamp = transition["timestamp"]

        diff = timestamp - last_timestamp

        if state not in time_spent_in_each_state:
            time_spent_in_each_state[state] = 0

        time_spent_in_each_state[state] += diff

        last_state = transition["from_state"]
        last_timestamp = transition["timestamp"]

    print(f"Nb steps: {nb_step}")
    print(f"Nb skipped steps: {nb_skipped_step}")
    print(f"Total time: {sum(time_spent_in_each_state.values())/1e9:.1f}s")
    print("=========Time spent in each state=========")
    for k, v in time_spent_in_each_state.items():
        print(f"- {k}: {v/1e9:.1f}s")

    plt.show()

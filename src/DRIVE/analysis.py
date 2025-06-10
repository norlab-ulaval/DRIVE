from dataclasses import dataclass
import pathlib

from matplotlib import pyplot as plt
import pandas as pd


@dataclass
class DriveDataset:
    dataset_folder: pathlib.Path
    geofence: pd.DataFrame
    positions: pd.DataFrame
    state_transitions: pd.DataFrame
    steps: pd.DataFrame


def read_dataset(dataset_folder: pathlib.Path) -> DriveDataset:
    geofence = pd.read_csv(dataset_folder / "geofence.csv")
    positions = pd.read_csv(dataset_folder / "positions.csv")
    state_transitions = pd.read_csv(dataset_folder / "state_transitions.csv")
    steps = pd.read_csv(dataset_folder / "steps.csv")

    return DriveDataset(dataset_folder, geofence, positions, state_transitions, steps)


def generate_overview_visualization(dataset: DriveDataset):
    fig_folder = dataset.dataset_folder / "figs"
    fig_folder.mkdir(exist_ok=True)

    steps_per_fig = 10
    v_x_max = 0.5
    v_omega_max = 1.0

    nb_step = 0
    nb_skipped_step = 0
    step_groups = []
    current_group = []
    v_arr = []
    angular_v_arr = []
    for step_id, grouped_pos in dataset.positions.groupby("step_id"):
        # -1 => Points before runnning state
        if step_id == -1:
            continue

        # No match in step, don't know why this is happening, need to investigate
        filtered = dataset.steps[dataset.steps["id"] == step_id]
        if filtered.empty:
            continue

        # Consider only completed step
        step = filtered.iloc[0]
        if step["completion_status"] == "skipped":
            nb_skipped_step += 1
            continue
        elif step["completion_status"] != "completed":
            continue

        v = step["commanded_linear_velocity"]
        angular_v = step["commanded_angular_velocity"]
        v_arr.append(v)
        angular_v_arr.append(angular_v)

        if len(current_group) >= steps_per_fig:
            step_groups.append(current_group)
            current_group = []

        current_group.append(grouped_pos)

    if len(current_group) > 0:
        step_groups.append(current_group)

    # Plotting trajectories overviews with only 10 steps per figures
    nb_step = 0
    for step_group in step_groups:
        plt.plot(dataset.geofence["x"], dataset.geofence["y"], color="#00f034")

        for step_pos in step_group:
            plt.plot(step_pos["x"], step_pos["y"], label=f"Step {nb_step+1}")
            nb_step += 1

        plt.legend()
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("DRIVE Steps Trajectories")

        nb_in_group = len(step_group)
        plt.savefig(fig_folder / f"overview_steps_{nb_step-nb_in_group+1}-{nb_step}")
        plt.close()

    # Plotting sampled input space
    plt.scatter(v_arr, angular_v_arr)
    plt.xlim(-v_x_max, v_x_max)
    plt.xlabel("Linear Speed $v_x$ (m/s)")
    plt.ylim(-v_omega_max, v_omega_max)
    plt.ylabel("Angular Speed $\\omega_z$ (rad/s)")
    plt.title("Sampled Input Space")
    plt.savefig(fig_folder / f"sampled_input_space")

    # Computing time spent in each state
    time_spent_in_each_state = {}
    last_state = dataset.state_transitions.iloc[0]["from_state"]
    last_timestamp = dataset.state_transitions.iloc[0]["timestamp"]
    for _, transition in dataset.state_transitions.iloc[1:].iterrows():
        state = transition["from_state"]
        timestamp = transition["timestamp"]

        diff = timestamp - last_timestamp

        if state not in time_spent_in_each_state:
            time_spent_in_each_state[state] = 0

        time_spent_in_each_state[state] += diff

        last_state = transition["from_state"]
        last_timestamp = transition["timestamp"]

    summary_str = ""
    summary_str += f"Nb steps: {nb_step}\n"
    summary_str += f"Nb skipped steps: {nb_skipped_step}\n"
    summary_str += f"Total time: {sum(time_spent_in_each_state.values())/1e9:.1f}s\n"
    summary_str += "=========Time spent in each state=========\n"
    for k, v in time_spent_in_each_state.items():
        summary_str += f"- {k}: {v/1e9:.1f}s\n"

    with open(dataset.dataset_folder / "overview.txt", "w") as f:
        f.write(summary_str)


if __name__ == "__main__":
    dataset = read_dataset(pathlib.Path("/home/ws/drive_datasets/2025-06-10_18-28-17"))
    generate_overview_visualization(dataset)

from dataclasses import dataclass
import pathlib

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import shutil

from DRIVE.writing import Acceleration6DOF, DriveStep, GeofencePoint, Position6DOF, StateTransition


@dataclass
class DriveDataset:
    dataset_folder: pathlib.Path
    geofence: pd.DataFrame
    positions: pd.DataFrame
    accelerations: pd.DataFrame
    state_transitions: pd.DataFrame
    steps: pd.DataFrame


def read_dataset(dataset_folder: pathlib.Path) -> DriveDataset:
    geofence_path = dataset_folder / "geofence.csv"
    if not geofence_path.exists():
        geofence = pd.DataFrame(columns=[GeofencePoint.fields()])
    else:
        geofence = pd.read_csv(geofence_path)

    positions_path = dataset_folder / "positions.csv"
    if not positions_path.exists():
        positions = pd.DataFrame(columns=[Position6DOF.fields()])
    else:
        positions = pd.read_csv(positions_path)

    accelerations_path = dataset_folder / "accelerations.csv"
    if not accelerations_path.exists():
        accelerations = pd.DataFrame(columns=[Acceleration6DOF.fields()])
    else:
        accelerations = pd.read_csv(accelerations_path)

    state_transitions_path = dataset_folder / "state_transitions.csv"
    if not state_transitions_path.exists():
        state_transitions = pd.DataFrame(columns=[StateTransition.fields()])
    else:
        state_transitions = pd.read_csv(state_transitions_path)

    steps_path = dataset_folder / "steps.csv"
    if not steps_path.exists():
        steps = pd.DataFrame(columns=[DriveStep.fields()])
    else:
        steps = pd.read_csv(steps_path)

    return DriveDataset(dataset_folder, geofence, positions, accelerations, state_transitions, steps)


def is_step_completed(dataset: DriveDataset, step_id: int) -> bool:
    filtered = dataset.steps[dataset.steps["id"] == step_id]
    if filtered.empty:
        return False

    step = filtered.iloc[0]

    return step["completion_status"] == "completed"


def generate_overview_visualization(dataset: DriveDataset):
    fig_folder = dataset.dataset_folder / "figs"
    if fig_folder.exists() and fig_folder.is_dir():
        shutil.rmtree(fig_folder)
    fig_folder.mkdir(exist_ok=True)

    steps_per_fig = 10

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
    plt.close("all")
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
    plt.scatter(angular_v_arr, v_arr)
    plt.xlabel("Angular Speed $\\omega_z$ (rad/s)")
    plt.ylabel("Linear Speed $v_x$ (m/s)")
    plt.title("Sampled Input Space")
    plt.savefig(fig_folder / f"sampled_input_space")

    # Computing time spent in each state
    if not dataset.state_transitions.empty:
        time_spent_in_each_state = {}
        last_timestamp = dataset.state_transitions.iloc[0]["timestamp"]
        for _, transition in dataset.state_transitions.iloc[1:].iterrows():
            state = transition["from_state"]
            timestamp = transition["timestamp"]

            diff = timestamp - last_timestamp

            if state not in time_spent_in_each_state:
                time_spent_in_each_state[state] = 0

            time_spent_in_each_state[state] += diff

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


def generate_gg_diag():
    fig_folder = dataset.dataset_folder / "figs"
    fig_folder.mkdir(exist_ok=True)

    plt.close("all")
    accs_long = []
    accs_lat = []
    step_ids = []

    # Gather acceleration and step IDs
    for step_id, group in dataset.accelerations.groupby("step_id"):
        acc_long = group["acc_x"].to_numpy()
        acc_lat = group["acc_y"].to_numpy()
        n = len(acc_long)

        accs_long.extend(acc_long)
        accs_lat.extend(acc_lat)
        step_ids.extend([step_id] * n)

    accs_long = np.array(accs_long)
    accs_lat = np.array(accs_lat)
    step_ids = np.array(step_ids)

    # Build figure with two subplots
    fig, (ax_gg, ax_input) = plt.subplots(1, 2, figsize=(12, 6))

    # ---- GG Plot Setup ----
    ax_gg.set_title("GG Diagram")
    ax_gg.set_xlabel("Lateral Acceleration (m/s²)")
    ax_gg.set_ylabel("Longitudinal Acceleration (m/s²)")
    ax_gg.grid(True)
    max_range = 8
    ax_gg.set_xlim(-max_range, max_range)
    ax_gg.set_ylim(-max_range, max_range)
    ax_gg.set_aspect("equal")
    ax_gg.scatter(accs_lat, accs_long, s=20, alpha=0.05, color="gray")
    (point_gg,) = ax_gg.plot([], [], "ro", markersize=5)
    trail_segments = []
    max_trail_length = 20
    step_text = ax_gg.text(0.02, 0.95, "", transform=ax_gg.transAxes, fontsize=12, color="black")

    # ---- Input Space Plot Setup ----
    ax_input.set_title("Commanded Input Space")
    ax_input.set_xlabel("Angular Velocity (rad/s)")
    ax_input.set_ylabel("Linear Velocity (m/s)")
    ax_input.grid(True)

    # Preload step command data
    step_cmds = dataset.steps.set_index("id")[["commanded_linear_velocity", "commanded_angular_velocity"]]
    all_lin_vels = step_cmds["commanded_linear_velocity"].to_numpy()
    all_ang_vels = step_cmds["commanded_angular_velocity"].to_numpy()
    ax_input.scatter(all_ang_vels, all_lin_vels, s=10, alpha=0.3, color="gray")

    # Active command dot
    (command_dot,) = ax_input.plot([], [], "bo", markersize=10, label="Current Command")
    command_trail_segments = []

    def init():
        point_gg.set_data([], [])
        command_dot.set_data([], [])
        step_text.set_text("")
        return [point_gg, command_dot, step_text]

    def update(frame):
        step_id = step_ids[frame]

        # --- GG diagram animation ---
        point_gg.set_data([accs_lat[frame]], [accs_long[frame]])

        # Remove old trail segments
        for segment in trail_segments:
            segment.remove()
        trail_segments.clear()

        start = max(0, frame - max_trail_length)
        for i in range(start, frame):
            alpha = (i - start + 1) / (frame - start + 1)
            seg = ax_gg.plot(
                [accs_lat[i], accs_lat[i + 1]],
                [accs_long[i], accs_long[i + 1]],
                color="red",
                alpha=alpha,
                linewidth=1,
                label="Current Acceleration",
            )[0]
            trail_segments.append(seg)

        # --- Input space update ---
        command_dot.set_data([], [])
        for seg in command_trail_segments:
            seg.remove()
        command_trail_segments.clear()

        if step_id in step_cmds.index:
            step_text.set_text(f"Step ID: {int(step_id)}")
            lin = step_cmds.loc[step_id, "commanded_linear_velocity"]
            ang = step_cmds.loc[step_id, "commanded_angular_velocity"]
            command_dot.set_data([ang], [lin])

            for i in range(start, frame):
                s0 = step_ids[i]
                s1 = step_ids[i + 1]
                if s0 in step_cmds.index and s1 in step_cmds.index:
                    lin0 = step_cmds.loc[s0, "commanded_linear_velocity"]
                    ang0 = step_cmds.loc[s0, "commanded_angular_velocity"]
                    lin1 = step_cmds.loc[s1, "commanded_linear_velocity"]
                    ang1 = step_cmds.loc[s1, "commanded_angular_velocity"]
                    alpha = (i - start + 1) / (frame - start + 1)
                    seg = ax_input.plot(
                        [ang0, ang1],
                        [lin0, lin1],
                        color="blue",
                        alpha=alpha,
                        linewidth=1,
                    )[0]
                    command_trail_segments.append(seg)
        else:
            step_text.set_text("")

        return [point_gg, command_dot, step_text] + trail_segments + command_trail_segments

    ani = FuncAnimation(
        fig,
        update,
        frames=len(accs_lat),
        init_func=init,
        blit=False,
        interval=50,
    )

    ax_input.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = "../../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h27s52/model_training_datasets/new_drive_format"
    dataset = read_dataset(pathlib.Path(path))
    # generate_overview_visualization(dataset)
    generate_gg_diag()
    plt.show()

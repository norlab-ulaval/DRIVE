from dataclasses import dataclass
from email import encoders
from json import encoder
import logging
import pathlib

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import shutil

from DRIVE.models import WARTHOG_MODEL
from DRIVE.writing import (
    Acceleration6DOF,
    DriveStep,
    EncoderData,
    GeofencePoint,
    Position6DOF,
    Speed6DOF,
    StateTransition,
)


@dataclass
class DriveDataset:
    dataset_folder: pathlib.Path
    geofence: pd.DataFrame
    positions: pd.DataFrame
    velocities: pd.DataFrame
    accelerations: pd.DataFrame
    state_transitions: pd.DataFrame
    steps: pd.DataFrame
    encoders: pd.DataFrame


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

    velocities_path = dataset_folder / "velocities.csv"
    if not velocities_path.exists():
        velocities = pd.DataFrame(columns=[Speed6DOF.fields()])
    else:
        velocities = pd.read_csv(velocities_path)

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

    encoders_path = dataset_folder / "steps.csv"
    if not encoders_path.exists():
        steps = pd.DataFrame(columns=[DriveStep.fields()])
    else:
        steps = pd.read_csv(encoders_path)

    encoders_path = dataset_folder / "encoders.csv"
    if not encoders_path.exists():
        encoders = pd.DataFrame(columns=[EncoderData.fields()])
    else:
        encoders = pd.read_csv(encoders_path)

    return DriveDataset(
        dataset_folder, geofence, positions, velocities, accelerations, state_transitions, steps, encoders
    )


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
    measured_vels_x = []
    measured_vels_yaw = []
    encoders_left = []
    encoders_right = []

    nb_steps = 20
    step_length = 120  # 20Hz for 6 seconds

    model = WARTHOG_MODEL

    # Gather acceleration and step IDs
    for step_id, group in dataset.accelerations[: nb_steps * step_length].groupby("step_id"):
        vel_df = dataset.velocities[dataset.velocities["step_id"] == step_id]
        if vel_df.empty:
            logging.warning(f"Step {step_id} has no velocity data, skipping.")
            continue

        encoders_df = dataset.encoders[dataset.encoders["step_id"] == step_id]
        if encoders_df.empty:
            logging.warning(f"Step {step_id} has no encoders data, skipping.")
            continue

        n = (step_length // 3) * 2

        v_x = vel_df["speed_x"].to_numpy()[:n]
        v_yaw = vel_df["speed_yaw"].to_numpy()[:n]

        acc_long = group["acc_x"].to_numpy()[:n]
        acc_lat = group["acc_y"].to_numpy()[:n]

        left_wheel = encoders_df["left_wheel_angular_velocity"].to_numpy()[:n]
        right_wheel = encoders_df["right_wheel_angular_velocity"].to_numpy()[:n]

        measured_vels_x.extend(v_x)
        measured_vels_yaw.extend(v_yaw)
        accs_long.extend(acc_long)
        accs_lat.extend(acc_lat)
        step_ids.extend([step_id] * n)
        encoders_left.extend(left_wheel)
        encoders_right.extend(right_wheel)

    accs_long = np.array(accs_long)
    accs_lat = np.array(accs_lat)
    step_ids = np.array(step_ids)
    encoders_left = np.array(encoders_left)
    encoders_right = np.array(encoders_right)

    # Build figure with two subplots
    fig, (ax_gg, ax_body_space, ax_wheel_space) = plt.subplots(1, 3, figsize=(12, 10))

    # ---- GG Plot Setup ----
    ax_gg.set_title("GG Diagram")
    ax_gg.set_xlabel("Lateral Acceleration (m/s²)")
    ax_gg.set_ylabel("Longitudinal Acceleration (m/s²)")
    ax_gg.grid(True)
    max_range = 8
    ax_gg.set_xlim(-max_range, max_range)
    ax_gg.set_ylim(-max_range, max_range)
    ax_gg.set_aspect("equal")
    ax_gg.scatter(accs_lat, accs_long, s=20, alpha=0.1, color="gray")
    (point_gg,) = ax_gg.plot([], [], "ro", markersize=5)
    trail_segments = []
    max_trail_length = 20
    step_text = ax_gg.text(0.02, 0.95, "", transform=ax_gg.transAxes, fontsize=12, color="black")

    # ---- Body Space Plot Setup ----
    ax_body_space.set_title("Body Input Space")
    ax_body_space.set_xlabel("Angular Velocity (rad/s)")
    ax_body_space.set_ylabel("Linear Velocity (m/s)")
    ax_body_space.set_xlim(-9, 9)
    ax_body_space.set_ylim(-9, 9)
    ax_body_space.set_aspect("equal")
    ax_body_space.grid(True)

    # Preload step command data
    step_cmds = dataset.steps.set_index("id")[["commanded_linear_velocity", "commanded_angular_velocity"]]
    all_lin_vels = step_cmds["commanded_linear_velocity"].to_numpy()
    all_ang_vels = step_cmds["commanded_angular_velocity"].to_numpy()
    ax_body_space.scatter(all_ang_vels[:nb_steps], all_lin_vels[:nb_steps], s=10, alpha=0.3, color="gray")

    # Active command dot
    (command_dot,) = ax_body_space.plot([], [], "bo", markersize=10, label="Current Command")
    command_trail_segments = []

    # Measured vel dot
    (measured_dot,) = ax_body_space.plot([], [], "o", markersize=10, color="green", label="Measured Velocity")

    # ---- Wheel Space Plot Setup ----
    ax_wheel_space.set_title("Wheel Input Space")
    ax_wheel_space.set_xlabel("Left Wheel Velocity (m/s)")
    ax_wheel_space.set_ylabel("Right Wheel Velocity (m/s)")
    ax_wheel_space.set_xlim(-9, 9)
    ax_wheel_space.set_ylim(-9, 9)
    ax_wheel_space.set_aspect("equal")
    ax_wheel_space.grid(True)

    # Active command dot
    (command_wheel_dot,) = ax_wheel_space.plot([], [], "bo", markersize=10, label="Current Command")

    # Measured vel dot
    (measured_wheel_dot,) = ax_wheel_space.plot([], [], "o", markersize=10, color="green", label="Measured Velocity")

    def init():
        point_gg.set_data([], [])
        command_dot.set_data([], [])
        measured_dot.set_data([], [])
        measured_wheel_dot.set_data([], [])
        command_wheel_dot.set_data([], [])
        step_text.set_text("")
        return [point_gg, command_dot, measured_dot, command_wheel_dot, measured_wheel_dot, step_text]

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

        # --- Body space update ---
        command_dot.set_data([], [])
        for seg in command_trail_segments:
            seg.remove()
        command_trail_segments.clear()

        if step_id in step_cmds.index:
            step_text.set_text(f"Step ID: {int(step_id)}")
            lin = step_cmds.loc[step_id, "commanded_linear_velocity"]
            ang = step_cmds.loc[step_id, "commanded_angular_velocity"]
            command_dot.set_data([ang], [lin])

            measured_v_x = measured_vels_x[frame]
            measured_v_yaw = measured_vels_yaw[frame]
            measured_dot.set_data([measured_v_yaw], [measured_v_x])

            U = model.inverse_kinematics(lin, ang)
            commanded_left_vel = U[0] * model.wheel_radius
            commanded_right_vel = U[1] * model.wheel_radius
            command_wheel_dot.set_data([commanded_left_vel], [commanded_right_vel])

            measured_left_vel = encoders_left[frame] * model.wheel_radius
            measured_right_vel = encoders_right[frame] * model.wheel_radius
            measured_wheel_dot.set_data([measured_left_vel], [measured_right_vel])

            for i in range(start, frame):
                s0 = step_ids[i]
                s1 = step_ids[i + 1]
                if s0 in step_cmds.index and s1 in step_cmds.index:
                    lin0 = step_cmds.loc[s0, "commanded_linear_velocity"]
                    ang0 = step_cmds.loc[s0, "commanded_angular_velocity"]
                    lin1 = step_cmds.loc[s1, "commanded_linear_velocity"]
                    ang1 = step_cmds.loc[s1, "commanded_angular_velocity"]
                    alpha = (i - start + 1) / (frame - start + 1)
                    seg = ax_body_space.plot(
                        [ang0, ang1],
                        [lin0, lin1],
                        color="blue",
                        alpha=alpha,
                        linewidth=1,
                    )[0]
                    command_trail_segments.append(seg)
        else:
            step_text.set_text("")

        return (
            [point_gg, command_dot, measured_dot, command_wheel_dot, measured_wheel_dot, step_text]
            + trail_segments
            + command_trail_segments
        )

    ani = FuncAnimation(
        fig,
        update,
        frames=len(accs_lat),
        init_func=init,
        blit=False,
        interval=50,
    )

    ax_body_space.legend()
    plt.tight_layout()
    plt.show()
    plt.close(fig)
    # ani.save(fig_folder / "gg_input_animation.gif", writer="pillow", fps=20)


if __name__ == "__main__":
    path = "../../drive_datasets/old_drive/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/new_drive_format"
    dataset = read_dataset(pathlib.Path(path))
    # generate_overview_visualization(dataset)
    generate_gg_diag()

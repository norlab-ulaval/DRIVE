import logging
from pathlib import Path
import re
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal

import numpy as np

from DRIVE.analysis import generate_overview_visualization, read_dataset
from DRIVE.common import Command, Pose
from DRIVE.dataset_recorder import DatasetRecorder
from DRIVE.geofencing import Geofence
from DRIVE.robot import Robot
from DRIVE.sampling import CommandSamplingStrategy
from DRIVE.writing import Acceleration6DOF, DriveStep, GeofencePoint, Position6DOF, Speed6DOF, StateTransition

StepCompletionStatus = Literal["completed", "skipped", "restarted"]


@dataclass
class Step:
    id: int
    command: Command
    start_timestamp_ns: int
    start_pose: Pose


class IllegalStateTransition(Exception):
    def __init__(self, current_state: str, action: str):
        super().__init__(f"Cannot perform {action} from current state ({current_state})")


class DriveState(ABC):
    def __init__(self, drive):
        self.drive = drive

    @abstractmethod
    def run(self, timestamp_ns: int):
        pass

    def get_state_name(self) -> str:
        name = self.__class__.__name__
        # Convert CamelCase to snake_case
        s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
        snake_case = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
        return snake_case


class WaitingState(DriveState):
    def run(self, timestamp_ns: int):
        pass


class GeofenceCreationState(DriveState):
    def __init__(self, drive):
        super().__init__(drive)

        self.geofence_points: list[np.ndarray] = [self.drive.robot.pose[:2]]
        self.distance_thresold_meters = 0.5

    def run(self, timestamp_ns: int):
        last_point = self.geofence_points[-1][:2]
        current_point = self.drive.robot.pose[:2]

        if np.linalg.norm(current_point - last_point) > self.distance_thresold_meters:
            self.geofence_points.append(current_point)


class ReadyState(DriveState):
    def run(self, timestamp_ns: int):
        pass


class RunningState(DriveState):
    def __init__(self, drive, timestamp_ns: int):
        super().__init__(drive)

    def run(self, timestamp_ns: int):
        if len(self.drive.completed_commands) >= self.drive.target_nb_steps:
            logging.info("Target number of steps reached, stopping drive")
            self.drive.stop_drive(timestamp_ns)
            return

        if not self.drive.is_robot_inside_geofence():
            self.drive.go_back_inside_geofence(timestamp_ns)
            return

        if not self.drive.robot.deadman_switch_pressed:
            logging.info("Deadman switch not pressed, pausing drive")
            self.drive.pause_drive(timestamp_ns)
            return

        current_step: Step = self.drive.current_step

        if timestamp_ns - current_step.start_timestamp_ns > self.drive.step_duration_s * 1e9:
            self.drive.sample_next_step(timestamp_ns, "completed")
            return

        self.drive.robot.send_command(current_step.command)


class PausedState(DriveState):
    def run(self, timestamp_ns: int):
        if self.drive.robot.deadman_switch_pressed:
            logging.info("Deadman switch pressed, resuming drive")
            self.drive.resume_drive(timestamp_ns)
            return


class BackToGeofenceState(DriveState):
    def __init__(self, drive, timestamp_ns: int):
        super().__init__(drive)

        self.waiting_for_goal = False

    def run(self, timestamp_ns: int):
        if not self.waiting_for_goal:
            geofence = self.drive.geofence
            goal_pose: Pose = np.array([geofence.origin[0], geofence.origin[1], 0, 0, 0, 0])

            logging.info(f"Sending goal {goal_pose} to robot")
            self.waiting_for_goal = True
            self.drive.robot.send_goal(goal_pose)
            return

        if self.drive.robot.goal_reached:
            logging.info("Goal reached, resuming drive")
            self.drive.resume_drive(timestamp_ns)
            return


class Drive:
    """
    The Drive class is the main control loop for the DRIVE protocol. It manages the high-level state machine that
    governs data collection, geofence creation and safety monitoring during autonomous drive sessions. It uses
    a robot interface, a command sampling strategy, and a dataset recorder to execute and log drive sessions.
    The 'run' method should be called in a main loop to execute the appropriate actions based on the current state.
    """

    def __init__(
        self,
        robot: Robot,
        command_sampling_strategy: CommandSamplingStrategy,
        target_nb_steps: int,
        step_duration_s: float,
        dataset_directory: Path,
    ):
        self.robot = robot
        self.command_sampling_strategy = command_sampling_strategy
        self.target_nb_steps = target_nb_steps
        self.step_duration_s = step_duration_s

        self.next_step_id = 1
        self.current_state = WaitingState(self)
        self.geofence: None | Geofence = None
        self.current_step: None | Step = None
        self.dataset_recorder: DatasetRecorder = DatasetRecorder(dataset_directory)
        self.completed_commands = []

    def _transition_to_new_state(self, new_state: DriveState, timestamp_ns: int):
        state_transition = StateTransition(
            int(timestamp_ns), 0, self.current_state.get_state_name(), new_state.get_state_name()
        )

        self.dataset_recorder.append(state_transition)

        self.current_state = new_state

    def run(self, timestamp_ns: int):
        self.current_state.run(timestamp_ns)

        # Saving recorded data
        step_id = self.current_step.id if self.current_step is not None else -1

        last_poses = self.robot.poses_buffer
        last_speeds = self.robot.speeds_buffer
        last_accelerations = self.robot.accelerations_buffer

        self.dataset_recorder.append_multiple(
            [Position6DOF(timestamp, step_id, x[0], x[1], x[2], x[3], x[4], x[5]) for x, timestamp in last_poses]
        )
        self.dataset_recorder.append_multiple(
            [Speed6DOF(timestamp, step_id, x[0], x[1], x[2], x[3], x[4], x[5]) for x, timestamp in last_speeds]
        )
        self.dataset_recorder.append_multiple(
            [
                Acceleration6DOF(timestamp, step_id, x[0], x[1], x[2], x[3], x[4], x[5])
                for x, timestamp in last_accelerations
            ]
        )

        self.robot.empty_buffers()

    def start_step(self, timestamp_ns: int, command: Command):
        next_step_id = self.next_step_id
        self.next_step_id += 1

        self.current_step = Step(next_step_id, command, timestamp_ns, self.robot.pose)

    def save_step(self, end_timestamp_ns: int, step_completion_status: StepCompletionStatus):
        if self.current_step is not None:
            step = DriveStep(
                self.current_step.id,
                self.current_step.start_timestamp_ns,
                end_timestamp_ns,
                self.current_step.command[0],
                self.current_step.command[1],
                step_completion_status,
            )
            self.dataset_recorder.append(step)

            if step_completion_status == "completed":
                self.completed_commands.append(self.current_step.command)

    def sample_next_step(self, timestamp_ns: int, step_completion_status: StepCompletionStatus):
        self.save_step(timestamp_ns, step_completion_status)

        command = self.command_sampling_strategy.sample_command()
        self.start_step(timestamp_ns, command)

        logging.info(f"Sampling next command {command} at timestamp {timestamp_ns}")

    def restart_current_step(self, timestamp_ns: int, step_completion_status: StepCompletionStatus):
        if self.current_step is not None:
            self.save_step(timestamp_ns, step_completion_status)

            self.start_step(timestamp_ns, self.current_step.command)

            logging.info(f"Restarting command {self.current_step.command} at timestamp {timestamp_ns}")

    def get_help_message(self) -> str:
        if self.current_state.__class__ == WaitingState:
            return "Waiting: DRIVE node is started. Click next to start the geofence creation"
        elif self.current_state.__class__ == GeofenceCreationState:
            return "Geofence Creation: Drive the robot around manually to draw the geofence. Click next to end the geofence creation"
        elif self.current_state.__class__ == ReadyState:
            return "Ready: Geofence has been created. Move the robot inside the geofence and click next to start sampling commands"
        elif self.current_state.__class__ == RunningState:
            return "Running: Robot is currently sampling and executing commands"
        elif self.current_state.__class__ == PausedState:
            return "Paused: Press the deadman switch for the robot to continue sampling and executing commands"
        elif self.current_state.__class__ == BackToGeofenceState:
            return "Driving back to geofence: Robot ran off the geofence. Drive it manually to the current goal and click next to resume"

        return "Unknown state"

    def is_robot_inside_geofence(self) -> bool:
        if self.geofence is None:
            return True

        current_point = self.robot.pose[:2]
        return self.geofence.is_point_inside(current_point)

    def skip_current_step(self, timestamp_ns: int):
        logging.info("Skipping command...")
        self.sample_next_step(timestamp_ns, step_completion_status="skipped")

    def get_geofence_points(self) -> np.ndarray:
        if self.current_state.__class__ == GeofenceCreationState:
            return np.array(self.current_state.geofence_points)  # type: ignore
        elif self.geofence is not None:
            return np.array([np.array(point) for point in self.geofence.points])

        return np.array([])

    # ============================================ State transitions ============================================
    def start_geofence(self, timestamp_ns: int):
        if self.current_state.__class__ == WaitingState:
            logging.info(f"Starting geofence creation at timestamp {timestamp_ns}")
            self._transition_to_new_state(GeofenceCreationState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_geofence")

    def restart_geofence(self, timestamp_ns: int):
        if self.current_state.__class__ == GeofenceCreationState:
            logging.info(f"Restarting geofence creation at timestamp {timestamp_ns}")
            self._transition_to_new_state(GeofenceCreationState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "restart_geofence")

    def confirm_geofence(self, timestamp_ns: int):
        if self.current_state.__class__ in (GeofenceCreationState, WaitingState):
            if len(self.current_state.geofence_points) < 5:  # type: ignore
                logging.warning("A geofence need to have more than 5 points before being confirmed")
                return

            logging.info(f"Confirmed geofence at timestamp {timestamp_ns}")

            try:
                self.geofence = Geofence(self.current_state.geofence_points)  # type: ignore
            except ValueError as e:
                logging.error(f"Failed to create geofence: {e}")
                self.restart_geofence(timestamp_ns)
                return

            geofence_points = [GeofencePoint(x, y) for x, y in self.geofence.points]
            self.dataset_recorder.append_multiple(geofence_points)  # type: ignore

            self._transition_to_new_state(ReadyState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "confirm_geofence")

    def start_drive(self, timestamp_ns: int):
        if self.current_state.__class__ == ReadyState and self.current_step is None:
            logging.info(f"Starting drive at timestamp {timestamp_ns}")
            self.sample_next_step(timestamp_ns, "completed")
            self._transition_to_new_state(RunningState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_drive")

    def pause_drive(self, timestamp_ns: int):
        if self.current_state.__class__ == RunningState:
            logging.info(f"Pausing drive at timestamp {timestamp_ns}")
            self._transition_to_new_state(PausedState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "pause_drive")

    def resume_drive(self, timestamp_ns: int):
        if self.current_state.__class__ in (PausedState, BackToGeofenceState) and self.current_step is not None:
            logging.info(f"Resuming drive at timestamp {timestamp_ns}")
            self.restart_current_step(timestamp_ns, "restarted")
            self._transition_to_new_state(RunningState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "resume_drive")

    def go_back_inside_geofence(self, timestamp_ns: int):
        if self.current_state.__class__ == RunningState and not self.is_robot_inside_geofence():
            logging.info(f"Going back to center at timestamp {timestamp_ns}")
            self._transition_to_new_state(BackToGeofenceState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "resume_drive")

    def stop_drive(self, timestamp_ns: int):
        if self.current_state.__class__ in (RunningState, PausedState, BackToGeofenceState):
            logging.info(f"Stopped at timestamp {timestamp_ns}")

            self.current_step = None
            self.completed_commands = []

            self._transition_to_new_state(ReadyState(self), timestamp_ns)

            dataset = read_dataset(self.dataset_recorder.datasets_folder)
            generate_overview_visualization(dataset)

            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "stop_drive")

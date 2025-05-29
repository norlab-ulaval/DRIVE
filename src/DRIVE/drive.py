import logging
import os
import re
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from DRIVE.common import Command, Pose
from DRIVE.dataset_reader import DatasetReader
from DRIVE.dataset_recorder import DatasetRecorder
from DRIVE.geofencing import Geofence
from DRIVE.robot import Robot
from DRIVE.sampling import CommandSamplingStrategy


@dataclass
class Step:
    command: Command
    start_timestamp_ns: float


class IllegalStateTransition(Exception):
    def __init__(self, current_state: str, action: str):
        super().__init__(f"Cannot perform {action} from current state ({current_state})")


class DriveState(ABC):
    def __init__(self, drive):
        self.drive = drive

    @abstractmethod
    def run(self, timestamp_ns: float):
        pass

    def get_state_name(self) -> str:
        name = self.__class__.__name__
        # Convert CamelCase to snake_case
        s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
        snake_case = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
        return snake_case


class WaitingState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class GeofenceCreationState(DriveState):
    def __init__(self, drive):
        super().__init__(drive)

        self.geofence_points: list[np.ndarray] = [self.drive.robot.pose[:2]]
        self.distance_thresold_meters = 0.5

    def run(self, timestamp_ns: float):
        last_point = self.geofence_points[-1][:2]
        current_point = self.drive.robot.pose[:2]

        if np.linalg.norm(current_point - last_point) > self.distance_thresold_meters:
            self.geofence_points.append(current_point)


class ReadyState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class RunningState(DriveState):
    def __init__(self, drive, timestamp_ns: float):
        super().__init__(drive)

    def run(self, timestamp_ns: float):
        if len(self.drive.commands) >= self.drive.target_nb_steps:
            logging.info("Target number of steps reached, stopping drive")
            self.drive.stop_drive(timestamp_ns)
            return

        if not self.drive.robot.deadman_switch_pressed:
            logging.info("Deadman switch not pressed, pausing drive")
            self.drive.pause_drive(timestamp_ns)
            return

        if not self.drive.is_robot_inside_geofence():
            self.drive.go_back_inside_geofence(timestamp_ns)
            return

        last_poses = self.drive.robot.poses_buffer
        last_speeds = self.drive.robot.speeds_buffer
        last_accelerations = self.drive.robot.accelerations_buffer

        self.drive.dataset_recorder.save_poses(last_poses)
        self.drive.dataset_recorder.save_speeds(last_speeds)
        self.drive.dataset_recorder.save_accelerations(last_accelerations)

        self.drive.robot.empty_buffers()

        current_step: Step = self.drive.current_step

        if timestamp_ns - current_step.start_timestamp_ns > self.drive.step_duration_s * 1e9:
            self.drive.sample_next_step(timestamp_ns)
            return

        self.drive.robot.send_command(current_step.command)


class PausedState(DriveState):
    def run(self, timestamp_ns: float):
        if self.drive.robot.deadman_switch_pressed:
            logging.info("Deadman switch pressed, resuming drive")
            self.drive.resume_drive(timestamp_ns)
            return


class BackToCenterState(DriveState):
    def __init__(self, drive, timestamp_ns: float):
        super().__init__(drive)

        self.waiting_for_goal = False

    def run(self, timestamp_ns: float):
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
        state_transition_cb,
    ):
        self.robot = robot
        self.command_sampling_strategy = command_sampling_strategy
        self.target_nb_steps = target_nb_steps
        self.current_state = WaitingState(self)
        self.step_duration_s = step_duration_s

        self.geofence: None | Geofence = None
        self.current_step: None | Step = None

        experience_dir = os.path.join("/home", "root", "datasets")
        self.dataset_recorder = DatasetRecorder(experience_dir)
        self.dataset_reader = DatasetReader(experience_dir)

        self.commands = []

        self._state_transition_cb = state_transition_cb

    def _transition_to_new_state(self, new_state: DriveState, timestamp_ns: float):
        self.dataset_recorder.save_state_transition(
            self.current_state.get_state_name(), new_state.get_state_name(), int(timestamp_ns)
        )
        self.current_state = new_state
        self._state_transition_cb(new_state.get_state_name())

    def run(self, timestamp_ns: float):
        self.current_state.run(timestamp_ns)

    def sample_next_step(self, timestamp_ns: float, is_step_completed: bool = True):
        command = self.command_sampling_strategy.sample_command()
        self.commands.append(command)
        self.current_step = Step(command, timestamp_ns)
        logging.info(f"Sampling next command {command} at timestamp {timestamp_ns}")

        self.dataset_recorder.save_command(command, is_step_completed, int(timestamp_ns))

    def is_robot_inside_geofence(self) -> bool:
        if self.geofence is None:
            return True

        current_point = self.robot.pose[:2]
        return self.geofence.is_point_inside(current_point)

    def save_dataset(self, dataset_name: str):
        self.dataset_recorder.save_experience(dataset_name)

    def get_datasets(self) -> list[str]:
        return self.dataset_reader.get_datasets()

    def load_geofence(self, dataset_name: str, timestamp_ns: float):
        geofence_points = self.dataset_reader.load_geofence(dataset_name)
        if geofence_points:
            geofence_points_tuple: list[tuple[float, float]] = [(point.x, point.y) for point in geofence_points]
            self.current_state.geofence_points = geofence_points_tuple  # type: ignore
            self.confirm_geofence(timestamp_ns)

    def skip_current_step(self, timestamp_ns: float):
        logging.info("Skipping command...")
        self.sample_next_step(timestamp_ns, is_step_completed=False)

    def get_commands(self) -> np.ndarray:
        return np.array(self.commands)

    def get_geofence_points(self) -> np.ndarray:
        if self.current_state.__class__ == GeofenceCreationState:
            return np.array(self.current_state.geofence_points)  # type: ignore
        elif self.geofence is not None:
            return np.array([np.array(point) for point in self.geofence.polygon.exterior.coords])

        return np.array([])

    # ============================================ State transitions ============================================
    def start_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == WaitingState:
            logging.info(f"Starting geofence creation at timestamp {timestamp_ns}")
            self._transition_to_new_state(GeofenceCreationState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_geofence")

    def restart_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == GeofenceCreationState:
            logging.info(f"Restarting geofence creation at timestamp {timestamp_ns}")
            self._transition_to_new_state(GeofenceCreationState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "restart_geofence")

    def confirm_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ in (GeofenceCreationState, WaitingState):
            logging.info(f"Confirmed geofence at timestamp {timestamp_ns}")
            self.geofence = Geofence(self.current_state.geofence_points)  # type: ignore
            self.dataset_recorder.save_geofence(self.current_state.geofence_points)  # type: ignore
            self._transition_to_new_state(ReadyState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "confirm_geofence")

    def start_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == ReadyState and self.current_step is None:
            logging.info(f"Starting drive at timestamp {timestamp_ns}")
            self.sample_next_step(timestamp_ns)
            self._transition_to_new_state(RunningState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_drive")

    def pause_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == RunningState:
            logging.info(f"Pausing drive at timestamp {timestamp_ns}")
            self._transition_to_new_state(PausedState(self), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "pause_drive")

    def resume_drive(self, timestamp_ns: float):
        if self.current_state.__class__ in (PausedState, BackToCenterState) and self.current_step is not None:
            logging.info(f"Resuming drive at timestamp {timestamp_ns}")
            self.current_step.start_timestamp_ns = timestamp_ns
            self._transition_to_new_state(RunningState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "resume_drive")

    def go_back_inside_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == RunningState and not self.is_robot_inside_geofence():
            logging.info(f"Going back to center at timestamp {timestamp_ns}")
            self._transition_to_new_state(BackToCenterState(self, timestamp_ns), timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "resume_drive")

    def stop_drive(self, reason: str, timestamp_ns: float):
        if self.current_state.__class__ in (RunningState, PausedState, BackToCenterState):
            logging.info(f"Stopped at timestamp {timestamp_ns}")
            self.dataset_recorder.save_stop_reason(reason)
            self._transition_to_new_state(WaitingState(self), timestamp_ns)

            self.geofence = None
            self.current_step = None
            self.commands.clear()

            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "stop_drive")

    def can_skip_command(self) -> bool:
        return self.current_state.__class__ in (RunningState, PausedState)

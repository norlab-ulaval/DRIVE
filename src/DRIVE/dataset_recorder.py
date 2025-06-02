import os

from DRIVE.common import Command, Pose
from DRIVE.csv_writer import CsvWriter
from DRIVE.data_types import (
    DriveStep,
    GeofencePoint,
    Position6DOF,
    Speed6DOF,
    Acceleration6DOF,
    Serializable,
    StateTransition,
)


class DatasetRecorder:
    def __init__(self, datasets_folder: str):
        self.datasets_folder = datasets_folder
        self.step_id = 0

        self.writers: dict[type[Serializable], CsvWriter] = {}

        self._register(DriveStep)
        self._register(Position6DOF)
        self._register(Speed6DOF)
        self._register(Acceleration6DOF)
        self._register(GeofencePoint)
        self._register(StateTransition)

    def _register(self, saveable_type: type[Serializable]):
        self.writers[saveable_type] = CsvWriter(saveable_type)

    def save_command(self, command: Command, is_step_completed: bool, timestamp_ns: int):
        new_step = DriveStep(timestamp_ns, self.step_id, command[0], command[1], is_step_completed)
        self.writers[DriveStep].save_line(new_step)
        self.step_id += 1

    def save_pose(self, pose: Pose, timestamp_ns: int):
        pose_6DOF = Position6DOF(timestamp_ns, self.step_id, *pose)
        self.writers[Position6DOF].save_line(pose_6DOF)

    def save_speed(self, speed: Pose, timestamp_ns: int):
        speed_6DOF = Speed6DOF(timestamp_ns, self.step_id, *speed)
        self.writers[Speed6DOF].save_line(speed_6DOF)

    def save_acceleration(self, acceleration: Pose, timestamp_ns: int):
        acc_6DOF = Acceleration6DOF(timestamp_ns, self.step_id, *acceleration)
        self.writers[Acceleration6DOF].save_line(acc_6DOF)

    def save_geofence(self, geofence_points: list[tuple[float, float]]):
        for x, y in geofence_points:
            point = GeofencePoint(x, y)
            self.writers[GeofencePoint].save_line(point)

    def save_state_transition(self, from_state: str, to_state: str, timestamp_ns: int):
        state_transition = StateTransition(timestamp_ns, self.step_id, from_state, to_state)
        self.writers[StateTransition].save_line(state_transition)

    def save_poses(self, poses_array):
        for pose, timestamp_ns in poses_array:
            self.save_pose(pose, timestamp_ns)

    def save_speeds(self, speeds_array):
        for speed, timestamp_ns in speeds_array:
            self.save_speed(speed, timestamp_ns)

    def save_accelerations(self, accelerations_array):
        for acc, timestamp_ns in accelerations_array:
            self.save_acceleration(acc, timestamp_ns)

    def save_experience(self, dataset_name: str):
        save_folder_path = os.path.join(self.datasets_folder, dataset_name)
        os.makedirs(save_folder_path, exist_ok=True)

        for writer in self.writers.values():
            writer.save_data_to_file(save_folder_path)

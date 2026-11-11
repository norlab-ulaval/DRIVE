import os
from pathlib import Path

from DRIVE.writing import (
    Acceleration6DOF,
    CsvWriter,
    DriveStep,
    GeofencePoint,
    Position6DOF,
    Speed6DOF,
    StateTransition,
    Writable,
    Writer,
)


class DatasetRecorder:
    def __init__(self, dataset_folder: Path):
        self.datasets_folder = dataset_folder

        os.makedirs(self.datasets_folder, exist_ok=True)

        if not dataset_folder.is_dir():
            raise Exception("dataset_folder is not a directory")

        self.writers: dict[type[Writable], Writer] = {
            Position6DOF: CsvWriter[Position6DOF](dataset_folder / "positions.csv"),
            Speed6DOF: CsvWriter[Speed6DOF](dataset_folder / "velocities.csv"),
            Acceleration6DOF: CsvWriter[Acceleration6DOF](dataset_folder / "accelerations.csv"),
            DriveStep: CsvWriter[DriveStep](dataset_folder / "steps.csv"),
            GeofencePoint: CsvWriter[GeofencePoint](dataset_folder / "geofence.csv"),
            StateTransition: CsvWriter[StateTransition](dataset_folder / "state_transitions.csv"),
        }

    def append(self, line: Writable):
        self.append_multiple([line])

    def append_multiple(self, lines: list[Writable]):
        if len(lines) == 0:
            return

        if type(lines[0]) not in self.writers:
            raise Exception(f"No writer registered for type {type(lines[0])}")

        self.writers[type(lines[0])].append_multiple(lines)

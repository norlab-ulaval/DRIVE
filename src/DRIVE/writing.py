from abc import ABC, abstractmethod
import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Generic, TypeVar


@dataclass
class Writable:
    @classmethod
    def fields(cls) -> list[str]:
        return list(cls.__annotations__.keys())


T = TypeVar("T", bound=Writable)


class Writer(Generic[T], ABC):
    @abstractmethod
    def append(self, line: T):
        pass

    @abstractmethod
    def append_multiple(self, lines: list[T]):
        pass


class CsvWriter(Writer[T]):
    def __init__(self, file_path: Path):
        self.file_path = file_path

    def append(self, line: T):
        self.append_multiple([line])

    def append_multiple(self, lines: list[T]):
        if len(lines) == 0:
            return

        if not self.file_path.exists():
            self._write_header(lines[0])

        with open(self.file_path, mode="a", newline="\n", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=lines[0].fields())
            writer.writerows([asdict(line) for line in lines])

    def _write_header(self, data: T):
        with open(self.file_path, mode="w", newline="\n", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=data.fields())
            writer.writeheader()


@dataclass
class Position6DOF(Writable):
    timestamp: int  # ns
    step_id: int
    x: float  # m
    y: float  # m
    z: float  # m
    roll: float  # rad
    pitch: float  # rad
    yaw: float  # rad


@dataclass
class Speed6DOF(Writable):
    timestamp: int  # ns
    step_id: int
    speed_x: float  # m/s
    speed_y: float  # m/s
    speed_z: float  # m/s
    speed_roll: float  # rad/s
    speed_pitch: float  # rad/s
    speed_yaw: float  # rad/s


@dataclass
class Acceleration6DOF(Writable):
    timestamp: int  # ns
    step_id: int
    acc_x: float  # m/(s**2)
    acc_y: float  # m/(s**2)
    acc_z: float  # m/(s**2)
    gyro_x: float  # rad/s
    gyro_y: float  # rad/s
    gyro_z: float  # rad/s


@dataclass
class DriveStep(Writable):
    id: int
    start_timestamp: int  # ns
    end_timestamp: int  # ns
    commanded_linear_velocity: float  # m/s
    commanded_angular_velocity: float  # rad/s
    completion_status: str


@dataclass
class GeofencePoint(Writable):
    x: float  # m
    y: float  # m


@dataclass
class StateTransition(Writable):
    timestamp: int  # ns
    step_id: int
    from_state: str
    to_state: str

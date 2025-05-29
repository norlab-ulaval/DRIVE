from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class Serializable(ABC):
    @classmethod
    def headers(cls) -> list[str]:
        return list(cls.__annotations__.keys())

    @staticmethod
    @abstractmethod
    def get_filename() -> str:
        pass


@dataclass
class Position6DOF(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "positions"

    timestamp: int  # ns
    step_id: int
    x: float  # m
    y: float  # m
    z: float  # m
    roll: float  # rad
    pitch: float  # rad
    yaw: float  # rad


@dataclass
class Speed6DOF(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "speeds"

    timestamp: int  # ns
    step_id: int
    speed_x: float  # m/s
    speed_y: float  # m/s
    speed_z: float  # m/s
    speed_roll: float  # rad/s
    speed_pitch: float  # rad/s
    speed_yaw: float  # rad/s


@dataclass
class Acceleration6DOF(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "accelerations"

    timestamp: int  # ns
    step_id: int
    a_x: float  # m/(s**2)
    a_y: float  # m/(s**2)
    a_z: float  # m/(s**2)
    a_roll: float  # rad/(s**2)
    a_pitch: float  # rad/(s**2)
    a_yaw: float  # rad/(s**2)


@dataclass
class DriveStep(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "steps"

    step_start_timestamp: int  # ns
    step_id: int
    commanded_linear_velocity: float  # m/s
    commanded_angular_velocity: float  # rad/s
    is_completed: bool


@dataclass
class GeofencePoint(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "geofence"

    x: float  # m
    y: float  # m


@dataclass
class StateTransition(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "state_transitions"

    timestamp: int  # ns
    step_id: int
    from_state: str
    to_state: str


@dataclass
class StopReason(Serializable):
    @staticmethod
    def get_filename() -> str:
        return "stop_reason"

    reason: str

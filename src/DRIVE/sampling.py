from abc import ABC, abstractmethod
from multiprocessing import Value

import numpy as np

from DRIVE.common import Command
from rclpy.exceptions import ParameterException


class CommandSamplingStrategy(ABC):
    """
    Abstract base class for command sampling strategies.

    Subclasses must implement the `sample_command` method to return a motion command,
    typically represented as a 2D numpy array [v_x, omega_z] (linear velocity, angular velocity).
    """

    @abstractmethod
    def sample_command(self) -> Command:
        pass


class RandomSampling(CommandSamplingStrategy):
    """
    Samples linear and angular speeds from uniform distributions defined by min/max bounds.
    """

    def __init__(
        self, min_linear_speed: float, max_linear_speed: float, min_angular_speed: float, max_angular_speed: float
    ):
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError("Minimum linear speed must be greater than maximum linear speed")

        if self.min_angular_speed > self.max_angular_speed:
            raise ValueError("Minimum angular speed must be greater than maximum angular speed")

        self.min_linear_speed = min_linear_speed
        self.max_linear_speed = max_linear_speed
        self.min_angular_speed = min_angular_speed
        self.max_angular_speed = max_angular_speed

    def sample_command(self) -> Command:
        v_x = np.random.uniform(self.min_linear_speed, self.max_linear_speed)
        omega_z = np.random.uniform(self.min_angular_speed, self.max_angular_speed)

        return np.array([v_x, omega_z])


class DiffDriveSampling(CommandSamplingStrategy):

    def __init__(self, wheel_radius: float, base_width: float, min_wheel_speed: float, max_wheel_speed: float):
        if wheel_radius <= 0:
            raise ValueError("Wheel radius must be greater than 0")

        if base_width <= 0:
            raise ValueError("Base width must be greater than 0")

        if min_wheel_speed > max_wheel_speed:
            raise ValueError("Minimum wheel speed must be greater than maximum wheel speed")

        self.wheel_radius = wheel_radius
        self.base_width = base_width
        self.min_wheel_speed = min_wheel_speed
        self.max_wheel_speed = max_wheel_speed

    def sample_command(self) -> Command:
        v_left = np.random.uniform(self.min_wheel_speed, self.max_wheel_speed)
        v_right = np.random.uniform(self.min_wheel_speed, self.max_wheel_speed)

        v_body = self.wheel_radius * (0.5 * v_left + 0.5 * v_right)
        v_angular_body = self.wheel_radius / self.base_width * (v_right - v_left)

        return np.array([v_body, v_angular_body])


class CommandSamplingFactory:
    @staticmethod
    def create_sampling_strategy(command_sampling_strategy_str: str, params: dict) -> CommandSamplingStrategy:
        if command_sampling_strategy_str == "random":
            return RandomSampling(
                params["min_linear_speed"],
                params["max_linear_speed"],
                params["min_angular_speed"],
                params["max_angular_speed"],
            )
        if command_sampling_strategy_str == "diff_drive":
            return DiffDriveSampling(
                params["wheel_radius"], params["base_width"], params["min_wheel_speed"], params["max_wheel_speed"]
            )
        else:
            raise ValueError(f"Unknown command sampling strategy: {command_sampling_strategy_str}")

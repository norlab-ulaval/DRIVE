from abc import ABC, abstractmethod
from cProfile import label
from multiprocessing import Value

from matplotlib import pyplot as plt
import numpy as np
import pointpats
import shapely

from DRIVE.common import Command


class CommandSamplingStrategy(ABC):
    """
    Abstract base class for command sampling strategies.

    Subclasses must implement the `sample_command` method to return a motion command,
    typically represented as a 2D numpy array [v_x, omega_z] (linear velocity, angular velocity).
    """

    @abstractmethod
    def sample_command(self) -> Command:
        pass

    @abstractmethod
    def visualize(self, nb_steps: int):
        """Shows a matplotlib figure showing the sampling zone and the next 'nb_steps' points to be sampled. This function will reset the RNG the starting seed to ensure it does not affect the sample_command"""
        pass


class RandomSampling(CommandSamplingStrategy):
    """
    Samples linear and angular speeds from uniform distributions defined by min/max bounds.
    """

    def __init__(
        self,
        min_linear_speed: float,
        max_linear_speed: float,
        min_angular_speed: float,
        max_angular_speed: float,
        seed=None,
    ):
        if min_linear_speed > max_linear_speed:
            raise ValueError("Minimum linear speed must be greater than maximum linear speed")

        if min_angular_speed > max_angular_speed:
            raise ValueError("Minimum angular speed must be greater than maximum angular speed")

        self.min_linear_speed = min_linear_speed
        self.max_linear_speed = max_linear_speed
        self.min_angular_speed = min_angular_speed
        self.max_angular_speed = max_angular_speed

        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def sample_command(self) -> Command:
        v_x = self.rng.uniform(self.min_linear_speed, self.max_linear_speed)
        omega_z = self.rng.uniform(self.min_angular_speed, self.max_angular_speed)

        return np.array([v_x, omega_z])

    def visualize(self, nb_steps: int):
        rect = np.array(
            [
                [self.min_angular_speed, self.max_linear_speed],
                [self.max_angular_speed, self.max_linear_speed],
                [self.max_angular_speed, self.min_linear_speed],
                [self.min_angular_speed, self.min_linear_speed],
            ]
        )

        poly = shapely.Polygon(rect)

        self.rng = np.random.default_rng(self.seed)

        commands = []
        for _ in range(nb_steps):
            commands.append(self.sample_command())
        commands = np.array(commands)

        self.rng = np.random.default_rng(self.seed)

        plt.figure()
        x, y = poly.exterior.xy
        plt.plot(x, y, label="Sampling Space", color="green")
        plt.scatter(commands[:, 1], commands[:, 0], color="blue", label="Sampled Commands")
        plt.xlabel("Angular Speed")
        plt.ylabel("Linear Speed")
        plt.title("Speed Constraint Polygon")
        plt.grid(True)
        plt.axis("equal")
        plt.show()


class DiffDriveSampling(CommandSamplingStrategy):

    def __init__(
        self,
        min_linear_speed: float,
        max_linear_speed: float,
        min_angular_speed: float,
        max_angular_speed: float,
        wheel_radius: float,
        base_width: float,
        min_wheel_speed: float,
        max_wheel_speed: float,
        seed=None,
    ):
        if wheel_radius <= 0:
            raise ValueError("Wheel radius must be greater than 0")

        if base_width <= 0:
            raise ValueError("Base width must be greater than 0")

        if min_wheel_speed > max_wheel_speed:
            raise ValueError("Minimum wheel speed must be greater than maximum wheel speed")

        self.min_linear_speed = min_linear_speed
        self.max_linear_speed = max_linear_speed
        self.min_angular_speed = min_angular_speed
        self.max_angular_speed = max_angular_speed
        self.wheel_radius = wheel_radius
        self.base_width = base_width
        self.min_wheel_speed = min_wheel_speed
        self.max_wheel_speed = max_wheel_speed

        self._compute_sampling_space()

        self.seed = seed
        np.random.seed(seed)  # pointspats uses global np.random :(

    def sample_command(self) -> Command:
        point = pointpats.random.poisson(self.sampling_space, size=1)  # type: ignore

        return np.array([point[0], point[1]])

    def visualize(self, nb_steps):
        np.random.seed(self.seed)

        commands = []
        for _ in range(nb_steps):
            commands.append(self.sample_command())
        commands = np.array(commands)

        np.random.seed(self.seed)

        plt.figure()

        x, y = self.body_input_space.exterior.xy
        plt.plot(y, x, color="red", label="Safety Limit", linestyle="--")

        x, y = self.wheel_input_space.exterior.xy
        plt.plot(y, x, color="yellow", label="Wheel Input Space")

        x, y = self.sampling_space.exterior.xy
        plt.plot(y, x, color="green", label="Sampling Space")

        plt.scatter(commands[:, 1], commands[:, 0], color="blue", label="Sampled Commands")

        plt.xlabel("Angular Speed")
        plt.ylabel("Linear Speed")
        plt.title("Speed Constraint Polygon")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.show()

    def _jacobian(self) -> np.ndarray:
        return self.wheel_radius * np.array([[1.0 / 2, 1.0 / 2], [-1.0 / (self.base_width), 1 / (self.base_width)]])

    def _compute_sampling_space(self):
        body_frame_constraints = np.array(
            [
                [self.min_linear_speed, self.max_angular_speed],
                [self.max_linear_speed, self.max_angular_speed],
                [self.max_linear_speed, self.min_angular_speed],
                [self.min_linear_speed, self.min_angular_speed],
            ]
        )

        wheel_constraints = np.array(
            [
                [self.max_wheel_speed, self.max_wheel_speed, self.min_wheel_speed, self.min_wheel_speed],
                [self.min_wheel_speed, self.max_wheel_speed, self.max_wheel_speed, self.min_wheel_speed],
            ]
        )

        body_frame_wheel_constraints = self._jacobian() @ wheel_constraints

        polygon_bf_constraints = shapely.Polygon(body_frame_constraints)
        pol_wheel_bf_constraints = shapely.Polygon(body_frame_wheel_constraints.T)

        self.wheel_input_space: shapely.Polygon = pol_wheel_bf_constraints
        self.body_input_space: shapely.Polygon = polygon_bf_constraints
        self.sampling_space: shapely.Polygon = shapely.intersection(pol_wheel_bf_constraints, polygon_bf_constraints)  # type: ignore


class CommandSamplingFactory:

    @staticmethod
    def create_sampling_strategy(
        command_sampling_strategy_str: str, params: dict, seed=None
    ) -> CommandSamplingStrategy:
        seed = params.get("seed", None)

        if command_sampling_strategy_str == "random":
            return RandomSampling(
                params["min_linear_speed"],
                params["max_linear_speed"],
                params["min_angular_speed"],
                params["max_angular_speed"],
                seed,
            )
        if command_sampling_strategy_str == "diff_drive":
            return DiffDriveSampling(
                params["min_linear_speed"],
                params["max_linear_speed"],
                params["min_angular_speed"],
                params["max_angular_speed"],
                params["wheel_radius"],
                params["base_width"],
                params["min_wheel_speed"],
                params["max_wheel_speed"],
                seed,
            )
        else:
            raise ValueError(f"Unknown command sampling strategy: {command_sampling_strategy_str}")


if __name__ == "__main__":
    params = {
        "min_linear_speed": -1.0,
        "max_linear_speed": 1.0,
        "min_angular_speed": -1.0,
        "max_angular_speed": 1.0,
        "wheel_radius": 1.0,
        "base_width": 1.0,
        "min_wheel_speed": -1.0,
        "max_wheel_speed": 1.0,
        "seed": 1234,
    }

    strategy = CommandSamplingFactory.create_sampling_strategy("diff_drive", params)
    strategy.visualize(100)

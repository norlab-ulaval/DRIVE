from abc import ABC, abstractmethod
from cProfile import label
from multiprocessing import Value

from matplotlib import pyplot as plt
import numpy as np
import pointpats
import shapely

from DRIVE.common import Command
from DRIVE.input_space import InputSpace
from DRIVE.models import IdealDiffDriveModel


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

        self.input_space = InputSpace(
            IdealDiffDriveModel(wheel_radius, base_width),
            min_linear_speed,
            max_linear_speed,
            min_angular_speed,
            max_angular_speed,
            min_wheel_speed,
            max_wheel_speed,
        )
        self.sampling_space = self.input_space.body_input_space()

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


class AlternatingNullDiffDriveSampling(CommandSamplingStrategy):
    """
    Alternates between a null command [0, 0] and a DiffDriveSampling command.
    """

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
        self.diff_drive_sampler = DiffDriveSampling(
            min_linear_speed,
            max_linear_speed,
            min_angular_speed,
            max_angular_speed,
            wheel_radius,
            base_width,
            min_wheel_speed,
            max_wheel_speed,
            seed,
        )
        
        self.null_command = np.array([0.0, 0.0])
        self.is_next_null = True  # Start with null command
        
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def sample_command(self) -> Command:
        if self.is_next_null:
            self.is_next_null = False
            return self.null_command.copy()
        else:
            self.is_next_null = True
            return self.diff_drive_sampler.sample_command()

    def visualize(self, nb_steps: int):
        # Reset state for visualization
        self.is_next_null = True
        np.random.seed(self.seed)
        
        commands = []
        null_commands = []
        diff_drive_commands = []
        
        for i in range(nb_steps):
            cmd = self.sample_command()
            commands.append(cmd)
            if np.array_equal(cmd, self.null_command):
                null_commands.append(cmd)
            else:
                diff_drive_commands.append(cmd)
        
        commands = np.array(commands)
        null_commands = np.array(null_commands) if null_commands else np.empty((0, 2))
        diff_drive_commands = np.array(diff_drive_commands) if diff_drive_commands else np.empty((0, 2))
        
        # Reset state after visualization
        self.is_next_null = True
        np.random.seed(self.seed)

        plt.figure()
        
        # Plot sampling space
        x, y = self.diff_drive_sampler.sampling_space.exterior.xy
        plt.plot(y, x, color="green", label="DiffDrive Sampling Space")
        
        # Plot commands
        if len(diff_drive_commands) > 0:
            plt.scatter(diff_drive_commands[:, 1], diff_drive_commands[:, 0], 
                       color="blue", label="DiffDrive Commands", alpha=0.7)
        
        if len(null_commands) > 0:
            plt.scatter(null_commands[:, 1], null_commands[:, 0], 
                       color="red", s=100, label="Null Commands")

        plt.xlabel("Angular Speed")
        plt.ylabel("Linear Speed")
        plt.title("Alternating Null/DiffDrive Sampling")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.show()


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
        if command_sampling_strategy_str == "alternating_null_diff_drive":
            return AlternatingNullDiffDriveSampling(
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

    # Test the new alternating strategy
    strategy = CommandSamplingFactory.create_sampling_strategy("alternating_null_diff_drive", params)
    strategy.visualize(20)

    # Test a few sample commands
    print("Sample commands from AlternatingNullDiffDriveSampling:")
    for i in range(6):
        cmd = strategy.sample_command()
        print(f"Command {i+1}: {cmd}")

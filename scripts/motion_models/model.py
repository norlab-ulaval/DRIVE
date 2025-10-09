from abc import ABC, abstractmethod
from typing import Callable

from matplotlib import pyplot as plt
import numpy as np

StateDiffFunction = Callable[[np.ndarray, np.ndarray], np.ndarray]  # f(x, u)
IntegratorFunction = Callable[
    [StateDiffFunction, np.ndarray, np.ndarray, float], np.ndarray
]  # x_next = f(dstate, x, u, dt)


class MotionModel(ABC):
    @abstractmethod
    def dstate(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        pass


def rk4(f: StateDiffFunction, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


def euler(f: StateDiffFunction, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    return x + dt * f(x, u)


class IdealDiffDrive(MotionModel):
    """
    Ideal differential drive model.

    State
    ----------
    [x, y, theta] (m, m, rad)


    Control input
    -----------
    [left_wheel_angular_speed, right_wheel_angular_speed] (rad/s, rad/s).
    """

    def __init__(self, wheelbase, wheel_radius):
        """
        wheelbase: (m)
        wheel_radius: (m)
        """
        self.wheelbase = wheelbase
        self.wheel_radius = wheel_radius

        self.J = wheel_radius * np.array(
            [
                [1.0 / 2.0, 1.0 / 2.0],
                [-1.0 / self.wheelbase, 1.0 / self.wheelbase],
            ]
        )
        self.inv_J = np.linalg.inv(self.J)

    def dstate(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Compute the time derivative of the system state.

        Parameters
        ----------
        x : np.ndarray
            State vector of shape (3, 1).
        u : np.ndarray
            Control input vector of shape (2, 1).

        Returns
        -------
        np.ndarray
            State derivative of shape (3, 1).
        """
        assert u.shape[0] == self.J.shape[1], f"Input u must be of shape {self.J.shape[1]}xN"

        v, omega = (self.J @ u).flatten()
        theta = x[2]

        return np.array([v * np.cos(theta), v * np.sin(theta), omega])


def predict(
    model: MotionModel, x_0: np.ndarray, u_arr: np.ndarray, dt: float, integrator: IntegratorFunction
) -> np.ndarray:
    """
    Propagate the system state over time given control inputs.

    Parameters
    ----------
    x_0 : np.ndarray
        Initial state vector of shape (3, 1).
    u_arr : np.ndarray
        Control input array of shape (2, N), where N is the number of time steps.
    dt : float
        Integration time step in seconds.
    integrator : callable
        Integration method to use (e.g., rk4, euler).

    Returns
    -------
    np.ndarray
        State trajectory array of shape (3, N+1).
    """

    N = u_arr.shape[1]
    x_arr = np.zeros((3, N + 1))
    x_arr[:, 0] = x_0[:, 0]

    for i in range(N):
        x = x_arr[:, i]
        u = u_arr[:, i]

        x_next = integrator(model.dstate, x, u, dt)
        x_arr[:, i + 1] = x_next

    return x_arr


if __name__ == "__main__":
    model = IdealDiffDrive(wheelbase=1.5, wheel_radius=1.0)

    x0 = np.zeros((3, 1))
    dt = 0.1
    N = int(6.0 / dt)
    u = np.array([1.0, 2.0])
    u_arr = np.column_stack([u for _ in range(N)])

    x_arr = predict(model, x0, u_arr, dt, rk4)
    x_euler_arr = predict(model, x0, u_arr, dt, euler)

    plt.scatter(x_arr[0, :], x_arr[1, :])
    plt.scatter(x_euler_arr[0, :], x_euler_arr[1, :], color="red")
    plt.axis("equal")
    plt.show()

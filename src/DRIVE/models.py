import numpy as np


class IdealDiffDriveModel:
    def __init__(self, wheelbase, wheel_radius):
        """
        wheelbase: (m)
        wheel_radius: (m)
        """
        self.wheelbase = wheelbase
        self.wheel_radius = wheel_radius

        self.J = wheel_radius * np.array(
            [
                [1 / 2.0, 1 / 2.0],
                [-1 / self.wheelbase, 1 / self.wheelbase],
            ]
        )
        self.inv_J = np.linalg.inv(self.J)

    def jacobian(self):
        return self.J

    def inv_jacobian(self):
        return self.inv_J

    def forward_kinematics(self, left_wheel_angular_speed, right_wheel_angular_speed):
        """
        left_wheel_angular_speed: (rad/s)
        right_wheel_angular_speed: (rad/s)

        Returns: [linear_velocity, angular_velocity] (m/s, rad/s)
        """
        u = np.array([left_wheel_angular_speed, right_wheel_angular_speed]).T
        return self.J @ u

    def matrix_forward_kinematics(self, wheel_speeds: np.ndarray) -> np.ndarray:
        """
        wheel_speeds: (N, 2) array of [left_wheel_angular_speed, right_wheel_angular_speed] (rad/s)

        Returns: (N, 2) array of [linear_velocity, angular_velocity] (m/s, rad/s)
        """
        return self.J @ wheel_speeds.T

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        """
        linear_velocity: (m/s)
        angular_velocity: (rad/s)

        Returns: [left_wheel_angular_speed, right_wheel_angular_speed] (rad/s)
        """
        u = np.array([linear_velocity, angular_velocity]).T
        return self.inv_J @ u

    def matrix_inverse_kinematics(self, body_speeds: np.ndarray) -> np.ndarray:
        """
        body_speeds: (N, 2) array of [linear_velocity, angular_velocity] (m/s, rad/s)

        Returns: (N, 2) array of [left_wheel_angular_speed, right_wheel_angular_speed] (rad/s)
        """
        return self.inv_J @ body_speeds.T


WARTHOG_MODEL = IdealDiffDriveModel(wheelbase=1.08, wheel_radius=0.3)

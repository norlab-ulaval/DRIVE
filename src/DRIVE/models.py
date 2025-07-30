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
                [0.0, 0.0],
                [-1 / self.wheelbase, 1 / self.wheelbase],
            ]
        )
        self.inv_J = np.linalg.pinv(self.J)

    def jacobian(self):
        return self.J

    def inv_jacobian(self):
        return self.inv_J

    def forward_kinematics(self, left_wheel_angular_speed, right_wheel_angular_speed):
        """
        left_wheel_angular_speed: (rad/s)
        right_wheel_angular_speed: (rad/s)
        """
        u = np.array([left_wheel_angular_speed, right_wheel_angular_speed]).T
        return self.J @ u

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        """
        linear_velocity: (m/s)
        angular_velocity: (rad/s)
        """
        u = np.array([linear_velocity, 0.0, angular_velocity]).T
        return self.inv_J @ u


WARTHOG_MODEL = IdealDiffDriveModel(wheelbase=1.08, wheel_radius=0.3)

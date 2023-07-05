import numpy as np
from util.transform_algebra import *

class Ideal_diff_drive:
    def __init__(self, r, baseline, dt):
        self.dt = dt
        self.r = r
        self.baseline = baseline
        self.jacobian = r * np.array([[0.5, 0.5],
                                      [-1/(baseline), 1/(baseline)]])
        self.jacobian_3x3 = r * np.array([[0.5, 0.5],
                                          [0.0, 0.0],
                                          [-1/(baseline), 1/(baseline)]])
        self.inv_jacobian = np.linalg.inv(self.jacobian)

        self.rotation_body_to_world = np.eye(2)
        self.body_vel_world_3d = np.zeros(6)
        self.body_vel_world_2d = np.zeros(3)
        self.state_2d = np.zeros(3)

    # TODO: Adapt to fit 2x2 jacobian matrix
    # def predict(self, init_state, input):
    #     """
    #     :param init_state: initial state array [x, y, z, roll, pitch, yaw]
    #     :param input: input array [omega_l, omega_r]
    #     :return: next_state
    #     """
    #     self.state_2d[:2] = init_state[:2]
    #     self.state_2d[2] = init_state[-1]
    #     yaw_to_rotmat2d(self.rotation_body_to_world, init_state[-1])
    #     body_vel = self.jacobian @ input
    #     self.body_vel_world_2d[:2] = self.rotation_body_to_world @ body_vel[:2]
    #     self.body_vel_world_2d[2] = body_vel[2]
    #     self.body_vel_world_3d[:2] = self.body_vel_world_2d[:2]
    #     self.body_vel_world_3d[-1] = self.body_vel_world_2d[-1]
    #
    #     return init_state + self.body_vel_world_3d * self.dt

    def compute_body_vel(self, input):
        return self.jacobian @ input

    def compute_body_vel_horizon(self, horizon_input):
        return self.jacobian @ horizon_input

    def compute_wheel_vels(self, body_vel):
        return self.inv_jacobian @ body_vel

    def predict(self, init_state, input):
        """
        :param init_state: initial state array [x, y, z, roll, pitch, yaw]
        :param input: input array [omega_l, omega_r]
        :return: next_state
        """
        self.state_2d[:2] = init_state[:2]
        self.state_2d[2] = init_state[-1]
        yaw_to_rotmat2d(self.rotation_body_to_world, init_state[-1])
        body_vel = self.jacobian_3x3 @ input
        self.body_vel_world_2d[:2] = self.rotation_body_to_world @ body_vel[:2]
        self.body_vel_world_2d[2] = body_vel[2]
        self.body_vel_world_3d[:2] = self.body_vel_world_2d[:2]
        self.body_vel_world_3d[-1] = self.body_vel_world_2d[-1]

        # print(self.body_vel_world_3d)

        return init_state + self.body_vel_world_3d * self.dt

    def predict_2d(self, init_state, input):
        """
        :param init_state: initial state array [x, y, z, roll, pitch, yaw]
        :param input: input array [omega_l, omega_r]
        :return: next_state
        """

        yaw_to_rotmat2d(self.rotation_body_to_world, init_state[-1])
        body_vel = self.jacobian_3x3 @ input
        self.body_vel_world_2d[:2] = self.rotation_body_to_world @ body_vel[:2]
        self.body_vel_world_2d[2] = body_vel[2]
        return init_state + self.body_vel_world_2d * self.dt

    def adjust_motion_params(self, params):
        return None
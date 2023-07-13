import numpy as np

from scipy.optimize import minimize

from util.util_func import *

class Powertrain_Trainer:
    def __init__(self, powertrain_model, init_params, dataframe,
                              timesteps_per_horizon, dt, saved_params_path):
        self.powetrain_model = powertrain_model
        self.dataframe = dataframe
        self.timesteps_per_horizon = int(timesteps_per_horizon)
        self.params = init_params
        self.dt = dt
        self.saved_param_path = saved_params_path

        encoder_left_str_list = []
        encoder_right_str_list = []
        for i in range(0, self.timesteps_per_horizon):
            str_encoder_left_i = 'left_wheel_vel_' + str(i)
            str_encoder_right_i = 'right_wheel_vel_' + str(i)
            encoder_left_str_list.append(str_encoder_left_i)
            encoder_right_str_list.append(str_encoder_right_i)
        self.encoder_left_vels = self.dataframe[encoder_left_str_list].to_numpy()
        self.encoder_right_vels = self.dataframe[encoder_right_str_list].to_numpy()

        cmd_left_str_list = []
        cmd_right_str_list = []
        for i in range(0, self.timesteps_per_horizon):
            str_cmd_left_i = 'cmd_left_' + str(i)
            str_cmd_right_i = 'cmd_right_' + str(i)
            cmd_left_str_list.append(str_cmd_left_i)
            cmd_right_str_list.append(str_cmd_right_i)
        self.cmd_left_vels = self.dataframe[cmd_left_str_list].to_numpy()
        self.cmd_right_vels = self.dataframe[cmd_right_str_list].to_numpy()

    def update_params(self, new_params):
        self.powetrain_model.update_params(new_params)

    def find_max_wheel_vels(self):
        left_min_vel = min(self.dataframe['left_wheel_vel_39'])
        left_max_vel = max(self.dataframe['left_wheel_vel_39'])
        right_min_vel = min(self.dataframe['right_wheel_vel_39'])
        right_max_vel = max(self.dataframe['right_wheel_vel_39'])
        return left_min_vel, left_max_vel, right_min_vel, right_max_vel

    def compute_powertrain_error_all_steps(self, init_params):
        self.update_params(init_params)
        # print(init_params)
        wheel_vel_error = 0
        counted_pred_counter = 0

        transitory_state_mask = self.dataframe['transitory_state_mask'].to_numpy() == 1

        if self.wheel_side == 'left':
            self.powetrain_model.min_vel = min(self.dataframe['left_wheel_vel_39'])
            self.powetrain_model.max_vel = max(self.dataframe['left_wheel_vel_39'])
        if self.wheel_side == 'right':
            self.powetrain_model.min_vel = min(self.dataframe['right_wheel_vel_39'])
            self.powetrain_model.max_vel = max(self.dataframe['right_wheel_vel_39'])

        # for i, (inputs, targets, step, encoders, icp_vx, icp_vy, icp_omega, steady_state_mask, transitory_state_mask) in enumerate(self.dataloader):
        for i in range(0, len(self.dataframe)):
            # print(inputs)
            # print(targets)
            if transitory_state_mask[i]:
                if self.wheel_side == 'left':
                    # prev_wheel_vel = inputs[0, 6].numpy()
                    prev_wheel_vel = self.encoder_left_vels[i, 0]
                if self.wheel_side == 'right':
                    # prev_wheel_vel = inputs[0, 7].numpy()
                    prev_wheel_vel = self.encoder_right_vels[i, 0]
                cmd_time_elapsed = 0
                # if True:
                for j in range(0, self.timesteps_per_horizon):
                    if self.wheel_side == 'left':
                        cmd_wheel_vel = self.cmd_left_vels[i,j]
                    if self.wheel_side == 'right':
                        cmd_wheel_vel = self.cmd_right_vels[i,j]

                    predicted_wheel_vel = self.powetrain_model.compute_bounded_wheel_vels(cmd_wheel_vel, prev_wheel_vel, cmd_time_elapsed)
                    prev_wheel_vel = predicted_wheel_vel
                    cmd_time_elapsed += self.dt
                    if self.wheel_side == 'left':
                        wheel_vel_error += np.abs(predicted_wheel_vel - self.encoder_left_vels[i,j])
                    if self.wheel_side == 'right':
                        wheel_vel_error += np.abs(predicted_wheel_vel - self.encoder_right_vels[i,j])
                # print(horizon_error)
                counted_pred_counter += 1
        # print('total error : ', wheel_vel_error)
        # print('horizons accounted : ', counted_pred_counter)
        return wheel_vel_error

    def train_model(self, init_params, method, bounds):
        fun = lambda x: self.compute_powertrain_error_all_steps(x)
        self.wheel_side = 'left'
        left_training_result = minimize(fun, init_params, method=method, bounds=bounds)
        self.wheel_side = 'right'
        right_training_result = minimize(fun, init_params, method=method, bounds=bounds)
        return left_training_result.x, right_training_result.x
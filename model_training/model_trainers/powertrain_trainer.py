import numpy as np

from scipy.optimize import minimize

from util.util_func import *

class Powertrain_Trainer:
    def __init__(self, powertrain_model, init_params, dataloader, timesteps_per_horizon, wheel_side, dt):
        self.powetrain_model = powertrain_model
        self.dataloader = dataloader
        self.timesteps_per_horizon = timesteps_per_horizon
        self.params = init_params
        self.wheel_side = wheel_side
        self.dt = dt

    def update_dataloader(self, new_dataloader):
        self.dataloader = new_dataloader

    def update_params(self, new_params):
        self.powetrain_model.update_params(new_params)

    def compute_powertrain_error_all_steps(self, init_params):
        self.update_params(init_params)
        print(init_params)
        wheel_vel_error = 0
        counted_pred_counter = 0
        # self.x_train[idx], self.y_train[idx], self.calib_step[idx], self.mask[idx], self.cmd_vx[idx], self.cmd_omega[idx], \
        #                self.encoder_vx[idx], self.encoder_omega[idx], self.icp_vx[idx], self.icp_vy[idx], self.icp_omega[idx]

        encoder_left_str_list = []
        encoder_right_str_list = []
        for i in range(0, 40):
            str_encoder_left_i = 'left_wheel_vel_' + str(i)
            str_encoder_right_i = 'right_wheel_vel_' + str(i)
            encoder_left_str_list.append(str_encoder_left_i)
            encoder_right_str_list.append(str_encoder_right_i)

        if self.wheel_side == 'left':
            self.powetrain_model.min_vel = min(self.dataloader.dataset.data['left_wheel_vel_39'])
            self.powetrain_model.max_vel = max(self.dataloader.dataset.data['left_wheel_vel_39'])
        if self.wheel_side == 'right':
            self.powetrain_model.min_vel = min(self.dataloader.dataset.data['right_wheel_vel_39'])
            self.powetrain_model.max_vel = max(self.dataloader.dataset.data['right_wheel_vel_39'])

        for i, (inputs, targets, step, encoders, icp_vx, icp_vy, icp_omega, steady_state_mask, transitory_state_mask) in enumerate(self.dataloader):
            # print(inputs)
            # print(targets)
            predicted_state = inputs[0, :6].numpy()
            transitory_state_mask_bool = transitory_state_mask.numpy()
            if transitory_state_mask_bool:
                if self.wheel_side == 'left':
                    prev_wheel_vel = inputs[0, 6].numpy()
                if self.wheel_side == 'right':
                    prev_wheel_vel = inputs[0, 7].numpy()
                cmd_time_elapsed = 0
                # if True:
                for j in range(0, self.timesteps_per_horizon):
                    input_id = 6 + j * 2
                    if self.wheel_side == 'left':
                        cmd_wheel_vel = inputs[0, input_id].numpy()
                    if self.wheel_side == 'right':
                        cmd_wheel_vel = inputs[0, input_id+1].numpy()

                    predicted_wheel_vel = self.powetrain_model.compute_bounded_wheel_vels(cmd_wheel_vel, prev_wheel_vel, cmd_time_elapsed)
                    prev_wheel_vel = predicted_wheel_vel
                    cmd_time_elapsed += self.dt
                    if self.wheel_side == 'left':
                        wheel_vel_error += np.abs(predicted_wheel_vel - encoders[0, input_id-6].numpy())
                    if self.wheel_side == 'right':
                        wheel_vel_error += np.abs(predicted_wheel_vel - encoders[0, input_id-5].numpy())
                # print(horizon_error)
                counted_pred_counter += 1
        print('total error : ', wheel_vel_error)
        print('horizons accounted : ', counted_pred_counter)
        return wheel_vel_error

    def train_model(self, init_params, method, bounds, saved_array_path):
        fun = lambda x: self.compute_powertrain_error_all_steps(x)
        training_result = minimize(fun, init_params, method=method, bounds=bounds)
        np.save(saved_array_path, training_result.x)
        return training_result.x
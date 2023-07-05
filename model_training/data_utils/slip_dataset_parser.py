import numpy as np
import pandas as pd
from scipy.interpolate import make_smoothing_spline

from util.util_func import *
from util.transform_algebra import *
from models.kinematic.ideal_diff_drive import Ideal_diff_drive
from models.powertrain.bounded_powertrain import Bounded_powertrain

class SlipDatasetParser:
    def __init__(self, torch_ready_dataset_path, export_dataset_path, powertrain_model_params_path, robot):
        self.data = pd.read_pickle(torch_ready_dataset_path)
        self.n_horizons = len(self.data)
        self.rate = 20
        self.timestep = 1 / self.rate
        self.step_time_vector = np.linspace(0, 2, 40)
        self.export_dataset_path = export_dataset_path

        if robot == 'husky':
            self.steady_state_step_len = 160
            self.wheel_radius = 0.33 / 2
            self.baseline = 0.55
            self.rate = 0.05
            min_wheel_vel = -7
            max_wheel_vel = 7

        if robot == 'warthog-wheel':
            self.steady_state_step_len = 140
            self.wheel_radius = 0.3
            self.baseline = 1.1652
            self.rate = 0.05
            min_wheel_vel = -14
            # min_wheel_vel = -5
            max_wheel_vel = 14

        if robot == 'warthog-track':
            self.steady_state_step_len = 140
            self.wheel_radius = 0.3
            self.baseline = 1.1652
            self.rate = 0.05
            min_wheel_vel = -14
            # min_wheel_vel = -5
            max_wheel_vel = 14

        if robot == 'marmotte':
            self.steady_state_step_len = 140
            self.wheel_radius = 0.116
            self.baseline = 0.5927
            self.training_horizon = 2
            self.calib_step_time = 6
            self.rate = 0.05
            min_wheel_vel = -10
            # min_wheel_vel = -5
            max_wheel_vel = 10

        self.ideal_diff_drive = Ideal_diff_drive(self.wheel_radius, self.baseline, self.timestep)
        self.k = np.array([self.wheel_radius, self.baseline])

        bounded_powertrain_left_params = np.load(powertrain_model_params_path + 'powertrain_training_left.npy')
        self.bounded_powertrain_left = Bounded_powertrain(min_wheel_vel, max_wheel_vel, bounded_powertrain_left_params[0],
                                                          bounded_powertrain_left_params[1], self.timestep)
        bounded_powertrain_right_params = np.load(powertrain_model_params_path + 'powertrain_training_right.npy')
        self.bounded_powertrain_right = Bounded_powertrain(min_wheel_vel, max_wheel_vel, bounded_powertrain_right_params[0],
                                                          bounded_powertrain_right_params[1], self.timestep)

        cmd_left_str_list = []
        cmd_right_str_list = []
        encoder_left_str_list = []
        encoder_right_str_list = []
        for i in range(0, 40):
            str_cmd_left_i = 'cmd_left_' + str(i)
            str_cmd_right_i = 'cmd_right_' + str(i)
            str_trans_left_i = 'left_wheel_vel_' + str(i)
            str_trans_right_i = 'right_wheel_vel_' + str(i)
            cmd_left_str_list.append(str_cmd_left_i)
            cmd_right_str_list.append(str_cmd_right_i)
            encoder_left_str_list.append(str_trans_left_i)
            encoder_right_str_list.append(str_trans_right_i)
        self.cmd_left_vels_array = self.data[cmd_left_str_list].to_numpy()
        self.cmd_right_vels_array = self.data[cmd_right_str_list].to_numpy()
        self.encoder_left_vels_array = self.data[encoder_left_str_list].to_numpy()
        self.encoder_right_vels_array = self.data[encoder_right_str_list].to_numpy()
        self.bounded_powertrain_left.min_vel = np.min(self.encoder_left_vels_array)
        self.bounded_powertrain_left.max_vel = np.max(self.encoder_left_vels_array)
        self.bounded_powertrain_right.min_vel = np.min(self.encoder_right_vels_array)
        self.bounded_powertrain_right.max_vel = np.max(self.encoder_right_vels_array)

        icp_x_str_list = []
        icp_y_str_list = []
        icp_z_str_list = []
        icp_roll_str_list = []
        icp_pitch_str_list = []
        icp_yaw_str_list = []
        for i in range(0, 40):
            str_icp_x_i = 'icp_x_' + str(i)
            str_icp_y_i = 'icp_y_' + str(i)
            str_icp_z_i = 'icp_z_' + str(i)
            str_icp_roll_i = 'icp_roll_' + str(i)
            str_icp_pitch_i = 'icp_pitch_' + str(i)
            str_icp_yaw_i = 'icp_yaw_' + str(i)
            icp_x_str_list.append(str_icp_x_i)
            icp_y_str_list.append(str_icp_y_i)
            icp_z_str_list.append(str_icp_z_i)
            icp_roll_str_list.append(str_icp_roll_i)
            icp_pitch_str_list.append(str_icp_pitch_i)
            icp_yaw_str_list.append(str_icp_yaw_i)
        self.icp_x_array = self.data[icp_x_str_list].to_numpy()
        self.icp_y_array = self.data[icp_y_str_list].to_numpy()
        self.icp_z_array = self.data[icp_z_str_list].to_numpy()
        self.icp_roll_array = self.data[icp_roll_str_list].to_numpy()
        self.icp_pitch_array = self.data[icp_pitch_str_list].to_numpy()
        self.icp_yaw_array = self.data[icp_yaw_str_list].to_numpy()

        imu_yaw_str_list = []
        for i in range(0, 40):
            str_imu_yaw_i = 'imu_yaw_' + str(i)
            imu_yaw_str_list.append(str_imu_yaw_i)
        self.imu_yaw_array = self.data[imu_yaw_str_list].to_numpy()

    def compute_transitory_vels(self):
        transitory_state_mask = self.data['transitory_state_mask'].to_numpy()
        self.transitory_left_vels_array = np.zeros((self.cmd_left_vels_array.shape[0], self.cmd_left_vels_array.shape[1]))
        self.transitory_left_vels_array[0, :] = self.cmd_left_vels_array[0, :]
        self.transitory_right_vels_array = np.zeros((self.cmd_right_vels_array.shape[0], self.cmd_right_vels_array.shape[1]))
        self.transitory_right_vels_array[0, :] = self.cmd_right_vels_array[0, :]
        for i in range(1, self.n_horizons):
            if transitory_state_mask[i] == 1:
                self.transitory_left_vels_array[i, 0] = self.encoder_left_vels_array[i, 0]
                self.transitory_right_vels_array[i, 0] = self.encoder_right_vels_array[i, 0]
                cmd_elapsed_time = 0
                for j in range(1, self.cmd_right_vels_array.shape[1]):
                    self.transitory_left_vels_array[i, j] = self.bounded_powertrain_left.compute_bounded_wheel_vels(self.cmd_left_vels_array[i, j],
                                                                                                                    self.transitory_left_vels_array[i, j-1],
                                                                                                                    cmd_elapsed_time)
                    self.transitory_right_vels_array[i, j] = self.bounded_powertrain_right.compute_bounded_wheel_vels(
                        self.cmd_right_vels_array[i, j],
                        self.transitory_right_vels_array[i, j - 1],
                        cmd_elapsed_time)
                    cmd_elapsed_time += self.timestep
            else:
                self.transitory_left_vels_array[i, :] = np.clip(self.cmd_left_vels_array[i, :], self.bounded_powertrain_left.min_vel, self.bounded_powertrain_left.max_vel)
                self.transitory_right_vels_array[i, :] = np.clip(self.cmd_right_vels_array[i, :], self.bounded_powertrain_right.min_vel, self.bounded_powertrain_right.max_vel)

    def compute_transitory_body_vels(self):
        self.idd_body_vels_x_array = np.zeros((self.cmd_left_vels_array.shape[0], self.cmd_left_vels_array.shape[1]))
        self.idd_body_vels_y_array = np.zeros((self.cmd_left_vels_array.shape[0], self.cmd_left_vels_array.shape[1]))
        self.idd_body_vels_yaw_array = np.zeros((self.cmd_left_vels_array.shape[0], self.cmd_left_vels_array.shape[1]))

        for i in range(0, self.n_horizons):
            for j in range(0, self.cmd_left_vels_array.shape[1]):
                input_array = np.array([self.transitory_left_vels_array[i, j], self.transitory_right_vels_array[i,j]])
                body_vel_array = self.ideal_diff_drive.compute_body_vel(input_array)
                self.idd_body_vels_x_array[i,j] = body_vel_array[0]
                self.idd_body_vels_yaw_array[i,j] = body_vel_array[1]

    def unwrap_trajectory(self, trajectory):
        unwrapped_trajectory = np.zeros(trajectory.shape[0])
        unwrapped_trajectory[0] = trajectory[0]
        for i in range(0, trajectory.shape[0]-1):
            trajectory_displacement = trajectory[i+1] - trajectory[i]
            unwrapped_trajectory[i + 1] = unwrapped_trajectory[i] + wrap2pi(trajectory_displacement)
            # if trajectory_displacement > np.pi:
            #     unwrapped_trajectory[i+1] = unwrapped_trajectory[i] + trajectory_displacement - np.pi
            # elif trajectory_displacement < np.pi:
            #     unwrapped_trajectory[i + 1] = unwrapped_trajectory[i] + trajectory_displacement + np.pi
            # else:
            #     unwrapped_trajectory[i+1] = unwrapped_trajectory[i] + trajectory_displacement

        return(unwrapped_trajectory)

    def icp_traj_as_smoothed_spline(self, window_id):
        lambda_param = 0.8
        icp_x_spline = make_smoothing_spline(self.step_time_vector, self.icp_x_array[window_id, :], lam=lambda_param)
        icp_y_spline = make_smoothing_spline(self.step_time_vector, self.icp_y_array[window_id, :], lam=lambda_param)
        # icp_z_spline = make_smoothing_spline(self.step_time_vector, self.icp_z_array[window_id, :])
        # icp_roll_spline = make_smoothing_spline(self.step_time_vector, self.icp_roll_array[window_id, :])
        # icp_pitch_spline = make_smoothing_spline(self.step_time_vector, self.icp_pitch_array[window_id, :])
        # TODO : implement smoothing spline
        icp_yaw_spline = make_smoothing_spline(self.step_time_vector, self.unwrap_trajectory(self.icp_yaw_array[window_id, :]), lam=lambda_param)

        return np.array([icp_x_spline, icp_y_spline, icp_yaw_spline])

    def compute_interpolated_smoothed_icp_states(self):
        self.icp_x_interpolated_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.icp_y_interpolated_array = np.zeros((self.icp_y_array.shape[0], self.icp_y_array.shape[1]))
        # icp_z_interpolated_array = np.zeros((self.icp_z_array.shape[0], self.icp_z_array.shape[1]))
        # icp_roll_interpolated_array = np.zeros((self.icp_roll_array.shape[0], self.icp_roll_array.shape[1]))
        # icp_pitch_interpolated_array = np.zeros((self.icp_pitch_array.shape[0], self.icp_pitch_array.shape[1]))
        self.icp_yaw_interpolated_array = np.zeros((self.icp_yaw_array.shape[0], self.icp_yaw_array.shape[1]))
        for i in range(0, self.n_horizons):
            spline_array = self.icp_traj_as_smoothed_spline(i)
            self.icp_x_interpolated_array[i, :] = spline_array[0](self.step_time_vector)
            self.icp_y_interpolated_array[i, :] = spline_array[1](self.step_time_vector)
            # icp_z_interpolated_array[i, :] = spline_array[2](self.step_time_vector)
            # icp_roll_interpolated_array[i, :] = spline_array[3](self.step_time_vector)
            # icp_pitch_interpolated_array[i, :] = spline_array[4](self.step_time_vector)
            self.icp_yaw_interpolated_array[i, :] = spline_array[2](self.step_time_vector)

    def correct_interpolated_smoothed_icp_states_yaw(self):
        correction_rotmat = np.eye(2)
        self.icp_x_corrected_interpolated_array = np.zeros(self.icp_x_interpolated_array.shape)
        self.icp_y_corrected_interpolated_array = np.zeros(self.icp_y_interpolated_array.shape)
        for i in range(0, self.n_horizons):
            yaw_offset = np.arctan2(self.icp_x_interpolated_array[i, 5], self.icp_y_interpolated_array[i, 5])
            if yaw_offset < 0 and yaw_offset > -np.pi/2:
                correction_yaw_angle = np.pi/2 + yaw_offset
            if yaw_offset <= -np.pi/2:
                correction_yaw_angle = np.pi/2 + yaw_offset
            if yaw_offset >= 0 and yaw_offset < np.pi/2:
                correction_yaw_angle = -(np.pi/2 - yaw_offset)
            if yaw_offset >= np.pi/2:
                correction_yaw_angle = -(np.pi/2 - yaw_offset)
            # else:
            #     correction_yaw_angle = yaw_offset
            yaw_to_rotmat2d(correction_rotmat, correction_yaw_angle)
            for j in range(0, self.icp_x_array.shape[1]):
                offset_position = np.array([self.icp_x_interpolated_array[i,j], self.icp_y_interpolated_array[i,j]]).reshape(2,1)
                corrected_position = correction_rotmat @ offset_position
                self.icp_x_corrected_interpolated_array[i,j] = corrected_position[0]
                self.icp_y_corrected_interpolated_array[i,j] = corrected_position[1]

    def compute_icp_single_step_vels(self):
        icp_body_to_world_rotmat_2d = np.eye(2)
        icp_next_pose_body_2d = np.zeros((3, 1))
        icp_current_pose_body_2d = np.zeros((3, 1))
        self.icp_x_single_step_vels_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.icp_y_single_step_vels_array = np.zeros((self.icp_y_array.shape[0], self.icp_y_array.shape[1]))
        self.icp_yaw_single_step_vels_array = np.zeros((self.icp_yaw_array.shape[0], self.icp_yaw_array.shape[1]))

        for i in range(0, self.n_horizons):
            for j in range(0, self.icp_x_array.shape[1]-1):
                yaw_to_rotmat2d(icp_body_to_world_rotmat_2d, self.icp_yaw_interpolated_array[i, j])
                icp_world_to_body_rotmat_2d = np.linalg.inv(icp_body_to_world_rotmat_2d)
                icp_next_position_world_2d = np.array([self.icp_x_interpolated_array[i, j+1],
                                                     self.icp_y_interpolated_array[i, j+1]]).reshape(2,1)
                icp_next_position_body_2d = icp_world_to_body_rotmat_2d @ icp_next_position_world_2d
                icp_next_pose_body_2d[:2, 0] = icp_next_position_body_2d[:2, 0]
                icp_next_pose_body_2d[2, 0] = self.icp_yaw_interpolated_array[i, j+1]
                icp_current_position_world_2d = np.array([self.icp_x_interpolated_array[i, j],
                                                     self.icp_y_interpolated_array[i, j]]).reshape(2,1)
                icp_current_position_body_2d = icp_world_to_body_rotmat_2d @ icp_current_position_world_2d
                icp_current_pose_body_2d[:2, 0] = icp_current_position_body_2d[:2, 0]
                icp_current_pose_body_2d[2, 0] = self.icp_yaw_interpolated_array[i, j]
                icp_disp_body_2d = icp_next_pose_body_2d - icp_current_pose_body_2d
                # icp_disp_body_2d[2, 0] = wrap2pi(icp_disp_body_2d[2, 0])
                icp_vel_body_2d = (icp_disp_body_2d) / self.timestep
                self.icp_x_single_step_vels_array[i, j] = icp_vel_body_2d[0]
                self.icp_y_single_step_vels_array[i, j] = icp_vel_body_2d[1]
                # self.icp_yaw_single_step_vels_array[i, j] = icp_vel_body_2d[2]
            self.icp_x_single_step_vels_array[i, -1] = self.icp_x_single_step_vels_array[i, -2]
            self.icp_y_single_step_vels_array[i, -1] = self.icp_y_single_step_vels_array[i, -2]
            # self.icp_yaw_single_step_vels_array[i, -1] = self.icp_yaw_single_step_vels_array[i, -2]
            self.icp_yaw_single_step_vels_array[i, :] = self.imu_yaw_array[i, :]

    # TODO: compute body_vel_disturptions

    def compute_body_vel_disturptions(self):
        self.body_vel_disturption_x_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.body_vel_disturption_y_array = np.zeros((self.icp_y_array.shape[0], self.icp_y_array.shape[1]))
        self.body_vel_disturption_yaw_array = np.zeros((self.icp_yaw_array.shape[0], self.icp_yaw_array.shape[1]))

        for i in range(0, self.n_horizons):
            for j in range(0, self.icp_x_array.shape[1]):
                self.body_vel_disturption_x_array[i,j] = self.idd_body_vels_x_array[i, j] - self.icp_x_single_step_vels_array[i, j]
                self.body_vel_disturption_y_array[i,j] = self.idd_body_vels_y_array[i, j] - self.icp_y_single_step_vels_array[i, j]
                self.body_vel_disturption_yaw_array[i,j] = self.idd_body_vels_yaw_array[i, j] - self.icp_yaw_single_step_vels_array[i, j]
    def append_slip_elements_to_dataset(self):
        self.compute_transitory_vels()
        self.compute_transitory_body_vels()
        self.compute_interpolated_smoothed_icp_states()
        self.correct_interpolated_smoothed_icp_states_yaw()
        self.compute_icp_single_step_vels()
        self.compute_body_vel_disturptions()

        new_data_array = np.concatenate((self.transitory_left_vels_array, self.transitory_right_vels_array,
                                         self.idd_body_vels_x_array, self.idd_body_vels_y_array, self.idd_body_vels_yaw_array,
                                              self.icp_x_interpolated_array, self.icp_y_interpolated_array, self.icp_yaw_interpolated_array,
                                              self.icp_x_corrected_interpolated_array, self.icp_y_corrected_interpolated_array,
                                              self.icp_x_single_step_vels_array, self.icp_y_single_step_vels_array, self.icp_yaw_single_step_vels_array,
                                              self.body_vel_disturption_x_array, self.body_vel_disturption_y_array, self.body_vel_disturption_yaw_array),
                                             axis=1)

        new_cols = []

        str_transitory_vel_left_list = []
        str_transitory_vel_right_list = []
        for i in range(0, 40):
            str_transitory_vel_left_i = 'transitory_vel_left_' + str(i)
            str_transitory_vel_right_i = 'transitory_vel_right_' + str(i)
            str_transitory_vel_left_list.append(str_transitory_vel_left_i)
            str_transitory_vel_right_list.append(str_transitory_vel_right_i)
        new_cols.extend(str_transitory_vel_left_list)
        new_cols.extend(str_transitory_vel_right_list)

        str_idd_vel_x_list = []
        str_idd_vel_y_list = []
        str_idd_vel_yaw_list = []
        for i in range(0, 40):
            str_idd_vel_x_i = 'idd_vel_x_' + str(i)
            str_idd_vel_y_i = 'idd_vel_y_' + str(i)
            str_idd_vel_yaw_i = 'idd_vel_yaw_' + str(i)
            str_idd_vel_x_list.append(str_idd_vel_x_i)
            str_idd_vel_y_list.append(str_idd_vel_y_i)
            str_idd_vel_yaw_list.append(str_idd_vel_yaw_i)
        new_cols.extend(str_idd_vel_x_list)
        new_cols.extend(str_idd_vel_y_list)
        new_cols.extend(str_idd_vel_yaw_list)

        str_icp_interpolated_x_list = []
        str_icp_interpolated_y_list = []
        str_icp_interpolated_yaw_list = []
        for i in range(0, 40):
            str_icp_interpolated_x_i = 'icp_interpolated_x_' + str(i)
            str_icp_interpolated_y_i = 'icp_interpolated_y_' + str(i)
            str_icp_interpolated_yaw_i = 'icp_interpolated_yaw_' + str(i)
            str_icp_interpolated_x_list.append(str_icp_interpolated_x_i)
            str_icp_interpolated_y_list.append(str_icp_interpolated_y_i)
            str_icp_interpolated_yaw_list.append(str_icp_interpolated_yaw_i)
        new_cols.extend(str_icp_interpolated_x_list)
        new_cols.extend(str_icp_interpolated_y_list)
        new_cols.extend(str_icp_interpolated_yaw_list)

        str_icp_corrected_interpolated_x_list = []
        str_icp_corrected_interpolated_y_list = []
        for i in range(0, 40):
            str_icp_corrected_interpolated_x_i = 'icp_corrected_interpolated_x_' + str(i)
            str_icp_corrected_interpolated_y_i = 'icp_corrected_interpolated_y_' + str(i)
            str_icp_corrected_interpolated_x_list.append(str_icp_corrected_interpolated_x_i)
            str_icp_corrected_interpolated_y_list.append(str_icp_corrected_interpolated_y_i)
        new_cols.extend(str_icp_corrected_interpolated_x_list)
        new_cols.extend(str_icp_corrected_interpolated_y_list)

        str_icp_vel_x_list = []
        str_icp_vel_y_list = []
        str_icp_vel_yaw_list = []
        for i in range(0, 40):
            str_icp_vel_x_i = 'icp_vel_x_' + str(i)
            str_icp_vel_y_i = 'icp_vel_y_' + str(i)
            str_icp_vel_yaw_i = 'icp_vel_yaw_' + str(i)
            str_icp_vel_x_list.append(str_icp_vel_x_i)
            str_icp_vel_y_list.append(str_icp_vel_y_i)
            str_icp_vel_yaw_list.append(str_icp_vel_yaw_i)
        new_cols.extend(str_icp_vel_x_list)
        new_cols.extend(str_icp_vel_y_list)
        new_cols.extend(str_icp_vel_yaw_list)

        str_body_vel_disturption_x_list = []
        str_body_vel_disturption_y_list = []
        str_body_vel_disturption_yaw_list = []
        for i in range(0, 40):
            str_body_vel_disturption_x_i = 'body_vel_disturption_x_' + str(i)
            str_body_vel_disturption_y_i = 'body_vel_disturption_y_' + str(i)
            str_body_vel_disturption_yaw_i = 'body_vel_disturption_yaw_' + str(i)
            str_body_vel_disturption_x_list.append(str_body_vel_disturption_x_i)
            str_body_vel_disturption_y_list.append(str_body_vel_disturption_y_i)
            str_body_vel_disturption_yaw_list.append(str_body_vel_disturption_yaw_i)
        new_cols.extend(str_body_vel_disturption_x_list)
        new_cols.extend(str_body_vel_disturption_y_list)
        new_cols.extend(str_body_vel_disturption_yaw_list)

        self.data[new_cols] = new_data_array

        self.data.to_pickle(self.export_dataset_path)

        return None
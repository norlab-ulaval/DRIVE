import numpy as np
import pandas as pd
from scipy.interpolate import make_smoothing_spline

from drive.util.util_func import *
from drive.util.transform_algebra import *
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive

class AccelerationDatasetParser:
    def __init__(self, slip_dataset, wheel_radius, baseline, rate, imu_angle):
        self.data = slip_dataset
        self.n_horizons = len(self.data)
        self.rate = rate
        self.timestep = 1 / self.rate
        self.step_time_vector = np.linspace(0, 2, 40)

        self.wheel_radius = wheel_radius
        self.baseline = baseline

        self.ideal_diff_drive = Ideal_diff_drive(self.wheel_radius, self.baseline, self.timestep)
        self.k = np.array([self.wheel_radius, self.baseline])

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

        str_transitory_vel_left_list = []
        str_transitory_vel_right_list = []
        for i in range(0, 40):
            str_transitory_vel_left_i = 'transitory_vel_left_' + str(i)
            str_transitory_vel_right_i = 'transitory_vel_right_' + str(i)
            str_transitory_vel_left_list.append(str_transitory_vel_left_i)
            str_transitory_vel_right_list.append(str_transitory_vel_right_i)
        self.transitory_left_vels_array = self.data[str_transitory_vel_left_i].to_numpy()
        self.transitory_right_vels_array = self.data[str_transitory_vel_right_i].to_numpy()

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
        self.idd_body_vels_x_array = self.data[str_idd_vel_x_list].to_numpy()
        self.idd_body_vels_y_array = self.data[str_idd_vel_y_list].to_numpy()
        self.idd_body_vels_yaw_array = self.data[str_idd_vel_yaw_list].to_numpy()

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
        self.icp_x_interpolated_array = self.data[str_icp_interpolated_x_list].to_numpy()
        self.icp_y_interpolated_array = self.data[str_icp_interpolated_y_list].to_numpy()
        self.icp_yaw_interpolated_array = self.data[str_icp_interpolated_yaw_list].to_numpy()

        str_icp_corrected_interpolated_x_list = []
        str_icp_corrected_interpolated_y_list = []
        for i in range(0, 40):
            str_icp_corrected_interpolated_x_i = 'icp_corrected_interpolated_x_' + str(i)
            str_icp_corrected_interpolated_y_i = 'icp_corrected_interpolated_y_' + str(i)
            str_icp_corrected_interpolated_x_list.append(str_icp_corrected_interpolated_x_i)
            str_icp_corrected_interpolated_y_list.append(str_icp_corrected_interpolated_y_i)
        self.icp_x_corrected_interpolated_array = self.data[str_icp_corrected_interpolated_x_list].to_numpy()
        self.icp_y_corrected_interpolated_array = self.data[str_icp_corrected_interpolated_y_list].to_numpy()

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
        self.icp_x_single_step_vels_array = self.data[str_icp_vel_x_list].to_numpy()
        self.icp_y_single_step_vels_array = self.data[str_icp_vel_y_list].to_numpy()
        self.icp_yaw_single_step_vels_array = self.data[str_icp_vel_yaw_list].to_numpy()

        str_imu_accel_x_list = []
        str_imu_accel_y_list = []
        str_imu_accel_z_list = []
        for i in range(0, 40):
            str_imu_accel_x_i = 'imu_acceleration_x_' + str(i)
            str_imu_accel_x_list.append(str_imu_accel_x_i)
            str_imu_accel_y_i = 'imu_acceleration_y_' + str(i)
            str_imu_accel_y_list.append(str_imu_accel_y_i)
            str_imu_accel_z_i = 'imu_acceleration_z_' + str(i)
            str_imu_accel_z_list.append(str_imu_accel_z_i)
        self.imu_accel_x_array = self.data[str_imu_accel_x_list].to_numpy()
        self.imu_accel_y_array = self.data[str_imu_accel_y_list].to_numpy()
        self.imu_accel_z_array = self.data[str_imu_accel_z_list].to_numpy()

    def compute_idd_accelerations(self):
        self.idd_body_accelerations_x_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.idd_body_accelerations_y_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.idd_body_accelerations_yaw_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        for i in range(0, self.n_horizons):
            self.idd_body_accelerations_x_array[i, :] = np.gradient(self.idd_body_vels_x_array[i, :])
            self.idd_body_accelerations_y_array[i, :] = np.gradient(self.idd_body_vels_y_array[i, :])
            self.idd_body_accelerations_yaw_array[i, :] = np.gradient(self.idd_body_vels_yaw_array[i, :])
        return None

    def smooth_icp_velocities(self):
        lambda_param = 0.8
        self.icp_x_single_step_vels_smoothed_array = np.zeros(((self.icp_x_array.shape[0], self.icp_x_array.shape[1])))
        self.icp_y_single_step_vels_smoothed_array = np.zeros(((self.icp_y_array.shape[0], self.icp_y_array.shape[1])))
        self.icp_yaw_single_step_vels_smoothed_array = np.zeros(((self.icp_yaw_array.shape[0], self.icp_yaw_array.shape[1])))
        for i in range(0, self.n_horizons):
            icp_vel_x_spline = make_smoothing_spline(self.step_time_vector, self.icp_x_single_step_vels_array[i, :], lam=lambda_param)
            self.icp_x_single_step_vels_smoothed_array[i, :] = icp_vel_x_spline(self.step_time_vector)
            icp_vel_y_spline = make_smoothing_spline(self.step_time_vector, self.icp_y_single_step_vels_array[i, :], lam=lambda_param)
            self.icp_y_single_step_vels_smoothed_array[i, :] = icp_vel_y_spline(self.step_time_vector)
            icp_vel_yaw_spline = make_smoothing_spline(self.step_time_vector, self.icp_yaw_single_step_vels_array[i, :], lam=lambda_param)
            self.icp_yaw_single_step_vels_smoothed_array[i, :] = icp_vel_yaw_spline(self.step_time_vector)
        return None
    def compute_icp_accelerations(self):
        self.smooth_icp_velocities()
        self.icp_x_single_step_accelerations_array = np.zeros((self.icp_x_array.shape[0], self.icp_x_array.shape[1]))
        self.icp_y_single_step_accelerations_array = np.zeros((self.icp_y_array.shape[0], self.icp_y_array.shape[1]))
        self.icp_yaw_single_step_accelerations_array = np.zeros((self.icp_yaw_array.shape[0], self.icp_yaw_array.shape[1]))
        for i in range(0, self.n_horizons):
            self.icp_x_single_step_accelerations_array[i, :] = np.gradient(self.icp_x_single_step_vels_smoothed_array[i, :])
            self.icp_y_single_step_accelerations_array[i, :] = np.gradient(self.icp_y_single_step_vels_smoothed_array[i, :])
            self.icp_yaw_single_step_accelerations_array[i, :] = np.gradient(self.icp_yaw_single_step_vels_smoothed_array[i, :])
        return None

    def remove_gravity_vector_from_imu(self):
        # TODO : use this function to extract gravity vector from icp orientation, then remove from imu gravity_vector = euler_to_rotmat(init_state[2:])[:, 2:]
        # TODO: use imu angular vel to derive angular acceleration
        self.imu_accel_x_no_grav_array = np.zeros((self.imu_accel_x_array.shape[0], self.imu_accel_x_array.shape[1]))
        self.imu_accel_y_no_grav_array = np.zeros((self.imu_accel_y_array.shape[0], self.imu_accel_y_array.shape[1]))
        imu_transform = np.eye(4)
        for i in range(0, self.n_horizons):
            for j in range(0, self.imu_accel_x_array.shape[1]):
                roll_angle = self.icp_roll_array[i, j]
                pitch_angle = self.icp_pitch_array[i, j] + self.imu_angle
                euler_to_transform(np.array([roll_angle, pitch_angle, 0]), imu_transform)
                gravity_vector = imu_transform @ np.array([0, 0, 9.8, 1.0])
                self.imu_accel_x_no_grav_array[i,j] = self.imu_accel_x_array[i,j] - gravity_vector[0]
                self.imu_accel_y_no_grav_array[i,j] = self.imu_accel_y_array[i,j] - gravity_vector[1]
        return None

    def append_acceleration_elements_to_dataset(self):
        # self.compute_transitory_vels()
        # self.compute_transitory_body_vels()
        # self.compute_interpolated_smoothed_icp_states()
        # self.correct_interpolated_smoothed_icp_states_yaw()
        # self.compute_icp_single_step_vels()
        # self.compute_body_vel_disturptions()
        self.compute_idd_accelerations()
        self.compute_icp_accelerations()
        self.remove_gravity_vector_from_imu()

        new_data_array = np.concatenate((self.idd_body_accelerations_x_array,
                                         self.idd_body_accelerations_y_array,
                                         self.idd_body_accelerations_yaw_array,
                                         self.icp_x_single_step_accelerations_array,
                                         self.icp_y_single_step_accelerations_array,
                                         self.icp_yaw_single_step_accelerations_array,
                                         self.imu_accel_x_no_grav_array,
                                         self.imu_accel_y_no_grav_array),
                                        axis=1)

        new_cols = []

        str_idd_acceleration_x_list = []
        str_idd_acceleration_y_list = []
        str_idd_acceleration_yaw_list = []
        for i in range(0, 40):
            str_idd_acceleration_x_i = 'idd_acceleration_x_' + str(i)
            str_idd_acceleration_y_i = 'idd_acceleration_y_' + str(i)
            str_idd_acceleration_yaw_i = 'idd_acceleration_yaw_' + str(i)
            str_idd_acceleration_x_list.append(str_idd_acceleration_x_i)
            str_idd_acceleration_y_list.append(str_idd_acceleration_y_i)
            str_idd_acceleration_yaw_list.append(str_idd_acceleration_yaw_i)
        new_cols.extend(str_idd_acceleration_x_list)
        new_cols.extend(str_idd_acceleration_y_list)
        new_cols.extend(str_idd_acceleration_yaw_list)

        str_icp_acceleration_x_list = []
        str_icp_acceleration_y_list = []
        str_icp_acceleration_yaw_list = []
        for i in range(0, 40):
            str_icp_acceleration_x_i = 'icp_acceleration_x_' + str(i)
            str_icp_acceleration_y_i = 'icp_acceleration_y_' + str(i)
            str_icp_acceleration_yaw_i = 'icp_acceleration_yaw_' + str(i)
            str_icp_acceleration_x_list.append(str_icp_acceleration_x_i)
            str_icp_acceleration_y_list.append(str_icp_acceleration_y_i)
            str_icp_acceleration_yaw_list.append(str_icp_acceleration_yaw_i)
        new_cols.extend(str_icp_acceleration_x_list)
        new_cols.extend(str_icp_acceleration_y_list)
        new_cols.extend(str_icp_acceleration_yaw_list)

        str_imu_acceleration_x_list = []
        str_imu_acceleration_y_list = []
        for i in range(0, 40):
            str_imu_acceleration_x_i = 'imu_acceleration_x_' + str(i)
            str_imu_acceleration_y_i = 'imu_acceleration_y_' + str(i)
            str_imu_acceleration_x_list.append(str_imu_acceleration_x_i)
            str_imu_acceleration_y_list.append(str_imu_acceleration_y_i)
        new_cols.extend(str_imu_acceleration_x_list)
        new_cols.extend(str_imu_acceleration_y_list)

        self.data[new_cols] = new_data_array

        return self.data
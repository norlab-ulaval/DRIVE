import numpy as np
import pandas as pd
from scipy.interpolate import make_smoothing_spline
from scipy.signal import butter, filtfilt
from drive.util.util_func import *
from drive.util.transform_algebra import *
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive
from drive.model_training.models.powertrain.bounded_powertrain import Bounded_powertrain
from extractors import *
from animate_time_constant import  produce_video,produce_video_traj,produce_video_traj_quiver
import pathlib


from first_order_model import *
import matplotlib.animation as animation
from matplotlib.backend_bases import KeyEvent

import tqdm
class SlipDatasetParser:
    def __init__(self, dataset, experiment_path, wheel_radius, baseline, min_wheel_vel, max_wheel_vel, rate,n_window=3):
        self.data = dataset
        self.n_horizons = len(self.data)
        self.rate = rate
        self.timestep = 1 / self.rate
        self.step_time_vector = np.linspace(0, 2, 40)
        self.step_time_vector_whole_cmd = np.linspace(0,6,120)
        self.wheel_radius = wheel_radius
        self.baseline = baseline
        self.nb_iteration_by_windows = int(column_type_extractor(dataset,"cmd_left").shape[1])
        self.n_windows = n_window

        self.ideal_diff_drive = Ideal_diff_drive(wheel_radius, baseline, self.timestep)
        self.k = np.array([self.wheel_radius, self.baseline])

        path_to_left_params = experiment_path / 'powertrain'/'powertrain_training_left.npy'
        #print("\n"*3,path_to_left_params,"\n"*3)
        bounded_powertrain_left_params = np.load(path_to_left_params)
        self.bounded_powertrain_left = Bounded_powertrain(min_wheel_vel, max_wheel_vel, bounded_powertrain_left_params[0],
                                                          bounded_powertrain_left_params[1], self.timestep)
        path_to_right_params = experiment_path / 'powertrain'/'powertrain_training_right.npy'
        #print("\n"*3,path_to_right_params,"\n"*3)
        bounded_powertrain_right_params = np.load(path_to_right_params)
        self.bounded_powertrain_right = Bounded_powertrain(min_wheel_vel, max_wheel_vel, bounded_powertrain_right_params[0],
                                                          bounded_powertrain_right_params[1], self.timestep)


        self.path_to_model_training_datasets =experiment_path.parent.parent/ "model_training_datasets" 
        self.path_to_model_training_datasets_video = self.path_to_model_training_datasets/"video"
        if self.path_to_model_training_datasets_video.is_dir() ==False:
            self.path_to_model_training_datasets_video.mkdir()


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

        self.step_icp_x_array = column_type_extractor(self.data,"step_frame_icp_x")
        self.step_icp_y_array = column_type_extractor(self.data,"step_frame_icp_y")
        self.step_icp_yaw_array = column_type_extractor(self.data,"step_frame_icp_yaw")

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
    

    def smmoth_using_moving_average(self, window_id):
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

    def icp_traj_as_smoothed_spline_on_a_whole_cmd(self,window_id,time,x,y,yaw):
        lambda_param = 0.6
        icp_x_spline = make_smoothing_spline(time, x[window_id, :], lam=lambda_param)
        icp_y_spline = make_smoothing_spline(time, y[window_id, :], lam=lambda_param)
        # icp_z_spline = make_smoothing_spline(self.step_time_vector, self.icp_z_array[window_id, :])
        # icp_roll_spline = make_smoothing_spline(self.step_time_vector, self.icp_roll_array[window_id, :])
        # icp_pitch_spline = make_smoothing_spline(self.step_time_vector, self.icp_pitch_array[window_id, :])
        # TODO : implement smoothing spline
        icp_yaw_spline = make_smoothing_spline(time, self.unwrap_trajectory(yaw[window_id, :]), lam=lambda_param)

        return np.array([icp_x_spline, icp_y_spline, icp_yaw_spline])
    
    def compute_interpolated_smoothed_icp_states_on_a_whole_cmd(self,x,y,yaw,time):
        #self.icp_x_array_reshape, self.icp_y_array_reshape, self.icp_yaw_array_reshape =  reshape_into_6sec_windows(self.step_icp_x_array), reshape_into_6sec_windows(self.step_icp_y_array), reshape_into_6sec_windows(self.step_icp_yaw_array)
        # icp_x_interpolated_array, icp_y_interpolated_array, icp_yaw_interpolated_array
        
        icp_x_interpolated= np.zeros(x.shape)
        icp_y_interpolated= np.zeros(y.shape)

        icp_yaw_interpolated = np.zeros(yaw.shape)
        for i in range(0, x.shape[1]):
            spline_array = self.icp_traj_as_smoothed_spline_on_a_whole_cmd(i,time,x,y,yaw)
            icp_x_interpolated[i, :] = spline_array[0](time)
            icp_y_interpolated[i, :] = spline_array[1](time)
            icp_yaw_interpolated[i, :] = spline_array[2](time)
        
        return icp_x_interpolated,icp_y_interpolated, icp_yaw_interpolated
    def icp_traj_smoothed_butter_on_a_whole_cmd(self,x,y,yaw,time):
        #self.icp_x_array_reshape, self.icp_y_array_reshape, self.icp_yaw_array_reshape =  reshape_into_6sec_windows(self.step_icp_x_array), reshape_into_6sec_windows(self.step_icp_y_array), reshape_into_6sec_windows(self.step_icp_yaw_array)
        # icp_x_interpolated_array, icp_y_interpolated_array, icp_yaw_interpolated_array
        fc = 0.90
        fs = 20
        Wn =  fc #*2*np.pi#angular frequency in rad/s of the cut off frequency
        btype="lowpass"
        output = "ba"
        order=5
        num,denom = butter(order,Wn,btype=btype,output=output,fs=fs)
    
        icp_x_interpolated= np.zeros(x.shape)
        icp_y_interpolated= np.zeros(y.shape)
        icp_yaw_interpolated = np.zeros(yaw.shape)
        
        for window_id in range(0, x.shape[0]):
            icp_x_interpolated[window_id,:] = filtfilt(num,denom,x[window_id,:])
            icp_y_interpolated[window_id,:] = filtfilt(num,denom, y[window_id,:])
            icp_yaw_interpolated[window_id,:] = filtfilt(num,denom,yaw[window_id,:])
        
        return icp_x_interpolated,icp_y_interpolated, icp_yaw_interpolated
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
    
    def compute_icp_single_step_vels_whole_cmd(self,x,y,yaw):        
        
        icp_body_to_world_rotmat_2d = np.eye(2)
        icp_next_pose_body_2d = np.zeros((3, 1))
        icp_current_pose_body_2d = np.zeros((3, 1))

        
        x_vels = np.zeros(x.shape)
        y_vels = np.zeros(x.shape)
        yaw_vels = np.zeros(x.shape)

        imu_yaw_reshape = reshape_into_6sec_windows(self.imu_yaw_array)
        for i in range(0, x.shape[0]):
            for j in range(0, x.shape[1]-1):
                yaw_to_rotmat2d(icp_body_to_world_rotmat_2d, yaw[i, j])
                
                icp_world_to_body_rotmat_2d = np.linalg.inv(icp_body_to_world_rotmat_2d)

                icp_next_position_world_2d = np.array([x[i, j+1], y[i, j+1]]).reshape(2,1)
                icp_next_position_body_2d = icp_world_to_body_rotmat_2d @ icp_next_position_world_2d
                icp_next_pose_body_2d[:2, 0] = icp_next_position_body_2d[:2, 0]

                

                
                #icp_next_pose_body_2d[2, 0] = yaw[i, j+1]

                icp_current_position_world_2d = np.array([x[i, j],
                                                    y[i, j]]).reshape(2,1)
                icp_current_position_body_2d = icp_world_to_body_rotmat_2d @ icp_current_position_world_2d

                icp_current_pose_body_2d[:2, 0] = icp_current_position_body_2d[:2, 0]
                icp_current_pose_body_2d[2, 0] = yaw[i, j]
                icp_disp_body_2d = icp_next_pose_body_2d - icp_current_pose_body_2d
                icp_vel_body_2d = (icp_disp_body_2d) / self.timestep
                x_vels[i, j] = icp_vel_body_2d[0]
                y_vels[i, j] = icp_vel_body_2d[1]
                # self.yaw_vels[i, j] = icp_vel_body_2d[2]
                icp_next_orientation = Rotation.from_euler("z",yaw[i,j+1])

                current_rotation_matrix= np.identity(3) 
                current_rotation_matrix[:2,:2] =  icp_body_to_world_rotmat_2d
                tf_world_to_body_orientation = Rotation.from_matrix(np.linalg.inv(current_rotation_matrix))
                delta_yaw = (tf_world_to_body_orientation * icp_next_orientation).as_euler("zyx")[0] 
                yaw_vels[i, j] = delta_yaw/self.timestep
                
            x_vels[i, -1] = x_vels[i, -2]
            y_vels[i, -1] = y_vels[i, -2]
            yaw_vels[i, -1] = yaw_vels[i, -2]
            #yaw_vels[i, :] = imu_yaw_reshape[i, :] # TODO : validate if I use the derivative of the ICP for yaw or IMU. 
        return x_vels,y_vels,yaw_vels

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
    
        
    def compute_time_cst_gain_delay_step_operation_points(self, df_slip,name_column_interest, name_column_cmd,freq, n_window = 3,verbose=False,produce_video_now=False,video_saving_path=""):
        """Compute the time_constant, time_delay, gains.

        Args:
            df_slip (df): _description_
            name_column_interest (str): _description_
            name_column_cmd (str): _description_
            n_window (int, optional): _description_. Defaults to 3.
            verbose (bool, optional): _description_. Defaults to False.
        """
        column_of_interest = column_type_extractor(df_slip, name_column_interest,verbose=verbose)
        column_interest_reshape = reshape_into_6sec_windows(column_of_interest, n_window=n_window)
        
        if name_column_cmd =="":
            cmd_of_interest_reshape = np.zeros(column_interest_reshape.shape)
            cmd_of_interest = np.zeros(column_of_interest.shape)
        else:
            cmd_of_interest = column_type_extractor(df_slip, name_column_cmd,verbose=verbose)
            cmd_of_interest_reshape = reshape_into_6sec_windows(cmd_of_interest, n_window=n_window)
            
        # To compute using the precedent steps
        #mask = df_slip["precedent_window_operation_point_mask"].to_numpy().reshape((column_of_interest.shape[0],1))
        #operation_point_interest,steps_cmd_interest_reshape = compute_operation_points_and_step_using_mask(column_of_interest,cmd_of_interest,mask,self.n_step)
        operation_point_interest,steps_cmd_interest_reshape = compute_operation_points_and_step(column_of_interest,cmd_of_interest)
        
        nb_timestamp = self.n_windows * self.nb_iteration_by_windows
        max_time_bounds = freq * nb_timestamp
        #print("max time bounds",max_time_bounds)
        #
        time_axis = create_time_axe(freq,nb_timestamp)
        first_order_model = FirstOrderModelWithDelay(1,1,freq)

        step_y_interest_centered = column_interest_reshape - operation_point_interest
        gains_computed, time_constants_computed, time_delay_computed,predictions = first_order_model.train_all_calib_state(time_axis,steps_cmd_interest_reshape,step_y_interest_centered,operation_point_interest,max_time_bounds)


        predictions_error = np.sum(np.abs(predictions-column_of_interest.reshape(predictions.shape))) 
        # Create a second dataframe 
        time_constant_error_flag = np.squeeze((np.abs(time_constants_computed)> nb_timestamp * freq ) | (time_constants_computed<0))
        time_delay_error_flag =np.squeeze((np.abs(time_delay_computed)> nb_timestamp * freq ) | (time_delay_computed<0))

        valid_mask = np.logical_or(time_delay_error_flag,  time_constant_error_flag)

        predictions_centered = predictions-operation_point_interest
    
        if produce_video_now:
            #produce_video(predictions,time_axis,cmd_of_interest_reshape,gt_of_interest_reshpae ,names=["cmd","model","measurement"],video_saving_path="")
            produce_video(predictions_centered,time_axis,steps_cmd_interest_reshape.reshape((steps_cmd_interest_reshape.shape[0],1)),step_y_interest_centered ,
                        names=[name_column_cmd,name_column_interest],video_saving_path=self.path_to_model_training_datasets_video)
        
        dico_predictions = {}
        for i in range(predictions_centered.shape[1]):
            dico_predictions[name_column_interest+f"_predictions_{i}"] = predictions[:,i]

        column_2_add = {name_column_interest+"_gains":gains_computed,
                    name_column_interest+"_time_constants":time_constants_computed,
                    name_column_interest+"_time_delay":time_delay_computed,
                    #name_column_interest+"_time_constant_problematic_computation":time_constant_error_flag,
                    name_column_interest+"_time_delay_problematic_computation":time_delay_error_flag,
                    #name_column_interest+"_valid_mask_tc":valid_mask,
                    name_column_interest+"_operation_points": np.squeeze(operation_point_interest),
                    name_column_interest+"_steps": np.squeeze(steps_cmd_interest_reshape),
                    name_column_interest+"_time_constants_to_show":time_constants_computed + time_delay_computed
                    }
        
        column_2_add.update(dico_predictions)
        
        return column_2_add


    def compute_time_constant_wheel_and_body(self, df_slip,rate, verbose = False,produce_video_now = False,video_saving_path=""):

        list_of_cmd_vector = ['cmd_left', "cmd_right","cmd_body_vel_x","cmd_body_vel_yaw",""]
        list_column_interest = ["left_wheel_vel","right_wheel_vel","step_frame_vx" , "step_frame_vyaw", "step_frame_vy" ]        
        dico_column_2_add = {}
        #print_column_unique_column(df_slip)
        for cmd_name, column_of_interest in zip(list_of_cmd_vector,list_column_interest):
            
            print("     "+"Computing the time constant of"+column_of_interest )
            dico_new_column = self.compute_time_cst_gain_delay_step_operation_points(df_slip,column_of_interest, cmd_name,1/rate, n_window = 3,verbose=verbose,produce_video_now=produce_video_now,video_saving_path=video_saving_path)
            dico_column_2_add.update(dico_new_column)

        if verbose:
            for key,value in dico_column_2_add.items():
                print(f'{key} shape = {value.shape}')
            
        return dico_column_2_add

    def append_slip_elements_to_dataset(self,compute_by_whole_step = False,debug=False,smooth=True):
        self.compute_transitory_vels() # Compute the vel of the wheels using the model
        self.compute_transitory_body_vels() # Compute idd velocity 
        self.compute_interpolated_smoothed_icp_states() # Smooth trajectory
        self.correct_interpolated_smoothed_icp_states_yaw()
        self.compute_icp_single_step_vels()
        self.compute_body_vel_disturptions()
        df_cmd = self.recompute_cmd_vel_body()
        self.data = pd.concat((self.data,df_cmd),axis=1)

       
        if debug:
            # For debug purpose
            n_window=3
            self.icp_x_array_reshape, self.icp_y_array_reshape, self.icp_yaw_array_reshape = self.reshape_icp_in_6_sec_traj(self.icp_x_array,
                                                                                                            self.icp_y_array,self.icp_yaw_array,n_window)  
            reshape_corrected =  self.reshape_icp_in_6_sec_traj(self.icp_x_corrected_interpolated_array,self.icp_y_corrected_interpolated_array,self.icp_yaw_interpolated_array,n_window)  
            reshape_interpolated = self.reshape_icp_in_6_sec_traj(self.icp_x_interpolated_array,self.icp_y_interpolated_array,self.icp_yaw_interpolated_array,n_window)  
        
            yaw_in_absolute_reference = self.imu_yaw_array * self.rate

            produce_video_traj_quiver(self.icp_x_array_reshape, self.icp_y_array_reshape,
                            reshape_interpolated[0],reshape_interpolated[1],
                            reshape_corrected[0],reshape_corrected[1],yaw_in_absolute_reference,
                            names=["trajectory_icp"],video_saving_path=self.path_to_model_training_datasets_video)

            
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

        
        list_commun_type = ["pwrtrain_conscient_cmd_body_vel_x","pwrtrain_conscient_cmd_body_vel_y","pwrtrain_conscient_cmd_body_vel_yaw"]
        new_data_array = np.concatenate((new_data_array,self.idd_body_vels_x_array,self.idd_body_vels_y_array,
                                        self.idd_body_vels_yaw_array),axis=1)
        list_body_names = []
        for commun_name in list_commun_type:
            for i in range(self.idd_body_vels_x_array.shape[1]):
                list_body_names.append(commun_name+f"_{i}")
                #print("",np.unique(self.idd_body_vels_x_array[3,:]))

        new_cols += list_body_names
        
        data_temp = pd.DataFrame(data=new_data_array,columns=new_cols)
        self.data = pd.concat((self.data,data_temp),axis=1)


        ## Compute on the whole step instead of 2 second windows.
        if compute_by_whole_step:
            self.step_icp_x_array_reshape = reshape_into_6sec_windows(self.step_icp_x_array)
            self.step_icp_y_array_reshape = reshape_into_6sec_windows(self.step_icp_y_array) 
            
            icp_yaw_wrapped = reshape_into_6sec_windows(self.step_icp_yaw_array)
            self.step_icp_yaw_array_reshape  = np.zeros_like(icp_yaw_wrapped)

            for i in range(self.step_icp_yaw_array_reshape .shape[0]):
                self.step_icp_yaw_array_reshape [i,:] = np.unwrap(icp_yaw_wrapped[i,:])

            if smooth:
                step_icp_interpolated=self.icp_traj_smoothed_butter_on_a_whole_cmd(self.step_icp_x_array_reshape,
                                                        self.step_icp_y_array_reshape,self.step_icp_yaw_array_reshape,
                                                        self.step_time_vector_whole_cmd) # Smooth trajectory

            else:
                # Reshape the smoothed icp in 2 second windows 
                self.step_icp_x_interpolated = self.step_icp_x_array
                self.step_icp_y_interpolated = self.step_icp_y_array
                self.step_icp_yaw_interpolated = self.step_icp_yaw_array

            step_vels_x_y_yaw = self.compute_icp_single_step_vels_whole_cmd(step_icp_interpolated[0],
                                                                                step_icp_interpolated[1],
                                                                                step_icp_interpolated[2])
            
            column_2_add = [reshape_into_2sec_windows(step_icp_interpolated[0]),reshape_into_2sec_windows(step_vels_x_y_yaw[0]),
                            reshape_into_2sec_windows(step_icp_interpolated[1]),reshape_into_2sec_windows(step_vels_x_y_yaw[1]),
                            reshape_into_2sec_windows(step_icp_interpolated[2]),reshape_into_2sec_windows(step_vels_x_y_yaw[2])]
            stack_step_frame = np.hstack(column_2_add)
            column_type = ["x","y","yaw"]
            all_step_columns = []
            for column in column_type: 
                columns1 = ["step_frame_interpolated_icp_"+column+f"_{i}" for i in range(column_2_add[0].shape[1])]
                columns2 = ["step_frame_v"+column+f"_{i}" for i in range(column_2_add[0].shape[1])]
                all_step_columns += (columns1 + columns2) 

            #print(all_step_columns)
            
            df = pd.DataFrame(data=stack_step_frame,columns=all_step_columns)
            #print_column_unique_column(df)
            self.data = pd.concat((self.data,df),axis=1)

        ## Compute the 
        return self.data
    
    

    ### Create the dataframe for diamond shape
    def recompute_cmd_vel_body(self):

        cmd_left = column_type_extractor(self.data, 'cmd_left',verbose=False)
        cmd_right = column_type_extractor(self.data, 'cmd_right',verbose=False)
        
        cmd_body_x = np.zeros_like(cmd_left)
        cmd_body_omega = np.zeros_like(cmd_left)

        for i in range(cmd_body_x.shape[0]):
            cmd_vel = np.vstack((cmd_left[i,:],cmd_right[i,:]))
            body_vel = compute_body_vel_IDD(cmd_vel , robot='warthog-wheel')
            cmd_body_x[i,:] = body_vel[0,:]
            cmd_body_omega[i,:] = body_vel[1,:]

        self.cmd_body_x = cmd_body_x
        self.cmd_body_omega = cmd_body_omega
        self.cmd_body_y = np.zeros_like(cmd_body_omega)
        dico_2_add = {}
        
        column_type = ["cmd_body_vel_x","cmd_body_vel_yaw","cmd_body_vel_y"]
        all_step_columns = []
        for column in column_type: 
            columns = [column+f"_{i}" for i in range(cmd_body_omega.shape[1])]
            all_step_columns += columns

        df = pd.DataFrame(data=np.concatenate((self.cmd_body_x,self.cmd_body_omega,self.cmd_body_y),axis=1),columns=all_step_columns)

        return df



    def create_dataframe_for_diamond_shape_graph(self,terrain,robot,traction,produce_video_now=False,verbose=False):
        
        
        df_slip_dataset = self.data 
        # Steady state keeping
        df_steady_state = df_slip_dataset.loc[df_slip_dataset['steady_state_mask'] == 1]

        #print(df_steady_state)

        # Remove the first window
        df_last_window = df_steady_state.drop_duplicates(subset=['steady_state_mask','calib_step'],keep='last')
        #print(df_last_window.shape)

        #print(df_last_window)
        
        cmd_left = np.mean(column_type_extractor(df_last_window, 'cmd_left',verbose=False),axis=1)
        cmd_right = np.mean(column_type_extractor(df_last_window, 'cmd_right',verbose=False),axis=1)
        cmd_vel = np.vstack((cmd_left,cmd_right))
        body_vel = compute_body_vel_IDD(cmd_vel , robot='warthog-wheel')

        
        
        ##
        icp_vel_x = np.mean(column_type_extractor(df_last_window, 'icp_vel_x',verbose=False),axis=1)
        icp_vel_y = np.mean(column_type_extractor(df_last_window, 'icp_vel_y',verbose=False),axis=1)
        icp_vel_yaw = np.mean(column_type_extractor(df_last_window, 'icp_vel_yaw',verbose=False),axis=1)
        body_vel_icp_mean = np.vstack((icp_vel_x,icp_vel_yaw))

        body_slip_x = np.mean(reshape_into_6sec_windows(self.body_vel_disturption_x_array)[:,-self.nb_iteration_by_windows:],axis=1)
        body_stimestelip_yaw = np.mean(reshape_into_6sec_windows(self.body_vel_disturption_yaw_array)[:,-self.nb_iteration_by_windows:],axis=1)
        body_stimestelip_y = np.mean(reshape_into_6sec_windows(self.body_vel_disturption_y_array)[:,-self.nb_iteration_by_windows:],axis=1)
        #raw_icp_vel_x_mean = np.mean(column_type_extractor(df_last_window, 'raw_icp_vel_x',verbose=False),axis=1)
        #raw_icp_vel_yaw_mean = np.mean(column_type_extractor(df_last_window, 'raw_icp_vel_yaw',verbose=False),axis=1)


        odom_speed_l = np.mean(column_type_extractor(df_last_window, 'left_wheel_vel',verbose=False),axis=1)
        odom_speed_right = np.mean(column_type_extractor(df_last_window, 'right_wheel_vel',verbose=False),axis=1)      

        size = odom_speed_l.shape[0]
        dictionnary_ = {"terrain": [terrain] * size,
                        "robot": [robot] * size,
                        "traction": [traction] * size,
                        #"offline_localisation":[offline_localisation]*size,
                        "cmd_left_wheels":cmd_left,
                        "cmd_right_wheels":cmd_right,
                        "cmd_body_x_lwmean":body_vel[0,:],
                        "cmd_body_yaw_lwmean":body_vel[1,:],
                        "icp_vel_x_smoothed":icp_vel_x,
                        "icp_vel_yaw_smoothed":icp_vel_yaw,
                        #"raw_icp_vel_x_mean":raw_icp_vel_x_mean,
                        #"raw_icp_vel_yaw_mean":raw_icp_vel_yaw_mean,
                        "odom_speed_left_wheels": odom_speed_l,
                        "odom_speed_right_wheels": odom_speed_right,
                        "slip_body_x_ss": body_slip_x,
                        "slip_body_yaw_ss": body_stimestelip_yaw,
                        "slip_body_y_ss": body_stimestelip_y,
                        "slip_wheel_left_ss": cmd_left-odom_speed_l,
                        "slip_wheel_right_ss": cmd_right-odom_speed_right,
                        }

        

        df = pd.DataFrame.from_dict(dictionnary_)

        
        ## Compute time and body time constant
        
        dico_2_add = self.compute_time_constant_wheel_and_body(df_slip_dataset,self.rate , verbose = False,produce_video_now = produce_video_now)
        df_temp  = pd.DataFrame.from_dict(dico_2_add)
        
        
        df = pd.concat((df,df_temp),axis=1)

        path = self.path_to_model_training_datasets/"steady_state_results.pkl"
        
        df.to_pickle(str(path))

        
        if verbose ==True:
            print(f"cmd_shape {cmd_vel.shape}")
            print(f"body_vel_icp_mean {body_vel_icp_mean.shape}")
        return df

if __name__ =="__main__":
    path_to_dataset = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets/torch_ready_dataframe.pkl")

    df = pd.read_pickle(path_to_dataset)

    
    path_2_exp = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_results/offline")

    data_parser = SlipDatasetParser(df,path_2_exp,0.3,1.08,-16.6666,16.666666,20)

    data = data_parser.append_slip_elements_to_dataset(compute_by_whole_step=True,debug=False,smooth=True)

    path_2_exp = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets/slip_dataset_all.pkl")
    data.to_pickle(path_2_exp)

    #print_column_unique_column(data)

    terrain = "gravel"
    robot = "warthog"
    traction = "wheels"
    data_parser.create_dataframe_for_diamond_shape_graph(terrain,robot,traction,produce_video_now=False)
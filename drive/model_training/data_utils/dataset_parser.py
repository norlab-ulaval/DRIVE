
import numpy as np
import pandas as pd
import pathlib 

from drive.util.util_func import *
from drive.util.transform_algebra import *
from drive.util.model_func import diff_drive
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive
from drive.model_training.data_utils.extractors import * 
from scipy.spatial.transform import Rotation

def compute_all_tf(tf_poses,tf_euler):
    list_tf = []

    size = tf_poses.shape[0]

    for i in range(size):
        transform = np.eye(4)
        rotation = Rotation.from_euler("xyz",tf_euler[i,:])
        
        transform[:3,:3] = rotation.as_matrix()
        transform[:3,3] = tf_poses[i,:]

        list_tf.append(transform)
    
    return list_tf

class DatasetParser:
    def __init__(self, raw_dataset_path, export_dataset_path, training_horizon, rate,
                 calib_step_time, wheel_radius, baseline, imu_inverted):
        
        if pathlib.Path(raw_dataset_path).is_file():
            self.dataframe = pd.read_pickle(raw_dataset_path)
        else:
            raise ValueError(f"the raw_dataset_path does not exist or is wrongly named. It should be raw_dataframe.pkl and be located at the folowing path \n {raw_dataset_path}")
            
        self.dataframe = self.dataframe[3:]
        self.export_dataset_path = export_dataset_path
        self.training_horizon = training_horizon

        self.wheel_radius = wheel_radius
        self.baseline = baseline
        self.rate = rate
        self.calib_step_time = calib_step_time
        self.imu_inverted = imu_inverted

        self.ideal_diff_drive = Ideal_diff_drive(self.wheel_radius, self.baseline, self.rate)
        self.k = np.array([self.wheel_radius, self.baseline])
        self.n_window = int(calib_step_time//training_horizon)

    def extract_values_from_dataset(self):
        run = self.dataframe
        #print(self.dataframe.columns)
        self.timestamp = run['ros_time'].to_numpy().astype('double')
        
        for i in range(0, self.timestamp.shape[0]):
            self.timestamp[i] = self.timestamp[i] * 10 ** (-9)
        self.timestamp = (self.timestamp - self.timestamp[0])# * 10 ** (-9)  # time (s)

        self.icp_id = run['icp_index'].to_numpy().astype('int')
        self.joy = run['joy_switch'].to_numpy()
        self.joy = self.joy == 'True'
        # self.good_calib_step = run['good_calib_step'].to_numpy()
        # self.good_calib_step = self.good_calib_step == 'True'
        self.calib_step = run['calib_step'].to_numpy().astype('int')

        self.icp_x = run['icp_pos_x'].to_numpy().astype('float')  # icp x position (m)
        self.icp_y = run['icp_pos_y'].to_numpy().astype('float')  # icp y position (m)
        self.icp_z = run['icp_pos_z'].to_numpy().astype('float')  # icp y position (m)

        self.icp_quat_x = run['icp_quat_x'].to_numpy().astype('float')
        self.icp_quat_y = run['icp_quat_y'].to_numpy().astype('float')
        self.icp_quat_z = run['icp_quat_z'].to_numpy().astype('float')
        self.icp_quat_w = run['icp_quat_w'].to_numpy().astype('float')
        self.calib_state = run['calib_state'].to_numpy().astype('str')

        self.wheel_left_vel = run['meas_left_vel'].to_numpy().astype('float')
        self.wheel_right_vel = run['meas_right_vel'].to_numpy().astype('float')
        self.wheel_vels = np.vstack((self.wheel_left_vel, self.wheel_right_vel)).T

        self.n_points = self.timestamp.shape[0]

        self.cmd_vx = run['cmd_vel_x'].to_numpy().astype('float')
        self.cmd_omega = run['cmd_vel_omega'].to_numpy().astype('float')
        self.cmd_body_vels = np.concatenate((self.cmd_vx.reshape(self.n_points, 1),  self.cmd_omega.reshape(self.n_points, 1)), axis=1)


        self.cmd_wheel_vels = np.zeros((self.n_points, 2))
        for i in range(0, self.n_points):
            self.cmd_wheel_vels[i, :] = self.ideal_diff_drive.compute_wheel_vels(self.cmd_body_vels[i, :])
        
        self.icp_roll = np.zeros(self.n_points)
        self.icp_pitch = np.zeros(self.n_points)
        self.icp_yaw = np.zeros(self.n_points)

        for i in range(0, self.n_points):
            self.icp_roll[i], self.icp_pitch[i], self.icp_yaw[i] = quaternion_to_euler(self.icp_quat_w[i],
                                                                                       self.icp_quat_x[i],
                                                                                       self.icp_quat_y[i],
                                                                                       self.icp_quat_z[i])  # icp orientation (rad)

        self.icp_id_arr = run['icp_index'].to_numpy().astype('int')

        self.imu_pitch = run['imu_y'].to_numpy().astype('float')
        self.imu_roll = run['imu_x'].to_numpy().astype('float')
        self.imu_yaw = run['imu_z'].to_numpy().astype('float')
        self.imu_euler = np.column_stack((self.imu_roll, self.imu_pitch, self.imu_yaw))

        self.imu_acceleration_x = run['imu_acceleration_x'].to_numpy().astype('float')
        self.imu_acceleration_y = run['imu_acceleration_y'].to_numpy().astype('float')
        self.imu_acceleration_z = run['imu_acceleration_z'].to_numpy().astype('float')

        self.icp_quat = np.column_stack((self.icp_quat_x, self.icp_quat_y,
                                    self.icp_quat_z, self.icp_quat_w))

        self.icp_euler = np.zeros((self.icp_quat.shape[0], 3))

        for i in range(self.icp_quat.shape[0]):
            self.icp_euler[i, 0], self.icp_euler[i, 1], self.icp_euler[i, 2] = quaternion_to_euler(self.icp_quat[i, 3],
                                                                                                   self.icp_quat[i, 0],
                                                                                                   self.icp_quat[i, 1],
                                                                                                   self.icp_quat[i, 2])

        self.icp_states = np.column_stack((self.timestamp, self.icp_id, self.icp_x, self.icp_y, self.icp_z, self.icp_euler))
        self.icp_states_2d = np.column_stack((self.timestamp, self.icp_id, self.icp_x, self.icp_y, self.icp_euler[:, 2]))

        self.steady_or_transitory_window = []

    def compute_wheel_vels(self):
        self.wheel_left_vel = np.zeros(self.n_points)
        self.wheel_right_vel = np.zeros(self.n_points)

        for i in range(20, self.n_points):
            dt = self.timestamp[i] - self.timestamp[i - 1]
            if dt < 0.01:
                self.wheel_left_vel[i] = self.wheel_left_vel[i - 1]
                self.wheel_right_vel[i] = self.wheel_right_vel[i - 1]
            else:
                self.wheel_left_vel[i] = (self.wheel_pos_left[i] - self.wheel_pos_left[i - 1]) / dt
                self.wheel_right_vel[i] = (self.wheel_pos_right[i] - self.wheel_pos_right[i - 1]) / dt

        n_points_convolution = 20
        self.wheel_left_vel = np.convolve(self.wheel_left_vel, np.ones((n_points_convolution,)) / n_points_convolution,
                                     mode='same')
        self.wheel_right_vel = np.convolve(self.wheel_right_vel, np.ones((n_points_convolution,)) / n_points_convolution,
                                      mode='same')
        self.wheel_vels = np.vstack((self.wheel_left_vel, self.wheel_right_vel)).T

    def compute_diff_drive_body_vels(self):
        self.diff_drive_vels = np.zeros((self.n_points, 3))

        for i in range(0, self.n_points):
            self.diff_drive_vels[i, :] = diff_drive(self.wheel_vels[i, :], self.k)

    def compute_icp_based_velocity(self):
        self.icp_vx = np.zeros(self.n_points)
        self.imu_omega = np.zeros(self.n_points)

        propa_cos = np.cos(self.icp_states[0, 4])
        propa_sin = np.sin(self.icp_states[0, 4])
        propa_mat = np.array([[propa_cos, -propa_sin, 0.0],
                              [propa_sin, propa_cos, 0.0], [0.0, 0.0, 1.0]])

        self.icp_vels = np.zeros((self.n_points, 3))
        icp_disp = np.zeros((1, 3))

        dt = 0

        for i in range(1, self.n_points - 1):
            dt += self.timestamp[i + 1] - self.timestamp[i]
            if self.icp_id[i + 1] != self.icp_id[i]:
                icp_disp = self.icp_states_2d[i + 1, 2:] - self.icp_states_2d[i, 2:]
                icp_disp[2] = wrap2pi(icp_disp[2])

                #         print(icp_states[i,4])
                propa_cos = np.cos(self.icp_states_2d[i, 4])
                propa_sin = np.sin(self.icp_states_2d[i, 4])
                propa_mat[0, 0] = propa_cos
                propa_mat[0, 1] = -propa_sin
                propa_mat[1, 0] = propa_sin
                propa_mat[1, 1] = propa_cos
                #         print(i)
                #         print(icp_disp)
                icp_disp = propa_mat.T @ icp_disp
                #         print(icp_disp)

                self.icp_vels[i, :] = icp_disp / dt

                dt = 0

            else:
                self.icp_vels[i, :] = self.icp_vels[i - 1, :]

        n_points_convolution = 10
        self.icp_vels[:, 0] = np.convolve(self.icp_vels[:, 0], np.ones((n_points_convolution,)) / n_points_convolution,
                                     mode='same')

    def concatenate_into_full_dataframe(self):
        cols = ['timestamp', 'imu_roll_vel', 'imu_pitch_vel', 'imu_yaw_vel', 'cmd_left', 'cmd_right',
                'icp_x', 'icp_y', 'icp_z', 'icp_roll', 'icp_pitch', 'icp_yaw', 'icp_vx', 'icp_vy', 'icp_omega',
                'encoder_wheel_left_vel', 'encoder_wheel_right_vel', 'diff_drive_vels_x', 'diff_drive_vels_y', 'diff_drive_vels_omega',
                'calib_step', 'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z']
        self.parsed_dataset = np.concatenate((self.timestamp.reshape(self.n_points, 1), self.imu_euler,
                                              self.cmd_wheel_vels,
                                              self.icp_states[:, 2:], self.icp_vels, # Va chercher x,y,z RPY
                                              self.wheel_left_vel.reshape(self.n_points, 1), self.wheel_right_vel.reshape(self.n_points, 1),
                                              self.diff_drive_vels, self.calib_step.reshape(self.n_points, 1),
                                              self.imu_acceleration_x.reshape(self.n_points, 1), self.imu_acceleration_y.reshape(self.n_points, 1),
                                              self.imu_acceleration_z.reshape(self.n_points, 1)), axis=1)

        self.parsed_dataset_df = pd.DataFrame(self.parsed_dataset, columns=cols)
        #self.parsed_dataset_df.to_pickle("drive/model_training/data_utils/debug/parsed_dataframe_df.pkl")

    def find_training_horizons(self):
        # self.parsed_dataset_steady_state = self.parsed_dataset[self.steady_state_mask]
        self.parsed_dataset_steady_state = self.parsed_dataset
        n_points_steady_state = self.parsed_dataset_steady_state.shape[0]
        self.horizon_starts = []
        self.horizon_ends = []

        for i in range(1, self.n_points):
            if self.calib_step[i] != self.calib_step[i-1]:
                elapsed_time_step = 0
                j = i-1
                window_2_switch = True
                window_3_switch = True
                # if self.parsed_dataset_steady_state[j, 20] == self.parsed_dataset_steady_state[-1, 20]:
                #     self.horizon_starts.pop()
                #     break
                #TODO: find all training horizons starting from calibration step ends
                while elapsed_time_step <= self.calib_step_time:
                    elapsed_time_step += self.timestamp[j+1] - self.timestamp[j]
                    if elapsed_time_step >= 2.0 and window_3_switch:
                        window_3_start = j
                        window_3_switch = False
                    if elapsed_time_step >= 4.0 and window_2_switch:
                        window_2_start = j
                        window_2_switch = False
                    j -= 1
                # append window 1 for valid step
                self.steady_or_transitory_window.append('transitory')
                self.horizon_starts.append(j)
                self.horizon_ends.append(window_2_start - 1)
                # append window 2 for valid step
                self.steady_or_transitory_window.append('steady')
                self.horizon_starts.append(window_2_start)
                self.horizon_ends.append(window_3_start - 1)
                # append window 3 for valid step
                self.steady_or_transitory_window.append('steady')
                self.horizon_starts.append(window_3_start)
                self.horizon_ends.append(i)

        self.n_horizons = len(self.horizon_starts)

    def define_steady_state_horizons(self):
        self.steady_state_horizons_list = []
        for i in range(self.n_horizons):
            if self.calib_state[self.horizon_starts[i]] == 'calib' and \
                    np.all(self.cmd_vx[self.horizon_starts[i]:self.horizon_ends[i]] == self.cmd_vx[self.horizon_starts[i]]):
                steady_state_bool = True
            else:
                steady_state_bool = False
            self.steady_state_horizons_list.append(steady_state_bool)

    def compute_step_frame_icp(self):
        df = self.torch_dataset_df

        tf_pose  = df[["init_tf_pose_x","init_tf_pose_y","init_tf_pose_z"]].to_numpy()
        tf_euler = df[["init_tf_pose_roll","init_tf_pose_pitch","init_tf_pose_yaw"]].to_numpy()
        list_tf = compute_all_tf(tf_pose,tf_euler)

        icp_x = column_type_extractor(df,"icp_x")
        icp_y = column_type_extractor(df,"icp_y")
        icp_z = column_type_extractor(df,"icp_z")
        
        icp_roll_wrapped = column_type_extractor(df,"icp_roll")
        icp_pitch_wrapped = column_type_extractor(df,"icp_pitch")
        # Create discontinuities if not unwrapped 
        icp_yaw_wrapped = column_type_extractor(df,"icp_yaw")

        # unwrapped the devil 
        icp_roll = np.zeros_like(icp_roll_wrapped)
        icp_pitch = np.zeros_like(icp_pitch_wrapped)
        icp_yaw = np.zeros_like(icp_yaw_wrapped)

        for i in range(icp_yaw_wrapped.shape[0]): 
            icp_roll[i,:] = np.unwrap(icp_roll_wrapped[i,:],discont=np.pi)
            icp_pitch[i,:] = np.unwrap(icp_pitch_wrapped[i,:],discont=np.pi)
            icp_yaw[i,:] = np.unwrap(icp_yaw_wrapped[i,:],discont=np.pi)

        size = tf_pose.shape[0]

        step_exp_x = np.zeros_like(icp_x)
        step_exp_y = np.zeros_like(icp_x)
        step_exp_z = np.zeros_like(icp_x)
        step_exp_roll = np.zeros_like(icp_x)
        step_exp_pitch = np.zeros_like(icp_x)
        step_exp_yaw = np.zeros_like(icp_x)

        tf_to_reapplied = np.eye(4)

        for i in  range(size):

            window_poses = np.vstack((icp_x[i,:],icp_y[i,:],icp_z[i,:]))
            window_angles = np.vstack((icp_roll[i,:],icp_pitch[i,:],icp_yaw[i,:]))

            tf_map_first_point = list_tf[i]

            step_window_poses = np.zeros_like(window_poses)

            step_window_euler_angles = np.zeros_like(window_poses)
            
            if i % self.n_window ==0: 
                tf_to_reapplied = np.linalg.inv(tf_map_first_point)

            for j in range(window_poses.shape[1]):

                pose = np.eye(4)
                rotation = Rotation.from_euler("xyz",window_angles[:,j])
                pose[:3,:3] = rotation.as_matrix()
                pose[:3,3] = window_poses[:,j]

                pose_map = tf_to_reapplied @ tf_map_first_point @ pose
                
                step_window_poses[:,j] = pose_map[:3,3]
                step_window_euler_angles[:,j] = Rotation.from_matrix(pose_map[0:3,0:3]).as_euler("xyz")

            
            step_exp_x[i,:] = step_window_poses[0,:]
            step_exp_y[i,:] = step_window_poses[1,:]
            step_exp_z[i,:] = step_window_poses[2,:]
            step_exp_roll[i,:] = step_window_euler_angles[0,:]
            step_exp_pitch[i,:] = step_window_euler_angles[1,:]
            step_exp_yaw[i,:] = step_window_euler_angles[2,:]



            
        return step_exp_x, step_exp_y, step_exp_z, step_exp_roll,step_exp_pitch, step_exp_yaw

    def create_last_window_mask(self):
        """The goal is to compute mask indicating if the precedent window can be used for the operation point. 

        1 == True  (precedent_window ==precedent_time)
        0 == False  (precedent_window !=precedent_time)
        
        Args:
            mask_with_0 (_type_): Mask indicating wether the precedent window can be used to extract the point. 
        """
        df = self.torch_dataset_df
        size = df["start_time"].shape[0]
        n_step = size//3

        start_column = df["start_time"].to_numpy()#.reshape((int(size//n_window),int(n_window)))
        end_column = df["end_time"].to_numpy()#.reshape((int(size//n_window),int(n_window)))

        differential = start_column[1:] - end_column[:-1]

        mask = list(differential < self.training_horizon*1.10)
        mask_with_0 = np.array([False] + mask)
        mask_reshape = mask_with_0.reshape((int(size//self.n_window),int(self.n_window)))

        return mask_with_0
    

    def build_torch_ready_dataset(self):
        init_state_tf = np.eye(4)
        homogeonous_state_position = np.zeros(4)
        homogeonous_state_position[3] = 1

        self.rate = 0.05
        timesteps_per_horizon = int(self.training_horizon / self.rate)

        torch_input_array = np.zeros((len(self.horizon_starts),
                                      12 + timesteps_per_horizon * 4 + timesteps_per_horizon * 6 + timesteps_per_horizon + 3*timesteps_per_horizon))  # [icp_x, icp_y, icp_yaw, vx0, vomega0, vx1, vomega1, vx2, vomega2, vx3, vomega3]
        torch_output_array = np.zeros((len(self.horizon_starts), 6))  # [icp_x, icp_y, icp_yaw]

        self.initial_tf_pose = np.zeros((len(self.horizon_starts),3))
        self.initial_tf_roll_pitch_yaw = np.zeros((len(self.horizon_starts),3))

        for i in range(0, len(self.horizon_starts)):
            # if i != 511:
            #     continue
            horizon_start = self.horizon_starts[i]
            horizon_end = self.horizon_ends[i]
            # torch_input_array[i, :6] = self.parsed_dataset[horizon_start, 6:12]  # init_state


            euler_tf = self.parsed_dataset[horizon_start, 9:12] #extract_angles roll pitch yaw
            self.initial_tf_roll_pitch_yaw[i,:] = euler_tf

            position_tf = self.parsed_dataset[horizon_start, 6:9]
            self.initial_tf_pose[i,:] = position_tf

            euler_pose_to_transform(euler_tf, position_tf, init_state_tf)
            init_state_tf_inv = np.linalg.inv(init_state_tf)

            torch_input_array[i, :6] = np.zeros(6)  # init_state set at 0
            torch_input_array[i, 6] = self.parsed_dataset[horizon_start, 20]  # calib_step
            # torch_input_array[i, 7] = self.parsed_dataset[horizon_start, 4]  # cmd_vx
            # torch_input_array[i, 8] = self.parsed_dataset[horizon_start, 5]  # cmd_omega
            for j in range(0, timesteps_per_horizon):  # adding wheel commands
                torch_input_array[i, 7 + j * 2] = self.parsed_dataset[horizon_start + j, 4]
                torch_input_array[i, 7 + j * 2 + 1] = self.parsed_dataset[horizon_start + j, 5]
            for j in range(0, timesteps_per_horizon):  # adding wheel encoder measurements
                torch_input_array[i, 87 + j * 2] = self.parsed_dataset[horizon_start + j, 15]
                torch_input_array[i, 87 + j * 2 + 1] = self.parsed_dataset[horizon_start + j, 16]
            # Append the 40 it of each windows of 2 seconfds. 
            for j in range(0, timesteps_per_horizon):  # adding intermediary icp measurements
                homogeonous_state_position[:3] = self.parsed_dataset[horizon_start + j, 6:9] # x, y, z, 1

                init_state_transformed_position = init_state_tf_inv @ homogeonous_state_position
                torch_input_array[i, 167 + j * 6:167 + j * 6 + 3] = init_state_transformed_position[:3]
                
                # Difference d<angle avec le debut de la window. 
                current_orientation = Rotation.from_euler("xyz",self.parsed_dataset[horizon_start + j,
                                                                        9:12])
                init_state_transformed_orientation =  Rotation.from_matrix(init_state_tf_inv[:3,:3] @ current_orientation.as_matrix()).as_euler("xyz")
                torch_input_array[i, 167 + 3 + j * 6:167 + 6 + j * 6] = init_state_transformed_orientation
                ### 


                torch_input_array[i, 3] = wrap2pi(torch_input_array[i, 3]) # State initial = 0 en roll pitch yaw ce qui est faux
                torch_input_array[i, 4] = wrap2pi(torch_input_array[i, 4])
                torch_input_array[i, 5] = wrap2pi(torch_input_array[i, 5])

            for j in range(0, timesteps_per_horizon): # adding wheel commands

                if self.imu_inverted:
                    torch_input_array[i, 407 + j] = self.parsed_dataset[horizon_start + j, 3] # imu yaw rate
                else:
                    torch_input_array[i, 407 + j] = -self.parsed_dataset[horizon_start + j, 3]  # imu yaw rate

            torch_input_array[i, 447] = np.mean(self.parsed_dataset[horizon_start:horizon_end, 12])  # icp_vx
            torch_input_array[i, 448] = np.mean(self.parsed_dataset[horizon_start:horizon_end, 13])  # icp_vy
            torch_input_array[i, 449] = np.mean(self.parsed_dataset[horizon_start:horizon_end, 14])  # icp_omega
            # torch_input_array[i, 170] = self.parsed_dataset[horizon_start, 21]  # steady_state_mask

            if self.steady_or_transitory_window[i] == 'steady':
                torch_input_array[i, 450] = True  # steady_state_mask
            else:
                torch_input_array[i, 450] = False  # steady_state_mask
            if self.steady_or_transitory_window[i] == 'transitory':
                torch_input_array[i, 451] = True  # transitory_mask
            else:
                torch_input_array[i, 451] = False  # transitory_mask

            for j in range(0, timesteps_per_horizon):  # adding imu accelerations
                torch_input_array[i, 452 + j * 3] = self.parsed_dataset[horizon_start + j, -3]
                torch_input_array[i, 452 + j * 3 + 1] = self.parsed_dataset[horizon_start + j, -2]
                torch_input_array[i, 452 + j * 3 + 2] = self.parsed_dataset[horizon_start + j, -1]


            # torch_output_array[i, :] = self.parsed_dataset[horizon_end, 6:12] # absolute final pose
            homogeonous_state_position[:3] = self.parsed_dataset[horizon_end, 6:9]
            output_state_transformed_pose = init_state_tf_inv @ homogeonous_state_position
            torch_output_array[i, :3] = output_state_transformed_pose[:3]
            torch_output_array[i, 3:] = self.parsed_dataset[horizon_end, 9:12] - self.parsed_dataset[horizon_start, 9:12] # relative final pose
            torch_output_array[i, 3] = wrap2pi(torch_output_array[i, 3])
            torch_output_array[i, 4] = wrap2pi(torch_output_array[i, 4])
            torch_output_array[i, 5] = wrap2pi(torch_output_array[i, 5])

        torch_array = np.concatenate((torch_input_array, torch_output_array), axis=1)

        cols = ['init_icp_x', 'init_icp_y', 'init_icp_z', 'init_icp_roll', 'init_icp_pitch', 'init_icp_yaw']
        cols.append('calib_step')
        # cols.append('cmd_vx')
        # cols.append('cmd_omega')
        for i in range(0, timesteps_per_horizon):
            str_cmd_vx_i = 'cmd_left_' + str(i)
            str_cmd_omega_i = 'cmd_right_' + str(i)
            cols.append(str_cmd_vx_i)
            cols.append(str_cmd_omega_i)
        for i in range(0, timesteps_per_horizon):
            str_encoder_left_i = 'left_wheel_vel_' + str(i)
            str_encoder_right_i = 'right_wheel_vel_' + str(i)
            cols.append(str_encoder_left_i)
            cols.append(str_encoder_right_i)
        for i in range(0, timesteps_per_horizon):
            str_icp_x_i = 'icp_x_' + str(i)
            str_icp_y_i = 'icp_y_' + str(i)
            str_icp_z_i = 'icp_z_' + str(i)
            str_icp_roll_i = 'icp_roll_' + str(i)
            str_icp_pitch_i = 'icp_pitch_' + str(i)
            str_icp_yaw_i = 'icp_yaw_' + str(i)
            cols.append(str_icp_x_i)
            cols.append(str_icp_y_i)
            cols.append(str_icp_z_i)
            cols.append(str_icp_roll_i)
            cols.append(str_icp_pitch_i)
            cols.append(str_icp_yaw_i)
        for i in range(0, timesteps_per_horizon):
            str_icp_x_i = 'imu_yaw_' + str(i)
            cols.append(str_icp_x_i)
        # cols.append('encoder_vx')
        # cols.append('encoder_omega')
        cols.append('icp_vx')
        cols.append('icp_vy')
        cols.append('icp_omega')
        cols.append('steady_state_mask')
        cols.append('transitory_state_mask')
        for i in range(0, timesteps_per_horizon):
            str_imu_accel_x_i = 'imu_acceleration_x_' + str(i)
            cols.append(str_imu_accel_x_i)
            str_imu_accel_y_i = 'imu_acceleration_y_' + str(i)
            cols.append(str_imu_accel_y_i)
            str_imu_accel_z_i = 'imu_acceleration_z_' + str(i)
            cols.append(str_imu_accel_z_i)
        cols.append('gt_icp_x')
        cols.append('gt_icp_y')
        cols.append('gt_icp_z')
        cols.append('gt_icp_roll')
        cols.append('gt_icp_pitch')
        cols.append('gt_icp_yaw')

        self.torch_dataset_df = pd.DataFrame(torch_array, columns=cols)
        
        self.torch_dataset_df["start_time"] = self.timestamp[self.horizon_starts]
        self.torch_dataset_df["end_time"] = self.timestamp[self.horizon_ends]


        self.torch_dataset_df[["init_tf_pose_x","init_tf_pose_y","init_tf_pose_z"]] = self.initial_tf_pose
        self.torch_dataset_df[["init_tf_pose_roll","init_tf_pose_pitch","init_tf_pose_yaw"]] = self.initial_tf_roll_pitch_yaw
        
        #print(self.cmd_vx.shape)
        #self.torch_dataset_df["cmd_body_vel_x"] = reshape_into_2sec_windows(self.cmd_vx.reshape((self.cmd_vx,1)))
        #self.torch_dataset_df["cmd_body_vel_y"] = reshape_into_2sec_windows(np.zeros_like(self.cmd_vx))
        #self.torch_dataset_df["cmd_body_vel_yaw"] = reshape_into_2sec_windows(self.cmd_omega.reshape((self.cmd_omega,1)))

        steps_frame_icps = self.compute_step_frame_icp()
        stack_step_frame = np.hstack(steps_frame_icps)
        column_type = ["x","y","z","roll","pitch","yaw"]
        all_step_columns = []
        for column in column_type: 
            columns = ["step_frame_icp_"+column+f"_{i}" for i in range(steps_frame_icps[0].shape[1])]
            all_step_columns += columns

        df = pd.DataFrame(data=stack_step_frame,columns=all_step_columns)
        df["precedent_window_operation_point_mask"] = self.create_last_window_mask()
        self.torch_dataset_df = pd.concat((self.torch_dataset_df,df),axis=1)
        
        
    def process_data(self):
        self.extract_values_from_dataset()
        self.compute_diff_drive_body_vels()
        self.compute_icp_based_velocity()
        self.concatenate_into_full_dataframe()
        self.find_training_horizons()
        self.define_steady_state_horizons()
        self.build_torch_ready_dataset()

        self.torch_dataset_df.to_pickle(self.export_dataset_path)
        return self.torch_dataset_df
    

if __name__=="__main__":

    raw_dataset_path = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets/warthog_wheels_gravel_1_data-raw.pkl"
    export_dataset_path = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets/warthog_gravel_dataframe.pkl"
    training_horizon = 2
    rate = 20
    calib_step_time = 6
    wheel_radius = 0.3
    baseline = 1.08
    imu_inverted = False

    
    df = DatasetParser( raw_dataset_path, export_dataset_path, training_horizon, rate,
                calib_step_time, wheel_radius, baseline, imu_inverted)
    
    df.process_data()
    print_column_unique_column(df.torch_dataset_df)

    data_left  = column_type_extractor(df.torch_dataset_df,"cmd_left")
    data_right = column_type_extractor(df.torch_dataset_df,"right_wheel_vel")
    
    # cmd_body_vels
    print(df.cmd_wheel_vels.shape)

    
    fig,axs = plt.subplots(1,1)
    #rint(df.cmd_body_vels.shape)
    list_unique = []

    
    
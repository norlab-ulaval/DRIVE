import pandas as pd 
import numpy as np 
from scipy.spatial.transform import Rotation 

df = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/raw_dataframe.pkl")

df.calib_step = df.calib_step.astype(float)

df = df.loc[1:]
df.reset_index(inplace=True)

print(df.columns)

print(df.shape)


def compute_position_and_orientation_in_position(df):
    """Compute position in body frame

    Args:
        df (_type_): _description_
    """


    quat_body_array = np.zeros((df.shape[0],4))
    position_array = np.zeros((df.shape[0],3))

    

    for i in range(df.shape[0]):

        line_ = df.loc[i]

        
        rotation_matrix_i =  Rotation.from_quat([line_["icp_quat_x"],line_["icp_quat_y"],
                                                line_["icp_quat_z"],line_["icp_quat_w"]],
                                                scalar_first=False)

        transformation_matrix_G_B_now = np.eye(4)

        transformation_matrix_G_B_now[:3,:3] = rotation_matrix_i.as_matrix() 
        transformation_matrix_G_B_now[:3,3] = np.array([line_["icp_pos_x"],line_["icp_pos_y"],
                                                line_["icp_pos_z"]])
        #print(transformation_matrix_G_B_now)

        if i ==0: 
            transformation_matrix_G_B_minus_one = transformation_matrix_G_B_now
            transformation_body = transformation_matrix_G_B_now
        else:
            transformation_body = np.linalg.inv(transformation_matrix_G_B_minus_one) @ transformation_matrix_G_B_now

        
        position_array[i,:] = transformation_body[:3,3]
        quat_body_array[i,:] = Rotation.from_matrix(transformation_body[:3,:3]).as_quat(scalar_first=False)






#print(df.shape)
#compute_position_and_orientation_in_position(df)
#
#df = df.loc[(df.calib_step > 2) &  (df.calib_step < 10)]
#df.to_csv("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/francois_dataframe_step_2_to_10.csv")

######
#Index(['ros_time', 'joy_switch', 'icp_index', 'calib_state', 'calib_step',
#       'meas_left_vel', 'meas_right_vel', 'cmd_vel_x', 'cmd_vel_omega',
#       'icp_pos_x', 'icp_pos_y', 'icp_pos_z', 'icp_quat_x', 'icp_quat_y',
#       'icp_quat_z', 'icp_quat_w', 'imu_x', 'imu_y', 'imu_z',
#       'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z',
#       'left_wheel_voltage', 'right_wheel_voltage', 'left_wheel_current',
#       'right_wheel_current'],
#      dtype='object')
#
import pandas as pd
import matplotlib.pyplot as plt

def verify_current_measurements():
       df = pd.read_pickle("calib_data/test_close/raw_dataframe.pkl")

       list_ = ['ros_time', 'joy_switch', 'icp_index', 'calib_state', 'calib_step',
              'meas_left_vel', 'meas_right_vel', 'cmd_vel_x', 'cmd_vel_omega',
              'icp_pos_x', 'icp_pos_y', 'icp_pos_z', 'icp_quat_x', 'icp_quat_y',
              'icp_quat_z', 'icp_quat_w', 'imu_x', 'imu_y', 'imu_z',
              'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z',
              'left_wheel_voltage', 'right_wheel_voltage', 'left_wheel_current',
              'right_wheel_current']
       print(df.head(5))


       df = df.astype({'ros_time': 'float',"left_wheel_current":'float',"right_wheel_voltage":'float' })
       plt.scatter(df["ros_time"].index,df["left_wheel_current"].values)
       plt.show()

       df.hist("right_wheel_voltage")
       plt.show()

def look_at_the_contain_of_input_space_calib():

       path = "calib_data/test_warthog/input_space_data_mw.pkl"

       df = pd.read_pickle(path).transpose()

       print(df.head)
look_at_the_contain_of_input_space_calib()
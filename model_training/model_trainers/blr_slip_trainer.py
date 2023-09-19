import numpy as np
import pandas as pd

from model_training.models.learning.blr_slip import FullBodySlipBayesianLinearRegression, SlipBayesianLinearRegression

class SlipBLRTrainer():
    def __init__(self, dataframe, wheel_radius, baseline, rate):
        self.dataframe = dataframe[:int(len(dataframe))]
        # self.dataframe = dataframe # TODO : switch for realtime doughnut

        a_param_init = 0
        b_param_init = 0
        param_variance_init = 999999999999999999999
        variance_init = 1000000000
        kappa_param = 1
        dt = 1/rate

        self.full_body_slip_blr = FullBodySlipBayesianLinearRegression(1, 1, 3,
                                                                       a_param_init,
                                                                       b_param_init,
                                                                       param_variance_init,
                                                                       variance_init,
                                                                       baseline,
                                                                       wheel_radius,
                                                                       dt,
                                                                       kappa_param)


    def extract_relevant_values(self):
        # isolate steady-state data

        # extract cmd_body_vel arrays (input arrays)

        idd_body_vel_x_str_list = []
        idd_body_vel_y_str_list = []
        idd_body_vel_yaw_str_list = []
        for i in range(0, 40):
            str_idd_vel_x_i = 'idd_vel_x_' + str(i)
            str_idd_vel_y_i = 'idd_vel_y_' + str(i)
            str_idd_vel_yaw_i = 'idd_vel_yaw_' + str(i)
            idd_body_vel_x_str_list.append(str_idd_vel_x_i)
            idd_body_vel_y_str_list.append(str_idd_vel_y_i)
            idd_body_vel_yaw_str_list.append(str_idd_vel_yaw_i)
        self.idd_body_vel_x_array = self.dataframe[idd_body_vel_x_str_list].to_numpy()
        self.idd_body_vel_y_array = self.dataframe[idd_body_vel_y_str_list].to_numpy()
        self.idd_body_vel_yaw_array = self.dataframe[idd_body_vel_yaw_str_list].to_numpy()

        # extract body_vel_distruptions arrays (output arrays)

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

        self.body_vel_disturption_x_array = self.dataframe[str_body_vel_disturption_x_list].to_numpy()
        self.body_vel_disturption_y_array = self.dataframe[str_body_vel_disturption_y_list].to_numpy()
        self.body_vel_disturption_yaw_array = self.dataframe[str_body_vel_disturption_yaw_list].to_numpy()

        self.steady_state_mask = self.dataframe['steady_state_mask'].to_numpy() == True

        self.steady_state_idd_body_vel_x = self.idd_body_vel_x_array[self.steady_state_mask]
        self.steady_state_idd_body_vel_y = self.idd_body_vel_y_array[self.steady_state_mask]
        self.steady_state_idd_body_vel_yaw = self.idd_body_vel_yaw_array[self.steady_state_mask]

        self.steady_state_body_vel_disturption_x = self.body_vel_disturption_x_array[self.steady_state_mask]
        self.steady_state_body_vel_disturption_y = self.body_vel_disturption_y_array[self.steady_state_mask]
        self.steady_state_body_vel_disturption_yaw = self.body_vel_disturption_yaw_array[self.steady_state_mask]

        self.x_train = np.column_stack((self.steady_state_idd_body_vel_x.flatten(), self.steady_state_idd_body_vel_y.flatten(),
                                   self.steady_state_idd_body_vel_yaw.flatten()))
        self.y_train = np.column_stack((self.steady_state_body_vel_disturption_x.flatten(),
                                   self.steady_state_body_vel_disturption_y.flatten(),
                                   self.steady_state_body_vel_disturption_yaw.flatten()))

    def train_blr_model(self):
        self.extract_relevant_values()
        self.full_body_slip_blr.train_params(self.x_train, self.y_train)

        return self.full_body_slip_blr


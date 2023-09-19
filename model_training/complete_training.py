import pandas as pd
import numpy as np

import argparse
import os
from time import gmtime, strftime

from data_utils.dataset_parser import DatasetParser
from data_utils.slip_dataset_parser import SlipDatasetParser
# from data_utils.acceleration_dataset_parser import AccelerationDatasetParser

from model_trainers.powertrain_trainer import Powertrain_Trainer
from models.powertrain.bounded_powertrain import Bounded_powertrain

from model_training.model_trainers.blr_slip_trainer import SlipBLRTrainer

parser = argparse.ArgumentParser()
parser.add_argument('-e', '--experiment_name', type=str)
parser.add_argument('-r', '--rate', type=float)
parser.add_argument('-t', '--training_horizon', type=float)
parser.add_argument('-s', '--calib_step_time', type=float)
parser.add_argument('-i', '--imu_inverted', type=bool)
args = parser.parse_args()
if args.experiment_name == None:
    args.experiment_name = strftime("%Y-%m-%d %H:%M:%S", gmtime())
experiment_data_path = os.getcwd() + '/' + '../calib_data/' + args.experiment_name + '/'
rate = args.rate
training_horizon = args.training_horizon
calib_step_time = args.calib_step_time
imu_inverted = args.imu_inverted

# input_space_df = pd.read_pickle(experiment_data_path + 'input_space_data.pkl')
# wheel_radius = input_space_df['calibrated_radius [m]'][0]
wheel_radius = 0.3
# baseline = input_space_df['calibrated baseline [m]'][0] # TODO: fix for next doughnut data
baseline = 1.1652 # TODO: fix for next doughnut data
# max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
# min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]
max_wheel_vel = 13
min_wheel_vel = -13

raw_data_path = experiment_data_path + 'data_raw.pkl'
torch_dataset_path = experiment_data_path + 'torch_ready_dataframe.pkl'
dataset_parser = DatasetParser(raw_data_path, torch_dataset_path, training_horizon,
                               rate, calib_step_time, wheel_radius, baseline, imu_inverted)
parsed_dataframe = dataset_parser.process_data()

timesteps_per_horizon = training_horizon * rate
dt = 1/rate
init_params = [0.4, 0.05]
bounds = [(0.0, 5.0), (0.0, 1.0)]
method = 'Nelder-Mead'
bounded_powertrain = Bounded_powertrain(min_wheel_vel, max_wheel_vel, time_constant=0.5, time_delay=0.05, dt=0.05)
powertrain_trainer = Powertrain_Trainer(powertrain_model=bounded_powertrain, init_params=init_params, dataframe=parsed_dataframe,
                              timesteps_per_horizon=timesteps_per_horizon, dt=0.05, saved_params_path=experiment_data_path)
left_training_result, right_training_result = powertrain_trainer.train_model(init_params=init_params, method=method, bounds=bounds)

left_min_vel, left_max_vel, right_min_vel, right_max_vel = powertrain_trainer.find_max_wheel_vels()

mean_min_vel = (left_min_vel + right_min_vel) / 2
mean_max_vel = (left_max_vel + right_max_vel) / 2

left_side_saved_params = experiment_data_path + 'powertrain/powertrain_training_left.npy'
os.makedirs(os.path.dirname(left_side_saved_params), exist_ok=True)
np.save(left_side_saved_params, left_training_result)

right_side_saved_params = experiment_data_path + 'powertrain/powertrain_training_right.npy'
os.makedirs(os.path.dirname(right_side_saved_params), exist_ok=True)
np.save(right_side_saved_params, right_training_result)

print(left_training_result)
print(right_training_result)

slip_dataset_parser = SlipDatasetParser(parsed_dataframe, experiment_data_path, wheel_radius, baseline, mean_min_vel, mean_max_vel, rate)
slip_dataset = slip_dataset_parser.append_slip_elements_to_dataset()

blr_slip_trainer = SlipBLRTrainer(slip_dataset, wheel_radius, baseline, rate)
trained_blr_slip_model = blr_slip_trainer.train_blr_model()

print('weights_x : ', trained_blr_slip_model.body_x_slip_blr.weights)
print('weights_y : ', trained_blr_slip_model.body_y_slip_blr.weights)
print('weights_yaw : ', trained_blr_slip_model.body_yaw_slip_blr.weights)

slip_blr_params_path = experiment_data_path + 'slip_blr/'
os.makedirs(os.path.dirname(slip_blr_params_path))
trained_blr_slip_model.save_params(slip_blr_params_path)
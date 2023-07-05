import pandas as pd
import numpy as np

import argparse
import os
from time import gmtime, strftime

from data_utils.dataset_parser import DatasetParser
from data_utils.slip_dataset_parser import SlipDatasetParser
from data_utils.acceleration_dataset_parser import AccelerationDatasetParser

from model_trainers.powertrain_trainer import Powertrain_Trainer
from models.powertrain.bounded_powertrain import Bounded_powertrain

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

input_space_df = pd.read_pickle(experiment_data_path + 'input_space_data.pkl')
wheel_radius = input_space_df['calibrated_radius [m]'][0]
baseline = input_space_df['calibrated baseline [m]'][0] # TODO: fix for next doughnut data
max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]

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
                              timesteps_per_horizon=timesteps_per_horizon, dt=0.05)
powertrain_trainer.train_model(init_params=init_params, method=method, bounds=bounds)


### TODO : parse dataset

### TODO : train powertrain

### TODO : parse slip, acceleration dataset

### TODO : train slip BLR model

### TODO : export training results

## TODO : IF user wants, export training datasets as well
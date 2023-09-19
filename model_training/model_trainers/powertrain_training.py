import math
# from pypointmatcher import pointmatcher, pointmatchersupport
import glob
import numpy as np
import copy
import pandas as pd
# import wmrde
import torch
from torch.utils.data import random_split
from torch.utils.data import DataLoader
# from torchmin import minimize

from models.powertrain.bounded_powertrain import Bounded_powertrain

from eval.torch_dataset import TorchWMRDataset
from eval.powertrain_trainer import Powertrain_Trainer

from scipy.optimize import minimize

# CUDA for PyTorch
use_cuda = torch.cuda.is_available()
device = torch.device("cuda:0" if use_cuda else "cpu")
torch.backends.cudnn.benchmark = True

# Parameters
params = {'batch_size': 64,
          'shuffle': True,
          'num_workers': 6}
max_epochs = 100

train_dataset_path = '/home/dominic/repos/norlab_WMRD/data/ral2023_dataset/husky/boreal_snow/torch_dataset_all.pkl'
# train_dataset_path = '/home/dominic/repos/norlab_WMRD/data/husky/vel_mask_array_all.npy'
training_horizon = 2 # seconds
timestep = 0.05 # seconds
timesteps_per_horizon = int(training_horizon / timestep)

wmr_train_dataset = TorchWMRDataset(train_dataset_path, body_or_wheel_vel='wheel', training_horizon=training_horizon)
wmr_train_dl = DataLoader(wmr_train_dataset)

# robot = 'marmotte'
robot = 'husky'
# robot = 'warthog-tracks'
if robot == 'marmotte':
    input_space_dataframe = pd.read_pickle('/home/dominic/repos/norlab_WMRD/data/marmotte/input_space/input_space_data.pkl')
    r = input_space_dataframe['calibrated_radius [m]'].to_numpy()[0]
    baseline = input_space_dataframe['calibrated baseline [m]'].to_numpy()[0]
    # min_wheel_vel = input_space_dataframe['maximum_wheel_vel_negative [rad/s]'].to_numpy()[0]
    min_wheel_vel = -10
    # max_wheel_vel = input_space_dataframe['maximum_wheel_vel_positive [rad/s]'].to_numpy()[0]
    max_wheel_vel = 10

if robot == 'husky':
    input_space_dataframe = pd.read_pickle('/home/dominic/repos/norlab_WMRD/data/husky/input_space_data/input_space_data_ga.pkl')
    # r = input_space_dataframe['calibrated_radius [m]'].to_numpy()[0]
    # baseline = input_space_dataframe['calibrated baseline [m]'].to_numpy()[0]
    r = 0.1651
    baseline = 0.512
    # min_wheel_vel = input_space_dataframe['maximum_wheel_vel_negative [rad/s]'].to_numpy()[0]
    min_wheel_vel = -7
    # max_wheel_vel = input_space_dataframe['maximum_wheel_vel_positive [rad/s]'].to_numpy()[0]
    max_wheel_vel = 7

if robot == 'warthog-tracks':
    input_space_dataframe = pd.read_pickle('/home/dominic/repos/norlab_WMRD/data/husky/input_space_data/input_space_data_ga.pkl')
    # r = input_space_dataframe['calibrated_radius [m]'].to_numpy()[0]
    # baseline = input_space_dataframe['calibrated baseline [m]'].to_numpy()[0]
    r = 0.3
    baseline = 1.1652
    min_wheel_vel = -15
    # min_wheel_vel = -5
    max_wheel_vel = 15
    # max_wheel_vel = 5

bounded_powertrain = Bounded_powertrain(min_wheel_vel, max_wheel_vel, time_constant=0.5, time_delay=0.05, dt=0.05)
init_params = [0.4, 0.05]
bounds = [(0.0, 5.0), (0.0, 1.0)]
method = 'Nelder-Mead'
# trained_params_path = 'training_results/marmotte/powertrain/boreal/powertrain_training_right.npy'
trained_params_path = '../data/ral2023_dataset/husky/boreal_snow/trained_params/powertrain/powertrain_training_right.npy'

powertrain_trainer = Powertrain_Trainer(powertrain_model=bounded_powertrain, init_params=init_params, dataloader=wmr_train_dl,
                              timesteps_per_horizon=timesteps_per_horizon, wheel_side='right', dt=0.05)
powertrain_trainer.train_model(init_params=init_params, method=method, bounds=bounds, saved_array_path=trained_params_path)

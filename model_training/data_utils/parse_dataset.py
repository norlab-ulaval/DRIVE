import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import minimize


from data_utils.dataset_parser import DatasetParser

# raw_dataset_path = '/media/dominic/462D-1EE8/rosbags/doughnut_calibs/warthog/depot_1/extracted_data/data.pkl'
raw_dataset_path = '/home/dominic/repos/norlab_WMRD/data/ral2023_dataset/warthog_wheels/gravel_1/warthog_wheels_gravel_1_data-raw.pkl'

# export_dataset_path = '/home/dominic/repos/norlab_WMRD/data/warthog_wheel/doughnut_datasets/depot_1/torch_dataset_all.pkl'
export_dataset_path = '/home/dominic/repos/norlab_WMRD/data/ral2023_dataset/warthog_wheels/gravel_1/torch_dataset_all.pkl'


robot = 'warthog-wheel'

if robot == 'husky':
    steady_state_step_len = 160
    wheel_radius = 0.33/2
    baseline = 0.55
    training_horizon = 2
    rate = 0.05

if robot == 'warthog-wheel':
    steady_state_step_len = 140
    wheel_radius = 0.3
    baseline = 1.1652
    training_horizon = 2
    rate = 0.05

if robot == 'warthog-track':
    steady_state_step_len = 140
    wheel_radius = 0.3
    baseline = 1.1652
    training_horizon = 2
    rate = 0.05

if robot == 'marmotte':
    steady_state_step_len = 160
    wheel_radius = 0.116
    baseline = 0.55
    training_horizon = 2
    rate = 0.05

## TODO: Import robot propreties from calibration data

dataset_parser = DatasetParser(raw_dataset_path, export_dataset_path, training_horizon, robot)

dataset_parser.process_data(5.0, -5.0, 6.0, -6.0)

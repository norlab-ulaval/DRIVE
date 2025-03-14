import numpy as np 
import pandas as pd 
import pathlib 
import yaml
from extractors import *
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive
import pickle 
import matplotlib.pyplot as plt 
global PATH_TO_METRIC 
global PATH_TO_SAVE_FOLDER
from datetime import datetime
import sys
import os
project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)
    

PATH_TO_METRIC = pathlib.Path('drive/model_training/data_utils/metric_config.yaml')
PATH_TO_SAVE_FOLDER = pathlib.Path('drive_datasets/results_multiple_terrain_dataframe/metric')
RESULTS_FILE_NAME = "metric_results.pkl"

PATH_TO_RESULT_FILE = PATH_TO_SAVE_FOLDER/RESULTS_FILE_NAME

if not PATH_TO_SAVE_FOLDER.is_dir():
    PATH_TO_SAVE_FOLDER.mkdir()


def compute_energy(vx,vy,omega_body, mass, inertia_constraints):
        """_summary_

        Args:
            vx (array): assuming that the vector is N by 1
            vy (_type_): assuming that the vector is N by 1
            omega_body (_type_): assuming that the vector is N by 1
        """

        translation_energy = 1/2 * mass * (vx**2+vy**2) 
        rotationnal_energy = 1/2 * mass * (inertia_constraints * omega_body**2)
        state_kin_energy =  translation_energy + rotationnal_energy

        
        return state_kin_energy,rotationnal_energy, translation_energy


if __name__ == "__main__":
    print("Hello world!")
    warthog_mass = 497.0
    husky_mass = 75.0
    warthog_vx = 5.0
    warthog_vy = 0.0
    warthog_omega = 0.0
    husky_vx = 1.0
    husky_vy = 0.0
    husky_omega = 0.0
    warthog_width = 1.52
    warthog_length = 2.13
    husky_width = 0.6604
    husky_length = 0.8382
    warthog_inertia = (warthog_width**2 + warthog_length**2)/12
    husky_inertia = (husky_width**2 + husky_length**2)/12

    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric_scatter.csv"
    df_warthog_metric = pd.read_csv(path_to_raw_result)
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric_scatter.csv"
    df_husky_metric = pd.read_csv(path_to_raw_result)
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog_vels = pd.read_csv(path_to_raw_result)
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky_vels = pd.read_csv(path_to_raw_result)
    
    df_warthog = pd.concat([df_warthog_metric, df_warthog_vels["cmd_body_lin_vel"], df_warthog_vels["cmd_body_yaw_vel"]], axis=1)
    df_husky = pd.concat([df_husky_metric, df_husky_vels["cmd_body_lin_vel"], df_husky_vels["cmd_body_yaw_vel"]], axis=1)
    warthog_energy = compute_energy(warthog_vx, warthog_vy, warthog_omega, warthog_mass, warthog_inertia)
    husky_energy = compute_energy(husky_vx, husky_vy, husky_omega, husky_mass, husky_inertia)
    print_column_unique_column(df_warthog)
    print("total_warthog_energy: ", warthog_energy[0])
    print("total_husky_energy: ", husky_energy[0])
    
    for terrain in df_warthog["terrain"].unique():
        df_terrain = df_warthog.loc[df_warthog.terrain==terrain]
        filtered_df = df_terrain # df_terrain.loc[(np.abs(df_terrain.cmd_body_yaw_vel) <= 0.5) & (np.abs(df_terrain.cmd_body_lin_vel) >= 1.0)]
        fig, axs = plt.subplots(1,1)
        ax = axs.scatter(filtered_df["cmd_body_yaw_vel"], filtered_df["cmd_body_lin_vel"], c=filtered_df["cmd_metric_total_energy_metric"],cmap="plasma")
        # axs.hist(filtered_df.cmd_metric_total_energy_metric, density=True)
        axs.set_xlabel('cmd_vel_yaw (rad/s)')
        axs.set_ylabel('cmd_vel_lin (m/s)')
        plt.title(terrain)
        plt.colorbar(ax)
        plt.show()

    for terrain in df_husky["terrain"].unique():
        df_terrain = df_husky.loc[df_husky.terrain==terrain]
        filtered_df = df_terrain.loc[(np.abs(df_terrain["cmd_body_yaw_vel"]) <= 2.0) & (np.abs(df_terrain.cmd_body_lin_vel <= 2.0))]
        fig, axs = plt.subplots(1,1)
        ax = axs.scatter(filtered_df["cmd_body_yaw_vel"], filtered_df["cmd_body_lin_vel"], c=filtered_df["cmd_metric_total_energy_metric"],cmap="plasma")
        plt.title(terrain)
        plt.colorbar(ax)
        plt.show()
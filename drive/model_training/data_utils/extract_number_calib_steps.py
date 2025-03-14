import pandas as pd
import shapely
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import pathlib
from shapely import concave_hull,MultiPoint,Point
import sys
import os
project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)
from extractors import *
import numpy as np


if __name__ == "__main__":

    steady_state_path = "drive_datasets/data/will/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    df_diamond = pd.read_pickle(steady_state_path)

    df_warthog = df_diamond.loc[df_diamond["robot"] == "warthog"]
    df_terrain = df_warthog.loc[df_warthog["terrain"] == "gravel"]

    print("Gravel terrain has", df_terrain.shape[0], "steps." )
    print("This is equal to", df_terrain.shape[0]*6, "seconds.")

    df_terrain = df_warthog.loc[df_warthog["terrain"] == "ice"]
    print("Ice terrain has", df_terrain.shape[0], "steps." )
    print("This is equal to", df_terrain.shape[0]*6, "seconds.")
    
    df_terrain = df_warthog.loc[df_warthog["terrain"] == "sand"]
    print("Sand terrain has", df_terrain.shape[0], "steps." )
    print("This is equal to", df_terrain.shape[0]*6, "seconds.")

    df_terrain = df_warthog.loc[df_warthog["terrain"] == "asphalt"]
    print("Asphalt terrain has", df_terrain.shape[0], "steps." )
    print("This is equal to", df_terrain.shape[0]*6, "seconds.")

    df_terrain = df_warthog.loc[df_warthog["terrain"] == "grass"]
    print("Grass terrain has", df_terrain.shape[0], "steps." )
    print("This is equal to", df_terrain.shape[0]*6, "seconds.")

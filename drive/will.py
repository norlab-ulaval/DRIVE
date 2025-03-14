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
import numpy as np



if __name__=="__main__":
    steady_state_path = "drive_datasets/data/warthog/wheels/asphalt/warthog_wheels_asphalt_2024_9_20_8h21s48/model_training_datasets/raw_dataframe.pkl"
    df = pd.read_pickle(steady_state_path)
    indices = df.index[df['calib_step'] == "51"].tolist()
    print(indices[119])
    cut_index = indices[119]
    df_cut = df.loc[:cut_index]
    pd.to_pickle(df_cut, "drive_datasets/data/warthog/wheels/asphalt/warthog_wheels_asphalt_2024_9_20_8h21s48/model_training_datasets/raw_dataframe_cut.pkl")
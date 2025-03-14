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
import yaml



def extract_time_values(df):
    times = df.calib_state.value_counts()
    print("Drive calib time:", times.calib * 1/20, "seconds.")
    print("Idle time:", times.idle * 1/20, "seconds.")
    print("Total time:", df.shape[0] * 1/20, "seconds.")

if __name__ == "__main__":
    path_to_drive_datasets_fodler = pathlib.Path("drive_datasets")
    path_to_data = path_to_drive_datasets_fodler/"data"

    black_list_topic_path = "drive/model_training/data_utils/update_drive_blacklist.yaml"

    with open(black_list_topic_path, 'r') as file:
        black_list_topic = yaml.load(file, Loader=yaml.FullLoader)

    for robot in path_to_data.iterdir():
        if robot.name == "to_class":
            print("\n"*5, "robot == to class")
            continue
        elif robot.name in black_list_topic["robot_to_skip"]:
            
            print("\n"*5, f"robot {robot.name} is in the blacklist")
            continue
        elif robot.name == "will":
            continue
        
        else:
            for traction in robot.iterdir():
                if robot == traction:
                        print("\n"*5, f"robot {robot} == traction {traction}")
                        continue # Case when the folder is empty
                        
                else:
                    
                    for terrain in traction.iterdir():
                        if terrain == traction:
                            print("\n"*5, f"terrain {terrain}== traction {traction}")
                            continue # Case when the folder is empty

                        elif terrain.name in black_list_topic["terrain_to_skip"]:
                            print("\n"*5, f"terrain {terrain.name} is in the blacklist")
                        else:
                            
                            for experiment in terrain.iterdir():
                                
                                print("Experiment : ", experiment)
                                raw_df_path =  experiment / "model_training_datasets/raw_dataframe.pkl"
                                df_raw = pd.read_pickle(raw_df_path)
                                extract_time_values(df_raw)

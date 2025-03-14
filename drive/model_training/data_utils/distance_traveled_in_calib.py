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


ratio = 0.1

def extract_minimum_sphere(ax,x,y,color, use_ratio=False):
    x_filtered = np.empty(0)
    y_filtered = np.empty(0)
    if use_ratio:
        for i in range(y.shape[0]):
            if y[i] > 1.9 or (y[i] < -1.71 and x[i] < -0.9) or (y[i] < -1.78 and x[i] > 0.86):
                pass
            else:
                x_filtered = np.append(x_filtered, x[i])
                y_filtered = np.append(y_filtered, y[i])
    else:
        x_filtered = x
        y_filtered = y
    points = np.concat((x_filtered.reshape(x_filtered.shape[0], 1), y_filtered.reshape(x_filtered.shape[0], 1)), axis=1)
    ax.scatter(x_filtered,y_filtered, edgecolor="none", color=color, alpha=0.2)
    points_shapely = MultiPoint(points)
    if use_ratio:
        polygon = concave_hull(points_shapely,ratio=ratio)
    else:
        polygon = concave_hull(points_shapely,ratio=1.0)

    # print(polygon)
    area = polygon.area
    x_filtered,y_filtered = polygon.exterior.xy
    ax.fill(x_filtered, y_filtered, color=color, alpha=0.2,label="convex hull")  # Fill color with transparency
    
    #hull = ConvexHull(points)
    #area = hull.area
    #ax.fill(points[hull.vertices, 0], points[hull.vertices, 1], color=color, alpha=0.1,label="convex hull")  # Fill color with transparency

    return ax, area,polygon

def extract_distance_travelled(df):
    distance = 0.0
    for i in range(df.shape[0]):
        try:
            if df.calib_state[i]=="calib":
                x1 = df.icp_pos_x[i]
                y1 = df.icp_pos_y[i]
                z1 = df.icp_pos_z[i]
                x2 = df.icp_pos_x[i+1]
                y2 = df.icp_pos_y[i+1]
                z2 = df.icp_pos_z[i+1]   
                distance += np.sqrt(np.power(float(x2)-float(x1), 2) + np.power(float(y2)-float(y1), 2) + np.power(float(z2)-float(z1), 2))
        except Exception:
                pass
    # print("Number of steps:", df.calib_step.unique())
    print("Distance travelled in calib mode:", distance)

def compute_area(df):
    x = df["icp_pos_x"].to_numpy().astype("float")
    y = df["icp_pos_y"].to_numpy().astype("float")
    print(x)
    fig,ax = plt.subplots(1, 1)
    ax, area, poly = extract_minimum_sphere(ax=ax, x=x, y=y, color="blue")
    print("Area covered by the driving zone is ", area, "m^2")

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
                                # print_column_unique_column(df_raw)
                                extract_distance_travelled(df_raw)
                                compute_area(df_raw)
                                # plt.show()

                                

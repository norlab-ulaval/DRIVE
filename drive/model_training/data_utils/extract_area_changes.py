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
ratio=1.0

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
    ax.scatter(x_filtered,y_filtered)
    points_shapely = MultiPoint(points)
    if use_ratio:
        polygon = concave_hull(points_shapely,ratio=ratio)
    else:
        polygon = concave_hull(points_shapely,ratio=1.0)

    # print(polygon)
    area = polygon.area
    x,y = polygon.exterior.xy
    ax.fill(x, y, color=color, alpha=0.2,label="convex hull")  # Fill color with transparency
    
    #hull = ConvexHull(points)
    #area = hull.area
    #ax.fill(points[hull.vertices, 0], points[hull.vertices, 1], color=color, alpha=0.1,label="convex hull")  # Fill color with transparency

    return ax, area,polygon



def plot_command_space(df,ax):
    alpha_parama = 0.2
    ax_to_plot = ax
    #ax_to_plot.scatter(df["cmd_body_yaw_lwmean"],df["cmd_body_x_lwmean"],color = "orange",label='Command',alpha=alpha_parama)
    ax_to_plot.set_xlabel("Angular velocity (omega) [rad/s]")
    ax_to_plot.set_ylabel("Forward velocity (V_x) [m/s]")
    # print_column_unique_column(df)
    ax,area,polygon = extract_minimum_sphere(ax,df["cmd_body_x_lwmean"].to_numpy(),df["cmd_body_yaw_lwmean"].to_numpy(),"orange", use_ratio=False)
    
    return ax, area

def plot_cmd_on_losange(path_to_cmd_vel,df,label,area_list,color_ref="grey"):

    # cmd_vel = pd.read_csv(path_to_cmd_vel).to_numpy()

    # print(cmd_vel.shape)

    fig, ax = plt.subplots(1,1)

    fig.subplots_adjust(hspace=0.5,wspace=0.5)
    ax, area_cmd = plot_command_space(df,ax)
    ax,area_achieved,polygon = extract_minimum_sphere(ax,df["icp_vel_x_smoothed"].to_numpy(),df["icp_vel_yaw_smoothed"].to_numpy(),color_ref, use_ratio=False)
    # print("Percentage of the cmd area lost:", 100 - (area_achieved/area_cmd) * 100)
    # print("area achieved: ", area_achieved)
    area_list.append(area_achieved)
    # list_within = []
    # for i in range(cmd_vel.shape[0]):
    #     point = Point(cmd_vel[i,1],cmd_vel[i,0])
        
    #     list_within.append(point.within(polygon))
    
    
    # percentage_of_point_not_in = np.round((1- sum(list_within)/len(list_within))*100,2)

    # print(percentage_of_point_not_in)
    

    # ax.set_title(f"{percentage_of_point_not_in} % of the cmd are out of the zone for gravel")
    # ax.scatter(cmd_vel[:,1],cmd_vel[:,0], label="cmd_snow",alpha =0.3)
    ax.legend(label)

    return ax




if __name__ == "__main__":

    steady_state_path = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    df_diamond = pd.read_pickle(steady_state_path)
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"brown","grass":"green","tile":"pink","boreal":"lightgray"}
    terrain_list_warthog = ["asphalt", "grass", "mud"] #, "sand", "ice"]
    df_warthog = df_diamond.loc[df_diamond["robot"]=="husky"]
    print_column_unique_column(df_warthog)
    area_list = []
    for i in terrain_list_warthog:
        df_terrain = df_warthog.loc[df_warthog["terrain"]==i]
        # print_column_unique_column(df_warthog)

        path_to_cmd_vel_csv = "drive_datasets/results_multiple_terrain_dataframe/area_test/combined_output.csv"

        label = ["CMD space sampled by drive", "steady_state speed on gravel", "CMD in snow"]
        plot_cmd_on_losange(path_to_cmd_vel_csv,df_terrain,label,color_ref="grey", area_list=area_list)
        # plt.show()
    for i in range(0, len(terrain_list_warthog)):
        print(terrain_list_warthog[i], 'percentage achieved: ', 100 * area_list[i]/max(area_list))

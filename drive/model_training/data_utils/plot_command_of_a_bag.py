import pandas as pd 
import shapely 
from drive.model_training.data_utils.extractors import *
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import pathlib
from shapely import concave_hull,MultiPoint,Point

import numpy as np 


def extract_minimum_sphere(ax,x,y,color):

    points = np.concat((x.reshape((x.shape[0],1)),y.reshape((x.shape[0],1))),axis=1)

    points_shapely = MultiPoint(points)
    polygon = concave_hull(points_shapely,ratio=0.3)
    print(polygon)
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
    
    ax,area,polygon = extract_minimum_sphere(ax,df["cmd_body_yaw_lwmean"].to_numpy(),df["cmd_body_x_lwmean"].to_numpy(),"orange")
    
    return ax, area

def plot_cmd_on_losange(path_to_cmd_vel,df,label,color_ref="grey"):

    cmd_vel = pd.read_csv(path_to_cmd_vel).to_numpy()

    print(cmd_vel.shape)

    fig, ax = plt.subplots(1,1)

    fig.subplots_adjust(hspace=0.5,wspace=0.5)
    ax, area = plot_command_space(df,ax)
    ax,area,polygon = extract_minimum_sphere(ax,df["icp_vel_yaw_smoothed"].to_numpy(),df["icp_vel_x_smoothed"].to_numpy(),color_ref)
    
    list_within = []
    for i in range(cmd_vel.shape[0]):
        point = Point(cmd_vel[i,1],cmd_vel[i,0])
        
        list_within.append(point.within(polygon))
    
    
    percentage_of_point_not_in = np.round((1- sum(list_within)/len(list_within))*100,2)

    print(percentage_of_point_not_in)
    

    ax.set_title(f"{percentage_of_point_not_in} % of the cmd are out of the zone for gravel")
    ax.scatter(cmd_vel[:,1],cmd_vel[:,0], label="cmd_snow",alpha =0.3)
    ax.legend(label)

    return ax




if __name__ == "__main__":

    steady_state_path = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    df_diamond = pd.read_pickle(steady_state_path)
    
    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"brown","grass":"green","tile":"pink","boreal":"lightgray"}

    df_terrain = df_diamond.loc[df_diamond["terrain"]=="gravel"]


    path_to_cmd_vel_csv = "/home/nicolassamson/workspaces/data/snow/cmd_vel/cmd_vel/cmd_vel.csv"

    label = ["CMD space sampled by drive", "steady_state speed on gravel", "CMD in snow"]
    plot_cmd_on_losange(path_to_cmd_vel_csv,df_terrain,label,color_ref="grey")
    plt.show()

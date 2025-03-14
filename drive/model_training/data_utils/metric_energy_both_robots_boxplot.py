import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np 
import matplotlib.gridspec as gridspec
from matplotlib.patches import Ellipse
import matplotlib as mpl
import matplotlib.patches as mpatches
import sys
import os
project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)
from extractors import *

def boxplot_all_terrain_all_robot(df,alpha_param=0.2,alpha_bp=0.4,path_to_save="figure/fig_robot_comparison_metric_boxplot.pdf"):

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plot_fs = 12

    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=10)
    plt.rc('ytick', labelsize=10)
    plt.rc('axes', labelsize=10)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0

    fig = plt.figure(figsize=(88/25.4, 3))
    axs = fig.add_subplot(111)
    
    fig.subplots_adjust(left=0.2)
    
    list_array_total = []
    list_terrain = []
    list_robot = []
    list_robot_name = []
    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
                
    for terrain in ["asphalt", "grass"]:
        df_terrain = df.loc[df.terrain==terrain]
        nb_robot = 0
        if terrain == "tile":
            continue
        for robot in df_terrain.robot.unique():

            nb_robot += 1 
            df_robot = df_terrain.loc[df_terrain.robot==robot]
            
            list_array_total.append(df_robot.total_energy_metric.loc[df_robot["terrain"] == terrain])

            list_robot_name.append(robot)
        list_robot.append(nb_robot)
        list_terrain.append(terrain)
    
    
    # Compute the position 
    delta_x = 0.5
    delta_same_terrain = 0.35
    position = 0
    list_position = []
    box_width = 0.3
    list_pos_labels = []
    list_pos_hfill = []
    # Compute the pos of boxes and pose of terrain labels
    pos_labels = 0
    pos_hfill = 0
    for value in list_robot:
        list_pos_hfill.append(pos_hfill)
        if value != 1:
            for i in range(1,value+1,1):
                if i ==1:
                    position += delta_x 
                    list_position.append(position)
                else:
                    position += delta_same_terrain 
                    list_position.append(position)
            pos_labels += delta_x + (delta_same_terrain/2) 
            list_pos_labels.append(pos_labels)
            pos_labels += (delta_same_terrain/2)
            #pos_labels += (value/2 * delta_same_terrain) 
        else:
            position += delta_x
            pos_labels += delta_x
            list_position.append(position)
            list_pos_labels.append(pos_labels)
            
        pos_hfill = position + delta_x/2
        list_pos_hfill.append(pos_hfill)

    box = axs.boxplot(list_array_total,showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box in [box]:
        
        for patch, terrain in zip(box['boxes'], ["asphalt", "asphalt", "grass", "grass"]):

            patch.set_facecolor(color_dict[terrain])  # Change to your desired color
            patch.set_alpha(alpha_bp)
        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')

    axs.vlines(1.1,ymax=5,ymin=0,color="black",alpha=alpha_bp, linewidth=0.75, linestyles="--")
    ticks = [0.5, 0.85, 1.35, 1.7]
    tick_labels = ["Warthog", "Husky", "Warthog", "Husky"]
    axs.set_xticks(ticks, tick_labels)
    axs.set_ylabel("Difficulty metric \n total energy (SI)")

    for ax in np.ravel(axs):
        ax.set_xlim(delta_x/2,list_pos_hfill[-1])
        ax.set_ylim(-0.02,1.02)
    ## Add the color fill 
    j = 1 
    print(list_position)
    print(list_pos_hfill)

    print(list_terrain)
    legend_labels = ["Asphalt", "Grass", "Grass"]
    axs.legend(labels=legend_labels)
    leg = axs.get_legend()
    handles = leg.legend_handles
    handles = [handles[0], handles[-1]]
    legend_labels = [legend_labels[0], legend_labels[-1]]
    axs.legend(handles, legend_labels, prop={'size': 10})

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    
def reorder_boxplot(list_array, list_color,list_terrain):

    # compute the list of median 
    list_median = []
    for terrain in list_array:
        list_median.append(terrain.median())
    # List median 
    np_array = np.array(list_median)

    # Order to applied
    order = np.argsort(np_array)
    
    list_data = []
    list_color_oredered = []
    list_terrain_reoredered = []
    for order_i in order:
        list_data.append(list_array[order_i])
        list_color_oredered.append(list_color[order_i])
        list_terrain_reoredered.append(list_terrain[order_i])

    return list_data, list_color_oredered, list_terrain_reoredered
    
if __name__ =="__main__":
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog = pd.read_csv(path_to_raw_result)
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky = pd.read_csv(path_to_raw_result)
    #df_husky = df_husky.drop()
    df = pd.concat([df_warthog,df_husky],axis=0)
    print_column_unique_column(df)
    filtered_df = df.loc[(np.abs(df["cmd_body_yaw_vel"]) <= 4.0)]
    df_warthog = filtered_df.loc[filtered_df.robot == "warthog"]
    df_husky = filtered_df.loc[filtered_df.robot == "husky"]


    #boxplot(df)
    boxplot_all_terrain_all_robot(filtered_df)
    #boxplot_few_robot_few_terrain(df)
    #print(df.columns)
    #plot_scatter_metric(df)
    #plot_histogramme_metric(df)
    median_w_a = np.median(np.abs(df_warthog.total_energy_metric.loc[df_warthog["terrain"] == "asphalt"]))
    median_w_g = np.median(np.abs(df_warthog.total_energy_metric.loc[df_warthog["terrain"] == "grass"]))
    median_h_a = np.median(np.abs(df_husky.total_energy_metric.loc[df_husky["terrain"] == "asphalt"]))
    median_h_g = np.median(np.abs(df_husky.total_energy_metric.loc[df_husky["terrain"] == "grass"]))
    print("warthog increment: ", median_w_a - median_w_g)
    print("husky increment: ", median_h_a - median_h_g)

    plt.show()



    print(0.40732918650830996)
    print(0.75 / 0.40732918650830996)
import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np 
import matplotlib.gridspec as gridspec
from matplotlib.patches import Ellipse
import matplotlib as mpl
import matplotlib.patches as mpatches
import sys
import os
import pickle
import matplotlib.patches as patches
from matplotlib.ticker import FuncFormatter
from shapely import Point
import shapely
import matplotlib.colors as mcolors 
#from drive.model_training.data_utils.metric_energy_boxplot import keep_only_steady_state_and_filter
project_root = os.path.abspath("/home/nicolassamson/ros2_ws/src/DRIVE")
if project_root not in sys.path:
    sys.path.append(project_root)
from drive.model_training.data_utils.extractors import *

OVERALL = False
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

def extract_data(df, art_dico,color_dict,abs=False):

    
    dico_data = {} 
    
    for terrain in list(df.terrain.unique()):
        df_terrain = df.loc[df.terrain==terrain]
        nb_robot = 0
        
        for robot in list(df_terrain.robot.unique()):
            
            
            if terrain == "tile":
                continue
            else:

                nb_robot += 1 
                df_robot = df_terrain.loc[df_terrain.robot==robot]
                
                # Create the params
                robot_dico = art_dico[robot] # Extract the robot
                dico = {"color":color_dict[terrain], "robot":robot}
                for keys, value in robot_dico.items():
                    if keys == "data":
                        
                        # Extracts all the data we want
                        for data_id, data_col in value.items():
                            if abs:
                                dico[data_id] = np.abs(df_robot[data_col])
                            else:
                                dico[data_id] = df_robot[data_col]
                    else:
                        dico[keys] = value
                dico["terrain"] = terrain
                dico_data[f"{terrain}_{robot}"] = dico
    return dico_data

def boxplot_all_terrain_warthog_robot(df,alpha_param=0.3,robot="warthog", 
                                    alpha_bp=0.4,path_to_save="figure/fig_slip_boxplot.pdf",
                                    linewidth_overall = 5):

    df = df.loc[df.robot == robot]

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=8)
    plt.rc('ytick', labelsize=8)
    plt.rc('axes', labelsize=8)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0

    fig, axs = plt.subplots(3,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.0)
    fig.subplots_adjust(left=.13, bottom=.08, right=.99, top=.97,hspace=0.1)
    #fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    list_array_x = []
    list_array_y = []
    list_array_rot = []
    list_terrain = []
    list_robot = []
    list_robot_name = []
    
    dico_robot_name = {"husky":"orange","warthog": "grey","Overall":"blue"}
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"#FFA500",
                "grass":"green","sand":"orangered","avide":"grey",
                "avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral",
                "Overall":"white"}
    
     # Compute and reorder the boxplot 
    for terrain in list(df.terrain.unique()):
        df_terrain = df.loc[df.terrain==terrain]
        nb_robot = 0
        if terrain == "tile":
            continue
        else:
            
            nb_robot += 1 
            df_robot = df_terrain.loc[df_terrain.robot==robot]
            
            
            list_array_x.append(np.abs(df_robot.slip_body_x_ss.loc[df_robot["terrain"] == terrain]))
            list_array_y.append(np.abs(df_robot.slip_body_y_ss.loc[df_robot["terrain"] == terrain]))
            list_array_rot.append(np.abs(df_robot.slip_body_yaw_ss.loc[df_robot["terrain"] == terrain]))

            list_robot_name.append(robot)
        list_robot.append(nb_robot)
        list_terrain.append(terrain)
    list_robot_name.append(robot)

    list_color = [color_dict[terrain] for terrain in df.terrain.unique()]

    list_array_x, list_color_x,list_terrain_reordered_x = reorder_boxplot(list_array_x, list_color,list_terrain )
    list_array_y, list_color_y,list_terrain_reordered_y = reorder_boxplot(list_array_y, list_color,list_terrain)
    list_array_rot, list_color_rot,list_terrain_reordered_rot = reorder_boxplot(list_array_rot, list_color,list_terrain)
    
    
    # for _list in [list_array_x, list_color_x, list_terrain_reordered_x]:
    #     tmp = _list[0]
    #     _list[0] = _list[2]
    #     _list[2] = tmp
    
    # for _list in [list_array_y, list_color_y, list_terrain_reordered_y]:
    #    tmp = _list[0]
    #    _list[0] = _list[2]
    #    _list[2] = tmp
    
    # for _list in [list_array_rot, list_color_rot, list_terrain_reordered_rot]:
    #     tmp = _list[0]
    #     _list[0] = _list[1]
    #     _list[1] = tmp


    #Add the overall 
    list_array_x.append([item for sublist in list_array_x for item in sublist])
    list_array_y.append([item for sublist in list_array_y for item in sublist])
    list_array_rot.append([item for sublist in list_array_rot for item in sublist])
    list_terrain.append("Overall")
    list_color_x.append("white")
    list_color_y.append("white")
    list_color_rot.append("white")
    list_terrain_reordered_rot.append("Overall")
    list_robot.append(1)
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
        else:
            position += delta_x
            pos_labels += delta_x
            list_position.append(position)
            list_pos_labels.append(pos_labels)
            
        pos_hfill = position + delta_x/2
        list_pos_hfill.append(pos_hfill)

    box1 = axs[0].boxplot(list_array_x,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box2 = axs[1].boxplot(list_array_y,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box3 = axs[2].boxplot(list_array_rot,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box, color_list in zip([box1,box2,box3], [list_color_x, list_color_y, list_color_rot]):
        
        for patch, color in zip(box['boxes'], color_list):

            patch.set_facecolor(color)  # Change to your desired color
            patch.set_alpha(alpha_bp)
        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')

    for ax in np.ravel(axs):
        ax.set_xticks([])       # Remove the ticks
        ax.set_xticklabels([])  # Remove the labels
        
    axs[0].set_ylabel("Longitudinal slip (m/s)")
    axs[1].set_ylabel("Lateral slip (m/s)")
    axs[2].set_ylabel("Angular slip (rad/s)")
    tick_labels = ['Gravel', 'Grass', 'Asphalt', 'Sand', 'Ice', 'Overall']
    ticks = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    axs[2].set_xticks(ticks, tick_labels)

    for ax in np.ravel(axs):
        ax.set_xlim(delta_x/2,list_pos_hfill[-1])
        # ax.set_ylim(0,1)
    axs[0].set_ylim(-0.1, 2.1)
    axs[1].set_ylim(-0.1, 2.1)
    axs[2].set_ylim(-0.1, 6)
    
    # Add the vertical thick line 
    # axs[0].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[1].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[2].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    fig.savefig(path_to_save[:-4]+".svg",dpi=300)
    

def slip_boxplot_both_robot(df,alpha_param=0.3, 
                                    alpha_bp=0.4,path_to_save="figure/fig_slip_boxplot_combined.pdf",
                                    linewidth_overall = 5, plot_slip_angle=True,abs= True):

    

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=8)
    plt.rc('ytick', labelsize=8)
    plt.rc('axes', labelsize=8)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0

    fig, axs = plt.subplots(3,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.0)
    fig.subplots_adjust(left=.13, bottom=.08, right=.99, top=.97,hspace=0.1)
    #fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    list_array_x = []
    list_array_y = []
    list_array_rot = []

    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"#FFA500",
                "grass":"green","sand":"orangered","avide":"grey",
                "avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral",
                "Overall":"white"}
    
    information_dico = {
        "husky":{
            "linestyle": "--",

            "data":
                {
                "Longitudinal slip (m/s)":"slip_body_x_ss",
                "Lateral slip (m/s)":"slip_body_y_ss",
                "Angular slip (rad/s)":"slip_body_yaw_ss"
                },
            },
        "warthog": {
            "linestyle": "-",

            "data":
                {
                "Longitudinal slip (m/s)":"slip_body_x_ss",
                "Lateral slip (m/s)":"slip_body_y_ss",
                "Angular slip (rad/s)":"slip_body_yaw_ss"
                }
            }
        }
    

    dico_data = extract_data(df, information_dico,color_dict ,abs=abs)
    
    dico_data[f"overall_husky"] = {"color":"white", "robot":"husky", 
                                "Longitudinal slip (m/s)":df["slip_body_x_ss"].loc[df.robot=="husky"].abs(),
                                "Lateral slip (m/s)":df["slip_body_y_ss"].loc[df.robot=="husky"].abs(),
                                "Angular slip (rad/s)":df["slip_body_yaw_ss"].loc[df.robot=="husky"].abs(),
                                "terrain":"Overall",
                                "linestyle":"--" }
    
    dico_data[f"overall_warthog"] = {"color":"white", "robot":"warthog", 
                                "Longitudinal slip (m/s)":df["slip_body_x_ss"].loc[df.robot=="warthog"].abs(),
                                "Lateral slip (m/s)":df["slip_body_y_ss"].loc[df.robot=="warthog"].abs(),
                                "Angular slip (rad/s)":df["slip_body_yaw_ss"].loc[df.robot=="warthog"].abs(),
                                "linestyle":"-",
                                "terrain":"Overall",}
    


    
    order_to_present = ["gravel_warthog", "grass_warthog","grass_husky", "asphalt_warthog",
                        "asphalt_husky", "sand_warthog","ice_warthog", "overall_warthog", "overall_husky"]
    big_delta = 0.30 
    small_delta = 0.20 
    box_width = 0.15
    delta_start = 0.10
    list_position = np.array([big_delta,2*big_delta,2*big_delta+small_delta, 3* big_delta+small_delta, 3* big_delta+ 2 * small_delta,  
                            4* big_delta+ 2 * small_delta, 5* big_delta+ 2 * small_delta ,
                            6* big_delta+ 2 * small_delta,6* big_delta+ 3 * small_delta ]) - (big_delta + delta_start)
    
    pos_vlines = 5.5* big_delta+ 2 * small_delta- (big_delta + delta_start)
    
    list_color_total = [dico_data[value]["color"] for value in order_to_present]
    list_patch_linestyle = [dico_data[value]["linestyle"] for value in order_to_present]


    #Add the overall 
    list_array_x = [dico_data[value]["Longitudinal slip (m/s)"] for value in order_to_present]
    list_array_y = [dico_data[value]["Lateral slip (m/s)"] for value in order_to_present]
    list_array_rot = [dico_data[value]["Angular slip (rad/s)"] for value in order_to_present]
    

    box1 = axs[0].boxplot(list_array_x,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box2 = axs[1].boxplot(list_array_y,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box3 = axs[2].boxplot(list_array_rot,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box, color_list, linestyle_list in zip([box1,box2,box3], 
                            [list_color_total, list_color_total, list_color_total],
                            [list_patch_linestyle, list_patch_linestyle, list_patch_linestyle]):
        
        for patch, color, linestyle in zip(box['boxes'], color_list,linestyle_list):

            patch.set_facecolor(color)  # Change to your desired color
            patch.set_alpha(alpha_bp)
            patch.set_linestyle(linestyle)

        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')

        for i, linestyle in enumerate(linestyle_list):
            box['whiskers'][i*2].set_linestyle(linestyle)
            box['whiskers'][i*2 + 1].set_linestyle(linestyle)
            box['caps'][i*2].set_linestyle(linestyle)
            box['caps'][i*2 + 1].set_linestyle(linestyle)

    for ax in np.ravel(axs):
        ax.set_xticks([])       # Remove the ticks
        ax.set_xticklabels([])  # Remove the labels

        
        
    axs[0].set_ylabel("Longitudinal slip (m/s)")
    axs[1].set_ylabel("Lateral slip (m/s)")
    axs[2].set_ylabel("Angular slip (rad/s)")

    tick_labels = ['Gravel', 'Grass', 'Asphalt', 'Sand', 'Ice', "Overall"]
    ticks = np.array([big_delta, 2*big_delta+small_delta/2,  3*big_delta+3*small_delta/2,  
                    4* big_delta+ 2 * small_delta,  5* big_delta+ 2 * small_delta,
                    6* big_delta+ 5/2 * small_delta ]) - (big_delta + delta_start)
    axs[2].set_xticks(ticks, tick_labels)

    for ax in np.ravel(axs):
        ax.set_xlim(min(list_position)-small_delta, max(list_position)+small_delta)
    
    axs[0].set_ylim(-0.1, 2.1)
    axs[1].set_ylim(-0.1, 2.1)
    axs[2].set_ylim(-0.1, 6)
    
    for ax in axs:
        ylim =ax.get_ylim()
        ax.vlines(pos_vlines,ymax=ylim[0],ymin=ylim[1],
                color="black",alpha=0.5,linewidth=0.75, linestyles="-.")
    
    # Add the vertical thick line 
    # axs[0].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[1].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[2].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    fig.savefig(path_to_save[:-4]+".svg",dpi=300)
    

def slip_boxplot_both_robot_slip_angle_added(df,alpha_param=0.3, 
                                    alpha_bp=0.4,path_to_save="figure/fig_slip_boxplot_combined_slip_angle.pdf",
                                    linewidth_overall = 5, plot_slip_angle=True,abs=True):
    
    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=8)
    plt.rc('ytick', labelsize=8)
    plt.rc('axes', labelsize=8)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0
    formatter = FuncFormatter(lambda x, pos: f'{x:.1f}')

    # Apply the formatter to the y-axis only
    
    fig, axs = plt.subplots(4,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.5)
    fig.subplots_adjust(left=.13, bottom=.15, right=.99, top=.97,hspace=0.1)
    #fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    list_array_x = []
    list_array_y = []
    list_array_rot = []

    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"#FFA500",
                "grass":"green","sand":"orangered","avide":"grey",
                "avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral",
                "Overall":"white"}
    
    information_dico = {
        "husky":{
            "linestyle": "-",
            "linecolor":"black",

            "data":
                {
                "Longitudinal slip (m/s)":"slip_body_x_ss",
                "Lateral slip (m/s)":"slip_body_y_ss",
                "Angular slip (rad/s)":"slip_body_yaw_ss",
                "Slip angle (rad)":"slip_angle_ss"
                },
            },
        "warthog": {
            "linestyle": "-",
            "linecolor":"black",
            "data":
                {
                "Longitudinal slip (m/s)":"slip_body_x_ss",
                "Lateral slip (m/s)":"slip_body_y_ss",
                "Angular slip (rad/s)":"slip_body_yaw_ss",
                "Slip angle (rad)":"slip_angle_ss"
                }
            }
        }
    

    dico_data = extract_data(df, information_dico,color_dict, abs  = abs)
    
    if OVERALL:
        dico_data[f"overall_husky"] = {"color":"white", "robot":"husky", 
                                    "Longitudinal slip (m/s)":df["slip_body_x_ss"].loc[df.robot=="husky"].abs(),
                                    "Lateral slip (m/s)":df["slip_body_y_ss"].loc[df.robot=="husky"].abs(),
                                    "Angular slip (rad/s)":df["slip_body_yaw_ss"].loc[df.robot=="husky"].abs(),
                                    "Slip angle (rad)": df["slip_angle_ss"].loc[df.robot=="husky"].abs(),
                                    "linestyle":"-",
                                    "linecolor":"black",
                                    "terrain":"Overall"}

        dico_data[f"overall_warthog"] = {"color":"white", "robot":"warthog", 
                                    "Longitudinal slip (m/s)":df["slip_body_x_ss"].loc[df.robot=="warthog"].abs(),
                                    "Lateral slip (m/s)":df["slip_body_y_ss"].loc[df.robot=="warthog"].abs(),
                                    "Angular slip (rad/s)":df["slip_body_yaw_ss"].loc[df.robot=="warthog"].abs(),
                                    "Slip angle (rad)": df["slip_angle_ss"].loc[df.robot=="warthog"].abs(),
                                    "linestyle":"-",
                                    "linecolor":"black",
                                    "terrain":"Overall"}
    
    #order_to_present = ["mud_husky","gravel_warthog", "grass_warthog","grass_husky", "asphalt_warthog",
    #                    "asphalt_husky", "sand_warthog","ice_warthog", "overall_warthog", "overall_husky"]
    if OVERALL:
        order_to_present = ["asphalt_husky","grass_husky","overall_husky", "mud_husky",
                            "gravel_warthog", "grass_warthog","asphalt_warthog","overall_warthog","sand_warthog","ice_warthog"
                            ]
    else:
        order_to_present = ["asphalt_husky","grass_husky", "mud_husky",
                            "gravel_warthog", "grass_warthog","asphalt_warthog","sand_warthog","ice_warthog"
                            ]
    
    big_delta = 0.15 
    small_delta = 0.20 
    box_width = 0.15
    delta_start = 0.10
    list_position = np.array([i* small_delta for i in range(1,len(order_to_present)+1)])
    
    if OVERALL:
        list_position[4:] += big_delta
        pos_vlines = np.mean(list_position[3:5])
    else:
        list_position[3:] += big_delta
        pos_vlines = np.mean(list_position[2:4])
    
    #list_position[-2:] += big_delta
    #list_position = np.array([big_delta,2*big_delta,3*big_delta, 3* big_delta+small_delta, 4* big_delta+ 1 * small_delta,  
    #                        4* big_delta+ 2 * small_delta, 5* big_delta+ 2 * small_delta , 6* big_delta+ 2 * small_delta, 
    #                        7* big_delta+ 2 * small_delta,7* big_delta+ 3 * small_delta ]) - (big_delta + delta_start)
    
    
    
    list_color_total = [dico_data[value]["color"] for value in order_to_present]
    list_patch_linestyle = [dico_data[value]["linestyle"] for value in order_to_present]
    list_colorline = [dico_data[value]["linecolor"] for value in order_to_present]

    #Add the overall 
    list_array_x = [dico_data[value]["Longitudinal slip (m/s)"] for value in order_to_present]
    list_array_y = [dico_data[value]["Lateral slip (m/s)"] for value in order_to_present]
    list_array_rot = [dico_data[value]["Angular slip (rad/s)"] for value in order_to_present]
    list_array_slip_angle = [dico_data[value]["Slip angle (rad)"] for value in order_to_present]


    box1 = axs[0].boxplot(list_array_x,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box2 = axs[1].boxplot(list_array_y,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box3 = axs[2].boxplot(list_array_rot,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)
    box4 = axs[3].boxplot(list_array_slip_angle,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)




    for box, color_list, linestyle_list,linecolor_list in zip([box1,box2,box3,box4], 
                            [list_color_total, list_color_total, list_color_total,list_color_total],
                            [list_patch_linestyle, list_patch_linestyle, list_patch_linestyle,list_patch_linestyle],
                            [list_colorline,list_colorline,list_colorline,list_colorline]):
        
        for patch, color, linestyle,linecolor in zip(box['boxes'], color_list,linestyle_list,list_colorline):
            rgb = mcolors.to_rgb(color)
            white =  (1.0,1.0,1.0)
            blended_rgb = tuple(
                alpha_bp * c + (1 - alpha_bp) * w
                for c, w in zip(rgb, white)
            )
            patch.set_facecolor(blended_rgb)  # Change to your desired color
            #patch.set_alpha(alpha_bp)
            patch.set_linestyle(linestyle)
            patch.set_edgecolor(linecolor)
        # Change the median line color to black
        for median,linecolor in zip(box['medians'],linecolor_list):
            median.set_color(linecolor)
        i = 0
        for linestyle,linecolor in zip(linestyle_list,list_colorline):
            
            box['whiskers'][i*2].set_linestyle(linestyle)
            box['whiskers'][i*2 + 1].set_linestyle(linestyle)
            box['caps'][i*2].set_linestyle(linestyle)
            box['caps'][i*2 + 1].set_linestyle(linestyle)

            if i >=4:
                box['whiskers'][i*2].set_color(linecolor)
                box['whiskers'][i*2 + 1].set_color(linecolor)
                box['caps'][i*2].set_color(linecolor)
                box['caps'][i*2 + 1].set_color(linecolor)
            i+=1
    for ax in np.ravel(axs):
        ax.set_xticks([])       # Remove the ticks
        ax.set_xticklabels([])  # Remove the labels

    for ax in np.ravel(axs):
        ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f'{x:.1f}'))
        
    axs[0].set_ylabel("Longitudinal slip (m/s)")
    axs[1].set_ylabel("Lateral slip (m/s)")
    axs[2].set_ylabel("Angular slip (rad/s)")
    axs[3].set_ylabel("Slip angle (rad)")
    axs[3].yaxis.set_major_formatter(formatter)

    #tick_labels = ["Mud",'Gravel', 'Grass', 'Asphalt', 'Sand', 'Ice', "Overall"]
    #ticks = np.array([big_delta, 2*big_delta,3*big_delta+small_delta/2,  4*big_delta+3*small_delta/2,  
    #                5* big_delta+ 2 * small_delta,   6* big_delta+ 2 * small_delta,
    #                7* big_delta+ 5/2 * small_delta ]) - (big_delta + delta_start)
    tick_labels = ["Husky","Warthog"]

    if OVERALL:
        ticks = np.array([2.5*small_delta,7.5*small_delta])
        ticks[1:] += big_delta
        ticks[2:] += big_delta
    else:
        ticks = np.array([2.0*small_delta,6*small_delta])
        ticks[1:] += big_delta
        ticks[2:] += big_delta
    axs[3].set_xticks(ticks, tick_labels)

    for ax in np.ravel(axs):
        ax.set_xlim(min(list_position)-small_delta, max(list_position)+small_delta)
    
    axs[0].set_ylim(-0.1, 1.1)
    axs[1].set_ylim(-0.1, 1.1)
    axs[3].set_ylim(-0.1, 0.6)
    axs[2].set_ylim(-0.1, 4.1)
    
    x0_terrain = np.mean(list_position[2:4])
    x1_terrain = np.mean(list_position[7:9])
    x2_terrain = np.mean(list_position[-2:])
    
    for ax in axs:
        ylim =ax.get_ylim()
        ax.vlines(pos_vlines,ymax=ylim[0],ymin=ylim[1],
               color="black",alpha=0.5,linewidth=0.75, linestyles="-.")
        #ax.add_patch(
        #    patches.Rectangle(
        #    (x0_terrain,ylim[0] ),            # (x0, y0) in data coordinates
        #    x1_terrain-x0_terrain, ylim[1]-ylim[0],            # width and height in data coordinates
        #    color='grey',
        #    zorder=0           # Behind the plot
        #    )
        #)
        # x3_terrain = ax.get_xlim()[1]
        # ax.add_patch(
            # patches.Rectangle(
            # (pos_vlines,ylim[0] ),            # (x0, y0) in data coordinates
            # x3_terrain-pos_vlines, ylim[1]-ylim[0],            # width and height in data coordinates
            # color='grey',
            # zorder=0           # Behind the plot
            # )
        # )
    # Add the vertical thick line 
    # axs[0].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[1].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[2].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")

    # Add legend to the Axes


    legend_handles = []
    # Custom legend patches
    terrain_list = []

    list_sorted_keys = list(dico_data.keys())
    list_sorted_keys.sort()
    print(list_sorted_keys)
    for key in list_sorted_keys:
        print(key)
        values = dico_data[key]
        if values["terrain"] not in terrain_list:
            label_no_cap = values["terrain"]
            label = label_no_cap[0].upper()+label_no_cap[1:]
            # if values["terrain"].lower() == "overall":
                # legend_handles.append(mpatches.Patch(facecolor=values["color"], label=label,alpha=alpha_bp,edgecolor="black"))
            # else:
                # legend_handles.append(mpatches.Patch(color=values["color"], label=label,alpha=alpha_bp))
            face_rgb = mcolors.to_rgba(values["color"], alpha=alpha_bp)
            legend_handles.append(mpatches.Patch(facecolor=face_rgb, label=label,edgecolor="black"))
            terrain_list.append(values["terrain"])
    
    if OVERALL:
        ax.legend(handles=legend_handles,loc='upper center',
            bbox_to_anchor=(0.5, -0.25),  # Adjust x=center, y=slightly below
            ncol=4, fontsize=8,
            handletextpad=1.0,     # space between handle and text
            columnspacing=1.5,    # Increase spacing between columns for better fit
            )      # space between columns)
    else:
        ax.legend(handles=legend_handles,loc='upper center',
        bbox_to_anchor=(0.5, -0.25),#0.42, -0.1),  # x=center, y=slightly below
        ncol=3, fontsize=8,
        handletextpad=1.0,     # space between handle and text
        columnspacing=1.0,

        )      # space between columns)

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    fig.savefig(path_to_save[:-4]+".svg",dpi=300)
    

def compute_slip_angle(df,column_vx= "step_frame_vx", column_vy= "step_frame_vy",nb_steady_state= 20):

    vx = column_type_extractor(df,column_vx)
    vy = column_type_extractor(df,column_vy)

    og_shape = vx.shape

    slip_angle = np.arctan(np.ravel(vy),np.ravel(vx)).reshape(og_shape)

    slip_angle_ss =np.mean(slip_angle[:,-nb_steady_state:],axis=1)
    
    #slip_angle_ss = np.rad2deg(slip_angle_ss)
    
    dico_data = create_columns_names_from_dict_with_names(["step_frame_slip_angle"],{"step_frame_slip_angle":slip_angle},{})

    df.reset_index(inplace=True)

    df2add = pd.DataFrame.from_dict(dico_data)
    df2add["slip_angle_ss"] = slip_angle_ss
    df_final = pd.concat([df,df2add],axis=1)

    return df_final



def export_graph_cmd(df):

    print_column_unique_column(df)
    dico_translate = { "slip_angle_ss":"slip_angle",
    "cmd_body_x_lwmean":"cmd_body_x_mean",
    "cmd_body_yaw_lwmean":"cmd_body_yaw_mean",
    "terrain":"terrain",
    "robot":"robot",
    "slip_body_x_ss":"slip_body_x_ss",
    "slip_body_y_ss":"slip_body_y_ss",
    "slip_body_yaw_ss":"slip_body_yaw_ss"
    }
    df_new = df[dico_translate.keys()]
    df_new.rename(dico_translate,inplace=True)

    
    return df_new


def slip_angle_boxplot_both_robot(df,alpha_param=0.3, 
                                    alpha_bp=0.4,path_to_save="figure/fig_slip_angle_boxplot_combined.pdf",
                                    linewidth_overall = 5, plot_slip_angle=True, deg=False,abs=False, violin= False):

    

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=8)
    plt.rc('ytick', labelsize=8)
    plt.rc('axes', labelsize=8)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0

    fig, axs = plt.subplots(1,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.0)
    fig.subplots_adjust(left=.13, bottom=.08, right=.99, top=.97,hspace=0.1)
    #fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"#FFA500",
                "grass":"green","sand":"orangered","avide":"grey",
                "avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral",
                "Overall":"white"}
    
    information_dico = {
        "husky":{
            "linestyle": "--",

            "data":
                {
                "Slip angle (rad)":"slip_angle_ss",
                },
            },
        "warthog": {
            "linestyle": "-",

            "data":
                {
                "Slip angle (rad)":"slip_angle_ss",
                }
            }
        }
    

    dico_data = extract_data(df, information_dico,color_dict, abs=abs)
    
    if abs:
        dico_data[f"overall_husky"] = {"color":"white", "robot":"husky", 
                                "Slip angle (rad)":df["slip_angle_ss"].loc[df.robot=="husky"].abs(),
                                "linestyle":"--" }
    
        dico_data[f"overall_warthog"] = {"color":"white", "robot":"warthog", 
                                "Slip angle (rad)":df["slip_angle_ss"].loc[df.robot=="warthog"].abs(),
                                "linestyle":"-"}
    
    else:
        dico_data[f"overall_husky"] = {"color":"white", "robot":"husky", 
                                "Slip angle (rad)":df["slip_angle_ss"].loc[df.robot=="husky"],
                                "linestyle":"--" }
    
        dico_data[f"overall_warthog"] = {"color":"white", "robot":"warthog", 
                                "Slip angle (rad)":df["slip_angle_ss"].loc[df.robot=="warthog"],
                                "linestyle":"-"}
    

    
    order_to_present = ["gravel_warthog", "grass_warthog","grass_husky", "asphalt_warthog",
                        "asphalt_husky", "sand_warthog","ice_warthog", "overall_warthog", "overall_husky"]
    big_delta = 0.30 
    small_delta = 0.20 
    box_width = 0.15
    delta_start = 0.10
    list_position = np.array([big_delta,2*big_delta,2*big_delta+small_delta, 3* big_delta+small_delta, 3* big_delta+ 2 * small_delta,  
                            4* big_delta+ 2 * small_delta, 5* big_delta+ 2 * small_delta ,
                            6* big_delta+ 2 * small_delta,6* big_delta+ 3 * small_delta ]) - (big_delta + delta_start)
    
    pos_vlines = 5.5* big_delta+ 2 * small_delta- (big_delta + delta_start)
    
    list_color_total = [dico_data[value]["color"] for value in order_to_present]
    list_patch_linestyle = [dico_data[value]["linestyle"] for value in order_to_present]


    #Add the overall 
    list_array_slip_angle = [dico_data[value]["Slip angle (rad)"] for value in order_to_present]
    
    if violin:
        #box1 = axs.violinplot(list_array_slip_angle,positions=list_position,widths=box_width)
        box1 = axs.violinplot(list_array_slip_angle, showmedians=True, positions=list_position, widths=box_width)
    else:
   
        box1 = axs.boxplot(list_array_slip_angle,whis=(2.5, 97.5),showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    i = 0
    for box, color_list, linestyle_list in zip([box1], 
                            [list_color_total, list_color_total, list_color_total],
                            [list_patch_linestyle, list_patch_linestyle, list_patch_linestyle]):
        
        if violin:
            iterate = box['bodies']
        else:
            iterate = box['boxes']
        for patch, color, linestyle in zip(iterate, color_list,linestyle_list):

            patch.set_facecolor(color)  # Change to your desired color
            patch.set_alpha(alpha_bp)
            patch.set_linestyle(linestyle)

       

        if not violin:
             # Change the median line color to black
            for median in box['medians']:
                median.set_color('black')

            for i, linestyle in enumerate(linestyle_list):
                box['whiskers'][i*2].set_linestyle(linestyle)
                box['whiskers'][i*2 + 1].set_linestyle(linestyle)
                box['caps'][i*2].set_linestyle(linestyle)
                box['caps'][i*2 + 1].set_linestyle(linestyle)
        else:

            for violon_i in box1["bodies"]:
                
                violon_i.set_edgecolor("k")
                violon_i.set_linestyle(linestyle_list[i])  # Change the edge color
                
                i += 1
            #for part in [, 'cmins', 'cmaxes']:
    for ax in np.ravel(axs):
        ax.set_xticks([])       # Remove the ticks
        ax.set_xticklabels([])  # Remove the labels

        
        
    axs.set_ylabel("Slip angle (rad)")

    tick_labels = ['Gravel', 'Grass', 'Asphalt', 'Sand', 'Ice', "Overall"]
    ticks = np.array([big_delta, 2*big_delta+small_delta/2,  3*big_delta+3*small_delta/2,  
                    4* big_delta+ 2 * small_delta,  5* big_delta+ 2 * small_delta,
                    6* big_delta+ 5/2 * small_delta ]) - (big_delta + delta_start)
    axs.set_xticks(ticks, tick_labels)

    axs.set_xlim(min(list_position)-small_delta, max(list_position)+small_delta)
    # for ax in np.ravel(axs):
        # ax.set_xlim(min(list_position)-small_delta, max(list_position)+small_delta)
    # 
    # axs[0].set_ylim(-0.1, 1.5)
    # axs[1].set_ylim(-0.1, 1.5)
    # axs[2].set_ylim(-0.1, 5)
    # 
    #for ax in axs:
    ylim =axs.get_ylim()
    axs.vlines(pos_vlines,ymax=ylim[0],ymin=ylim[1],
            color="black",alpha=0.5,linewidth=0.75, linestyles="-.")

    # Add the vertical thick line 
    # axs[0].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[1].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")
    # axs[2].vlines(list_pos_hfill[-3],ymax=10,ymin=-10,color="black",alpha=0.5, linewidth=0.75,linestyles="--")

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    fig.savefig(path_to_save[:-4]+".svg",dpi=300)
    
def filter_warthog_command_space_to_husky(df,debug=True,cols=["cmd_body_yaw_lwmean","cmd_body_x_lwmean"]):
    GEOM_PICKLE = "drive_datasets/results_multiple_terrain_dataframe/husky_geom_limits_by_terrain_for_filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    with open(GEOM_PICKLE,"rb") as f:
        geom_limits = pickle.load(f)
    
    df_warthog = df.loc[df["robot"]=="warthog"]

    cmd= df_warthog[cols].to_numpy()

    husky_limits = geom_limits["body"]["grass"]
    print(geom_limits)
    filter = []
    for i in range(cmd.shape[0]):

        pt = Point(cmd[i,:])
        
        if shapely.within(pt, husky_limits) or shapely.touches(pt,husky_limits):
            filter.append(True)
        else:
            filter.append(False)
    
    df_warthog = df_warthog.loc[filter]

    if debug:

        for terrain in df_warthog["terrain"].unique():
            terrain_data = df_warthog.loc[df_warthog["terrain"]==terrain].shape
            print(f"Shape of terrain {terrain} is {terrain_data}") 
        df_warthog.plot.scatter(y=cols[1],x=cols[0])
        print(df_warthog.shape)
        plt.show()
    return df_warthog

if __name__ =="__main__":
    
    path_to_warthog_results = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    df_warthog = pd.read_pickle(path_to_warthog_results)
    #filtered_df = keep_only_steady_state_and_filter(df_warthog,119,39,yaw_filter =4.0,
    #                                keep_only_steady_state = True,
    #                                filter_data = True,
    #                                col_to_filter_with = "cmd_body_yaw_lwmean")
    filtered_df = df_warthog.loc[(np.abs(df_warthog["cmd_body_yaw_lwmean"]) <=4.0)]
    boxplot_all_terrain_warthog_robot(filtered_df)

    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    df_husky = pd.read_pickle(path_to_raw_result)
    df_husky = compute_slip_angle(df_husky)
    filtered_df = compute_slip_angle(filtered_df)

    df_husky.to_pickle("drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset_slip_angle.pkl")
    filtered_df.to_pickle("drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset_slip_angle.pkl")
    
    df_combined =  df = pd.concat([filtered_df,df_husky],axis=0,ignore_index=True)
    
    absolute = True
    slip_angle_boxplot_both_robot(df_combined,abs=absolute)
    slip_angle_boxplot_both_robot(df_combined,violin=True,abs=absolute)

    df_husky_gma = export_graph_cmd(df_husky)
    df_warthog_gma = export_graph_cmd(filtered_df)
    
    df_husky_gma.to_csv("drive_datasets/results_multiple_terrain_dataframe/husky_slip_angle_gma.csv")
    df_warthog_gma.to_csv("drive_datasets/results_multiple_terrain_dataframe/warthog_slip_angle_gma.csv")
    

    
    slip_boxplot_both_robot_slip_angle_added(df_combined)
    # path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    # df_husky = pd.read_csv(path_to_raw_result)
    #df_husky = df_husky.drop()
    # df = pd.concat([df_warthog,df_husky],axis=0)
    #plt.hist(filtered_df.cmd_body_x_lwmean)
    #plt.show()
    # print_column_unique_column(df_warthog)
    #boxplot(df)
    
    #boxplot_few_robot_few_terrain(df)
    #print(df.columns)
    #plot_scatter_metric(df)
    #plot_histogramme_metric(df)
    # median_long_ice = np.median(np.abs(filtered_df.slip_body_x_ss.loc[filtered_df["terrain"] == "ice"]))
    # median_lat_ice = np.median(np.abs(filtered_df.slip_body_y_ss.loc[filtered_df["terrain"] == "ice"]))
    median_yaw_ice = np.median(np.abs(filtered_df.slip_body_yaw_ss.loc[filtered_df["terrain"] == "grass"]))
    # median_long_grass = np.median(np.abs(filtered_df.slip_body_x_ss.loc[filtered_df["terrain"] != "ice"]))
    # median_lat_grass = np.median(np.abs(filtered_df.slip_body_y_ss.loc[filtered_df["terrain"] != "ice"]))
    median_yaw_mud = np.median(np.abs(filtered_df.slip_body_yaw_ss.loc[filtered_df["terrain"] == "mud"]))
    median_yaw_asphalt = np.median(np.abs(filtered_df.slip_body_yaw_ss.loc[filtered_df["terrain"] == "asphalt"]))

    print("median grass: ", median_yaw_ice)
    print("median mud: ", median_yaw_mud)
    print("median asphalt: ", median_yaw_asphalt)

    median_yaw_overall = np.median(np.abs(filtered_df.slip_body_yaw_ss))
    print("median yaw overall: ", median_yaw_overall)
    median_long_overall = np.median(np.abs(filtered_df.slip_body_x_ss))
    print("median long overall: ", median_long_overall)
    median_lat_overall = np.median(np.abs(filtered_df.slip_body_y_ss))
    print("median lat overall: ", median_lat_overall)

    std_slip_late_mud =  np.std(df_husky.loc[df_husky["terrain"]=="mud"].slip_body_y_ss ) 

    std_slip_late_grass =  np.std(df_husky.loc[df_husky["terrain"]=="grass"].slip_body_y_ss )
    std_slip_late_asphalt =  np.std(df_husky.loc[df_husky["terrain"]=="asphalt"].slip_body_y_ss )

    print("std mud",std_slip_late_mud)
    print("std grass",std_slip_late_grass)
    print("std asphalt",std_slip_late_asphalt)

    print("std mud on asphalt",std_slip_late_mud/std_slip_late_asphalt)
    print("std mud on grass",std_slip_late_mud/std_slip_late_grass)
    
    
    plt.show()
    
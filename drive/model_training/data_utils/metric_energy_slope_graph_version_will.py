import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np 




import matplotlib.gridspec as gridspec

from matplotlib.patches import Ellipse
import matplotlib as mpl
from matplotlib.lines import Line2D
import sys
import os
project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)
from extractors import *


def moving_average(x,y,percentile=95.0, r = 0.01):


    
    x_windows = np.linspace(0,1,101)
    y_windows = np.zeros_like(x_windows)
    y_windows_std = np.zeros_like(x_windows)
    for i,x_window in enumerate(x_windows):

        mask =  np.abs(x-x_window) <= r

        x_masked = x[mask]
        y_masked = y[mask]

        if y_masked.size == 0:
            continue
        else:
            y_windows[i] = np.percentile(y_masked,percentile)
            y_windows_std[i] = np.std(y_masked)
    return x_windows,y_windows,y_windows_std

def plot_metric_scatter_scatter(df_res,alpha_param=0.4,suffix="",y_column="y_coordinates",
                                percentile_filtering=False,percentile=50,radius= 0.01,
                                path_to_save = "figure/fig_metric_will.pdf",
                                only_total_energy = False,dpi=300):

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

    plt.rc('font', **font)
    plot_fs = 12

    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=9)
    plt.rc('ytick', labelsize=9)
    plt.rc('axes', labelsize=10)
    mpl.rcParams['lines.dashed_pattern'] = [2, 2]
    mpl.rcParams['lines.linewidth'] = 1.0
    df = df_res.copy()
    #df.reset_index(inplace=True, names="terrain")

    # col = ["slope","slope_std","x_95","y_std_95"]
    

    fig = plt.figure(constrained_layout=True)
    
    #fig.subplots_adjust(hspace=0.4,wspace=0.2)
    
    # Define the gridspec layout with custom width_ratios
    # In this case, we have 2 columns, but we make the second column narrower
    
    #ax4 = fig.add_subplot(gs[0,0])
    #ax5 = fig.add_subplot(gs[1,0])
    #ax6 = fig.add_subplot(gs[2,0])
    #axs_to_turn_off = [ax4,ax5,ax6]
    
    fig.set_figwidth(88/25.4)
    fig.set_figheight(3.0)
    list_y_coordinates = ["_total_energy_metric"]
    labels = ["$95^{th}$ percentile \n kinetic energy (J)"]
    # Create subplots with gridspec
    gs = gridspec.GridSpec(2, 1, wspace=0.4,height_ratios=[4,3], figure=fig)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])#,sharex=ax1)
    # ax3 = fig.add_subplot(gs[2])
    n_size= 2
    axs = [ax1,ax2]
    df_husky = df.loc[df.robot=="husky"]
    df_warthog = df.loc[df.robot=="warthog"]
    df_ice = df_warthog.loc[df_warthog.terrain=="ice"]
    df_mud = df_husky.loc[df_husky.terrain=="mud"]
    ax1.scatter(df_ice.cmd_metric_total_energy_metric, df_ice.y_coordinates_total_energy_metric, c="blue")
    ax2.scatter(df_mud.cmd_metric_total_energy_metric, df_mud.y_coordinates_total_energy_metric, c="brown")

    # for metric_name,ax,ylabel in zip(list_y_coordinates,axs,labels):
        
        
    #     for robot in df.robot.unique():
    #         linestyle_dict = {"husky":"--","warthog":"-"}
    #         linestyle = linestyle_dict[robot]
    #         first_time_robot=True
    #         df_robot = df.loc[df.robot==robot]

            
    #         for terrain in np.unique(df_robot.terrain):
    #             df_terrain = df_robot.loc[df_robot.terrain==terrain]
    #             x = df_terrain["cmd_metric"+metric_name].to_numpy()
    #             y = df_terrain[y_column+metric_name].to_numpy()

    #             #y_95 = np.percentile(y,95)
    #             #mask = y <= y_95
    #             #x_masked = x[mask]
    #             #y_masked = y[mask]
            
    #             x_masked = x
    #             y_masked = y
    #             color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
                
                


    #             if percentile_filtering:
    #                 x_masked,y_masked,y_windows_std = moving_average(x,y,percentile=percentile,r = radius)
    #                 color_list = [color_dict[terrain]]*x_masked.shape[0]
    #                 # Scatter plot the data points 
    #                 if percentile==50:
    #                     ax.errorbar(x_masked, y_masked,yerr=y_windows_std, color=color_list[0],alpha=0.9,label=terrain)    
    #                 else:
    #                     test=1
                    
    #                     #ax.scatter(x_masked, y_masked, color=color_list,alpha=0.9,s=0.8)
    #                     ax.plot(x_masked, y_masked, color=color_list[0],alpha=alpha_param,label=terrain,ls=linestyle)
                    
                    
                    
    #             else:
    #                 color_list = [color_dict[terrain]]*x_masked.shape[0]
    #                 ax.scatter(x_masked, y_masked, color=color_list,alpha=0.2,s=0.8,label=terrain)
                
    #         x = df_robot["cmd_metric"+metric_name].to_numpy()
    #         y = df_robot[y_column+metric_name].to_numpy()
    #         x_masked,y_masked,y_windows_std = moving_average(x,y,percentile=percentile,r = radius)
    #         label_robot = f"{robot[0].capitalize()}{robot[1:]}"
    #         #label_robot[0].capitalize() 
    #         ax.plot(x_masked, y_masked, color="black",alpha=1,label=label_robot,ls=linestyle)
            
            
    #     ax.set_ylabel(f"{ylabel}")
                
    # Add labels and legend
    
    
    
    #axs[0].set_title('New mapping of DRIVE (mass include)'+suffix)
    
    for ax in np.ravel(axs):
        #ax.legend()
        ax.grid(True)
        ylimit = ax.get_ylim()
        ax.set_ylim(1,ylimit[1])
        ax.set_xlim(0,1)
        ax.set_yscale("log")

    #axs.reshape(3,1)

        # Extract legends from both axes
    # legend1 = axs[0].get_legend_handles_labels()

    # # Combine legends from both axes
    # handles = legend1[0] 
    # labels = legend1[1]
    
    # husky_labels = labels[:4]
    # husky_handles = handles[:4]
    # warthog_labels = labels[5:]
    # warthog_handles = handles[5:]
    # robot_label = [warthog_labels[-1][0].upper()+ warthog_labels[-1][1:], husky_labels[-1][0].upper()+husky_labels[-1][1:]]
    # robot_handles = [warthog_handles[-1], husky_handles[-1]]
    

    # final_labels = []
    # final_handles = []

    
    # terrain_label = labels[:3] + labels[4:-1]
    # terrain_handle = handles[:3] + handles[4:-1]

    # filtered_label = []
    # filtered_handle = []
    # ##
    # for label,handle  in zip(terrain_label,terrain_handle):
    #     if (label[0].upper()+label[1:]) in filtered_label:
    #         continue
    #     else:
    #         print(label)
    #         filtered_label.append(label[0].upper()+label[1:] )
    #         handle.set_linestyle("-")
    #         filtered_handle.append(handle)
    
    # filtered_handle.append(handles[-1])
    # filtered_label.append("All \n terrain")

        
    

    y_position_l1 = 0.37
    x_pos_l1 = 0.65
    # Create a legend in the figure (outside the axes)
    # for i,label in enumerate(final_labels):
    #     final_labels[i] = label[0].capitalize() + label[1:]
    
    # legend_terrain  = fig.legend(filtered_handle, filtered_label, 
    #         ncols=2,bbox_to_anchor = (x_pos_l1,y_position_l1),
    #         columnspacing=0.4,title=r"$\mathbf{Terrain}$",
    #         labelspacing=0.1,
    #         handletextpad=0.3)
    
    # Get the bounding box of the legend
    # bbox = legend_terrain.get_window_extent()
    # legend_height = bbox.height / dpi  # Height in inches
    # legend_width =  bbox.width / dpi 
    
    # print(x_pos_l1,legend_width)
    # fig.legend(robot_handles, robot_label, 
    #         bbox_to_anchor = (x_pos_l1+legend_width/2+0.02 ,y_position_l1),
    #         ncols=1,
    #         columnspacing=0.4,title=r"$\mathbf{Robot}$",
    #         labelspacing=0.1,
    #         handletextpad=0.3)
    
    
    #fig.legend(final_handles, final_labels, 
    #        loc='lower right',ncols=3,
    #        columnspacing=0.3,title=r"$\textbf{P95 of Energy}$",
    #        labelspacing=0.1)
    # Reput the husky line in dashe
    # for handle in husky_handles:
    #     handle.set_linestyle("--")
    #for handle in warthog_handles:
    #    handle.set_linestyle("-")
    for ax in np.ravel(axs[1:]):
        ax.axis("off")
        ax.set_yticklabels([])
        ax.set_yticks([])
        ax.grid(False)
        ax.set_axis_off()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
    
    axs[n_size-2].set_xlabel("Difficulty metric (SI)")

    fig.savefig(fname=path_to_save,dpi=dpi)
    #axs[2].set_xticklabels([])
    print(gs)

if __name__ =="__main__":
    
    #PATH_TO_RESULT = "drive_datasets/results_multiple_terrain_dataframe/metric/results_slope_metric.csv"
    #df = pd.read_csv(PATH_TO_RESULT)
    #print(df.columns)
    #plot_metric_3d(df,suffix="")
    #plt.show()
    #plot_metric_depending_on_sampling_space(df)
    #plot_metric_scatter(df,alpha_param=0.4,suffix="",show_ellipse=False)
    #print(df.head(5))
    #plt.show()
    #print(df.std_metric_total_energy_metric)

    #####
    df_res_warthog = pd.read_csv("drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric_scatter.csv")
    df_res_husky = pd.read_csv(f"drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric_scatter.csv")
    
    df_res_husky = df_res_husky.loc[df_res_husky.terrain != "tile"]
    df_res = pd.concat([df_res_husky,df_res_warthog],axis=0) 
    only_total_energy = True
    print_column_unique_column(df_res)
    plot_metric_scatter_scatter(df_res,alpha_param=1.0,suffix="",percentile_filtering=True,
                                percentile=95,radius= 0.06,
                                only_total_energy = only_total_energy)#y_column="wheels_metric"), y_column="cmd_diff_icp"
    
    #####
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog = pd.read_csv(path_to_raw_result)
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky = pd.read_csv(path_to_raw_result)

    print(df_husky.terrain.unique())
    
    df = pd.concat([df_warthog,df_husky],axis=0)


    print(df.columns)

    #boxplot(df)
    #print(df.columns)
    #plot_scatter_metric(df)
    #plot_histogramme_metric(df)
    plt.show()



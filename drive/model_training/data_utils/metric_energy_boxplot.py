import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np 
import matplotlib.gridspec as gridspec
from matplotlib.patches import Ellipse
import matplotlib as mpl
import matplotlib.patches as mpatches
import matplotlib as mpl
import sys
import os
import seaborn as sns
import matplotlib.patches as patches
import matplotlib.colors as mcolors 
from drive.model_training.data_utils.slip_boxplot import filter_warthog_command_space_to_husky

project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)


OVERALL = False


def boxplot_all_terrain_all_robot(df,alpha_param=0.2,alpha_bp=0.6,path_to_save="figure/fig_metric_boxplot.pdf"):

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

    fig, axs = plt.subplots(3,1)
    fig.set_figwidth(88/25.4)
    fig.set_figheight(1.5)


    
    fig.subplots_adjust(hspace=0.4,wspace=0.4)
    
    list_array_transl = []
    list_array_rot = []
    list_array_total = []
    list_terrain = []
    list_robot = []
    list_robot_name = []
    
    dico_robot_name = {"husky":"grey","warthog": "orange"}
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
                
    for terrain in list(df.terrain.unique()):
        df_terrain = df.loc[df.terrain==terrain]
        nb_robot = 0
        if terrain == "tile":
            continue
        for robot in df_terrain.robot.unique():

            nb_robot += 1 
            df_robot = df_terrain.loc[df_terrain.robot==robot]
            
            list_array_total.append(df_robot.total_energy_metric.loc[df_robot["terrain"] == terrain])
            list_array_rot.append(df_robot.rotationnal_energy_metric.loc[df_robot["terrain"] == terrain])
            list_array_transl.append(df_robot.translationnal_energy_metric.loc[df_robot["terrain"] == terrain])

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

    box1 = axs[0].boxplot(list_array_rot,showfliers=False,patch_artist=True,positions=list_position,widths=box_width,label=list_robot_name)
    box2 = axs[1].boxplot(list_array_transl,showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box3 = axs[2].boxplot(list_array_total,showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box in [box1,box2,box3]:
        
        for patch, robot_name in zip(box['boxes'],list_robot_name):

            patch.set_facecolor(dico_robot_name[robot_name])  # Change to your desired color
            patch.set_alpha(alpha_bp)
        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')
    list_terrain_x_ticks = [terrain[0].capitalize() + terrain[1:] for terrain in list_terrain]

    axs[0].set_xticks(list_pos_labels,labels=[])
    axs[1].set_xticks(list_pos_labels,labels=[])
    #axs[2].set_xticks(list_pos_labels,labels=list_terrain_x_ticks)

    axs[0].set_ylabel("Difficulty metric \n rotationnal energy [J]")
    axs[1].set_ylabel("Difficulty metric \n translationnal energy [J]")
    axs[2].set_ylabel("Difficulty metric \n total energy [J]")

    # Extract legends from both axes
    legend1 = axs[0].get_legend_handles_labels()
    # Combine legends from both axes
    handles = legend1[0] 
    labels = legend1[1]

    final_handles = [handles[4],handles[5]]
    final_labels = [labels[4][0].capitalize() + labels[4][1:],labels[5][0].capitalize() + labels[5][1:]]
    
    axs[0].legend(handles=final_handles,labels=final_labels)
    #axs[1].set_ylabel("translationnal_energy_metric")
    #axs[2].set_ylabel("total_energy_metric")

    for ax in np.ravel(axs):
        ax.set_xlim(delta_x/2,list_pos_hfill[-1])
        ax.set_ylim(0,1)
    ## Add the color fill 
    j = 1 
    print(list_position)
    print(list_pos_hfill)

    print(list_terrain)
    for terrain  in list_terrain:
        
        axs[0].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_dict[terrain],alpha=alpha_param,label=terrain)
        axs[1].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_dict[terrain],alpha=alpha_param)
        axs[2].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_dict[terrain],alpha=alpha_param)
        
        j+=2
    fig.tight_layout()

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    

    
def reorder_boxplot(list_array, list_color,list_terrain):

    # compute the list of median 
    list_median = []
    for data in list_array:
        #print(data)
        list_median.append(data.median())
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
def boxplot_all_terrain_warthog_robot(df,alpha_param=0.2,robot="warthog", 
                                    alpha_bp=0.4,path_to_save="figure/fig_metric_boxplot.pdf",
                                    linewidth_overall = 5):

    df = df.loc[df.robot == "warthog"]

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
    fig.set_figheight(88/25.4 / 1.618)

    
    # fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    list_array_transl = []
    list_array_rot = []
    list_array_total = []
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
            
            
            list_array_total.append(df_robot.total_energy_metric.loc[df_robot["terrain"] == terrain])
            # list_array_rot.append(df_robot.rotationnal_energy_metric.loc[df_robot["terrain"] == terrain])
            # list_array_transl.append(df_robot.translationnal_energy_metric.loc[df_robot["terrain"] == terrain])

            list_robot_name.append(robot)
        list_robot.append(nb_robot)
        list_terrain.append(terrain)
    list_robot_name.append(robot)
    
    list_color = [color_dict[terrain] for terrain in df.terrain.unique()]

    list_array_total, list_color_total,list_terrain_reordered_total = reorder_boxplot(list_array_total, list_color,list_terrain )
    # list_array_rot, list_color_rot,list_terrain_reordered_rot = reorder_boxplot(list_array_rot, list_color,list_terrain)
    # list_array_transl, list_color_transl,list_terrain_reordered_transl = reorder_boxplot(list_array_transl, list_color,list_terrain)

    for _list in [list_array_total, list_color_total, list_terrain_reordered_total]:
        tmp = _list[0]
        _list[0] = _list[1]
        _list[1] = tmp
    
    # 
    # Add the overall 
    list_array_total.append([item for sublist in list_array_total for item in sublist])
    # list_array_rot.append([item for sublist in list_array_rot for item in sublist])
    # list_array_transl.append([item for sublist in list_array_transl for item in sublist])
    list_terrain.append("Overall")
    # list_color_rot.append("white")
    # list_color_transl.append("white")
    list_color_total.append("white")
    list_terrain_reordered_total.append("Overall")
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
            #pos_labels += (value/2 * delta_same_terrain) 
        else:
            position += delta_x
            pos_labels += delta_x
            list_position.append(position)
            list_pos_labels.append(pos_labels)
            
        pos_hfill = position + delta_x/2
        list_pos_hfill.append(pos_hfill)

    # box1 = axs[0].boxplot(list_array_rot,showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    # box2 = axs[1].boxplot(list_array_transl,showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box1 = axs.boxplot(list_array_total,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box in [box1]:  # ,box2,box3
        
        for patch, color in zip(box['boxes'],list_color_total):

            patch.set_facecolor(color)  # Change to your desired color
            patch.set_alpha(alpha_bp)
        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')
    #list_terrain_x_ticks = [terrain[0].capitalize() + terrain[1:] for terrain in list_terrain]
    #axs[0].set_xticks(list_pos_labels,labels=[])
    #axs[1].set_xticks(list_pos_labels,labels=[])
    #axs[2].set_xticks(list_pos_labels,labels=list_terrain_x_ticks)

    #for ax in np.ravel(axs):
    #    ax.set_xticks([])       # Remove the ticks
    #    ax.set_xticklabels([])  # Remove the labels
        
    # axs[0].set_ylabel("Difficulty metric \n rotationnal energy [J]")
    # axs[1].set_ylabel("Difficulty metric \n translationnal energy [J]")
    for ax in np.ravel(axs):
        ax.set_ylabel("Unpredictability metric")

    # Extract legends from both axes
    #legend1 = axs[0].get_legend_handles_labels()
    # Combine legends from both axes
    #handles = legend1[0] 
    #labels = legend1[1]

    #print(labels)
    #final_handles = [handles[4],handles[5]]
    #final_labels = [labels[4][0].capitalize() + labels[4][1:],labels[5][0].capitalize() + labels[5][1:]]
    
    #axs[0].legend(handles=final_handles,labels=final_labels)
    #axs[1].set_ylabel("translationnal_energy_metric")
    #axs[2].set_ylabel("total_energy_metric")

    for ax in np.ravel(axs):
        ax.set_xlim(delta_x/2,list_pos_hfill[-1])
        ax.set_ylim(0,1.02)
    ## Add the color fill 
    j = 1 
    print(list_position)
    print(list_pos_hfill)

    print(list_terrain)
    # for color_total,label  in zip(list_color_total,list_terrain_reordered_total):
        
    #     # axs[0].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_rot,alpha=alpha_param)
    #     # axs[1].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_transl,alpha=alpha_param)
    #     axs.fill_between(list_pos_hfill[j-1:j+1],y1=1.5,color=color_total,alpha=alpha_param,label=label[0].upper()+label[1:])
        
    #     j+=2
    
    # Add the vertical thick line 
    # axs[0].vlines(list_pos_hfill[-3],ymax=1,ymin=0,color="black",linewidth=linewidth_overall)
    # axs[1].vlines(list_pos_hfill[-3],ymax=1,ymin=0,color="black",linewidth=linewidth_overall)
    axs.vlines(list_pos_hfill[-3],ymax=1.5,ymin=0,color="black",alpha=0.5,linewidth=0.75, linestyles="--")
    #axs[1].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_transl,alpha=alpha_param)
    #axs[2].fill_between(list_pos_hfill[j-1:j+1],y1=1,color=color_total,alpha=alpha_param,label=label[0].upper()+label[1:])
    #    
    # handles, labels = axs.get_legend_handles_labels()

    # overall = mpatches.Patch(edgecolor='black',facecolor="white")
    # handles[-1] = overall
    # fig.legend(handles,labels,bbox_to_anchor= (0.78,0.125),ncols=3)
    #fig.tight_layout()
    tick_labels = ['Gravel', 'Grass', 'Asphalt', 'Sand', 'Ice', 'Overall']
    ticks = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    axs.set_xticks(ticks, tick_labels)
    #fig.tight_layout()
    #fig.subplots_adjust(left=.15, bottom=.16, right=.99, top=.97)
    fig.subplots_adjust(left=.13, bottom=.10, right=.99, top=.97,hspace=0.1)
    
    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    #plt.show()
    
    print(path_to_save)



def boxplot_all_terrain_husky_warthog_robot(df,alpha_param=0.2,robot_list=["warthog"], 
                                    alpha_bp=0.4,path_to_save="figure/fig_metric_boxplot.pdf",
                                    linewidth_overall = 5):

    #df = df.loc[df.robot == "warthog"]

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
    fig.set_figheight(88/25.4 / 1.618)

    
    # fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    
    list_array_transl = []
    list_array_rot = []
    list_array_total = []
    list_terrain = []
    list_robot = []
    list_robot_name = []
    
    dico_robot_name = {"husky":"orange","warthog": "grey","Overall":"blue"}
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"#FFA500",
                "grass":"green","sand":"orangered","avide":"grey",
                "avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral",
                "Overall":"white"}
    
    # Compute and reorder the boxplot 
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
                data = df_robot.total_energy_metric.loc[df_robot["terrain"] == terrain]
                list_array_total.append(data)
                # list_array_rot.append(df_robot.rotationnal_energy_metric.loc[df_robot["terrain"] == terrain])
                # list_array_transl.append(df_robot.translationnal_energy_metric.loc[df_robot["terrain"] == terrain])

                list_robot_name.append(robot)
                list_robot.append(nb_robot)
                list_terrain.append(terrain)
                
                if robot == "husky":
                    linestyle = "-"
                    linecolor= "black"
                elif robot == "warthog":
                    linestyle = "-"
                    linecolor = "black"
                dico_data[f"{terrain}_{robot}"] = {"color":color_dict[terrain], "robot":robot,
                                                "ls":linestyle,  "data":data,"linecolor":linecolor,"terrain":terrain}
    
    if OVERALL:
        dico_data[f"overall_husky"] = {"color":"white", "robot":"husky", "data":df.total_energy_metric.loc[df.robot=="husky"],"ls":"-" ,"linecolor":"black","terrain":"overall"}
        dico_data[f"overall_warthog"] = {"color":"white", "robot":"warthog", "data":df.total_energy_metric.loc[df.robot=="warthog"],"ls":"-","linecolor":"black","terrain":"overall"}
    
    # order_to_present = ["gravel_warthog", "grass_warthog","grass_husky", "asphalt_warthog",
                        # "asphalt_husky","mud_husky","sand_warthog","ice_warthog", "overall_husky",
                        # "overall_warthog"]
    # list_position = np.array([big_delta,2*big_delta,2*big_delta+small_delta, 3* big_delta+small_delta, 3* big_delta+ 2 * small_delta,
                            #   4* big_delta+ 2 * small_delta,  
                            # 5* big_delta+ 2 * small_delta, 6* big_delta+ 2 * small_delta ,
                            # 7* big_delta+ 2 * small_delta,7* big_delta+ 3 * small_delta ]) - (big_delta + delta_start)
    # 
    if OVERALL:
        order_to_present = [ "asphalt_husky","grass_husky", "overall_husky", "mud_husky",
                            "grass_warthog", "gravel_warthog","asphalt_warthog","overall_warthog",
                            "sand_warthog", "ice_warthog"]
    else:
        order_to_present = [ "asphalt_husky","grass_husky", "mud_husky",
                            "grass_warthog", "gravel_warthog","asphalt_warthog",
                            "sand_warthog", "ice_warthog"]

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

    list_array_total = [dico_data[value]["data"] for value in order_to_present]
    list_color_total = [dico_data[value]["color"] for value in order_to_present]
    list_patch_linestyle = [dico_data[value]["ls"] for value in order_to_present]
    list_colorline = [dico_data[value]["linecolor"] for value in order_to_present]

    # box1 = axs[0].boxplot(list_array_rot,showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    # box2 = axs[1].boxplot(list_array_transl,showfliers=False,patch_artist=True,positions=list_position,widths=box_width)
    box1 = axs.boxplot(list_array_total,whis=(2.5, 97.5),showfliers=False,patch_artist=True, positions=list_position,widths=box_width)

    for box in [box1]:  # ,box2,box3
        
        for patch, color,linestyle in zip(box['boxes'],list_color_total,list_patch_linestyle):
            rgb = mcolors.to_rgb(color)
            white =  (1.0,1.0,1.0)
            blended_rgb = tuple(
                alpha_bp * c + (1 - alpha_bp) * w
                for c, w in zip(rgb, white)
            )
            patch.set_facecolor(blended_rgb)  # Change to your desired color
            #patch.set_alpha(alpha_bp)  # Change to your desired color
            #patch.set_alpha(alpha_bp)
            patch.set_linestyle(linestyle)
        # Change the median line color to black
        for median in box['medians']:
            median.set_color('black')
        i = 0
        for linestyle,linecolor in zip(list_patch_linestyle,list_colorline):
            
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
    tick_labels = ["Husky","Warthog"]
    ticks = np.array([2.5*small_delta,7.5*small_delta])
    ticks[1:] += big_delta
    ticks[2:] += big_delta
    #tick_labels = ['Gravel', 'Grass', 'Asphalt',"Mud", 'Sand', 'Ice', "Overall"]
    #ticks = np.array([big_delta, 2*big_delta+small_delta/2,  3*big_delta+3*small_delta/2,  
    #                4* big_delta+ 2 * small_delta,5* big_delta+ 2 * small_delta,  6* big_delta+ 2 * small_delta,
    #                7* big_delta+ 5/2 * small_delta ]) - (big_delta + delta_start)
    axs.set_xticks(ticks, tick_labels)
    
    for ax in np.ravel(axs):
        ax.set_ylabel("Unpredictability metric")
        ax.set_xlim(min(list_position)-small_delta, max(list_position)+small_delta)
    
    #axs.vlines(pos_vlines,ymax=1.1,ymin=0,color="black",alpha=0.5,linewidth=0.75, linestyles="-.")
    axs.set_ylim(0,1.1)

    for ax in np.ravel(axs):
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


    #fig.tight_layout()
    #fig.subplots_adjust(left=.15, bottom=.16, right=.99, top=.97)

    fig.subplots_adjust(left=.13, bottom=.275, right=.99, top=.97,hspace=0.1)
    
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
    
    print(legend_handles)
    if OVERALL:
        axs.legend(handles=legend_handles,loc='upper center',
            bbox_to_anchor=(0.5, -0.1125),#0.42, -0.1),  # x=center, y=slightly below
            ncol=4, fontsize=8,
            handletextpad=1.0,     # space between handle and text
            columnspacing=1.0,
            )      # space between columns)
    else:
        axs.legend(handles=legend_handles,loc='upper center',
            bbox_to_anchor=(0.5, -0.1125),#0.42, -0.1),  # x=center, y=slightly below
            ncol=3, fontsize=8,
            handletextpad=1.0,     # space between handle and text
            columnspacing=1.0,
            )      # space between columns)

    fig.savefig(path_to_save,dpi=300)
    fig.savefig(path_to_save[:-4]+".png",dpi=300)
    #plt.show()
    
    print(path_to_save)

def metric_husky(df):

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

    fig, ax1 = plt.subplots(1,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.0)
    fig.subplots_adjust(left=.13, bottom=.08, right=.99, top=.97,hspace=0.1)
    #fig.subplots_adjust(hspace=0.2 ,wspace=0.4)
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"orangered","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
    
    for terrain in list(df.terrain.unique()):

        df_terrain = df.loc[df.terrain==terrain]

        ax1.scatter(df_terrain.cmd_body_yaw_vel, df_terrain.cmd_body_lin_vel,c=color_dict[terrain],label = terrain)
    ax1.legend()
    list_terrain = list(df.terrain.unique())

    #df.plot.box(column="total_energy_metric",by="terrain")
    #df.plot.box(column="translationnal_energy_metric",by="terrain")
    fig, axs = plt.subplots(5,1)
    
    df["sum_energy_vs_total_diff_percentage"] = (-df.total_energy_metric + (df.translationnal_energy_metric + df.rotationnal_energy_metric))

    df["diff_energy_rotation"] = df.cmd_metric_total_energy_metric_rotationnal_j_components - df.cmd_metric_idd_rotationnal_j 
    df["diff_energy_translation"] = df.cmd_metric_total_energy_metric_translationnal_j_components - df.cmd_metric_idd_translationnal_j 
    sns.violinplot(ax= axs[0],x='terrain', y="cmd_metric_total_energy_metric_translationnal_j_components", data=df)
    sns.violinplot(ax= axs[1],x='terrain', y="cmd_metric_total_energy_metric_translationnal_weights", data=df)
    sns.violinplot(ax= axs[2],x='terrain', y="cmd_metric_total_energy_metric_rotationnal_j_components", data=df)
    sns.violinplot(ax= axs[3],x='terrain', y="cmd_metric_total_energy_metric_rotationnal_weights", data=df)
    sns.violinplot(ax= axs[4],x='terrain', y="total_energy_metric", data=df)

    fig, axs2 = plt.subplots(2,1)
    
    fig.set_figwidth(88/25.4) 
    fig.set_figheight(4.0)
    fig.subplots_adjust(left=.13, bottom=.08, right=.99, top=.97,hspace=0.1)

    #  'cmd_metric_idd_rotationnal_j',
    #    'cmd_metric_idd_translationnal_j', 'cmd_metric_idd_total_j', 'terrain',
    #df.plot.box(column="sum_energy_vs_total_diff_percentage",by="terrain",showfliers=False)
    sns.violinplot(ax= axs2[0],x='terrain', y="diff_energy_rotation", data=df)
    sns.violinplot(ax= axs2[1],x='terrain', y="diff_energy_translation", data=df)

    #plt.show()

def print_color_list():
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"orangered","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
    for key in color_dict:
        print("=========================================")
        print("Terrain: ", key)
        print("Matplotlib named color: ", color_dict[key])
        print("Hex color code: ", mpl.colors.to_hex(color_dict[key], keep_alpha=False))


def keep_only_steady_state_and_filter(df,size_col,nb_steady_state,yaw_filter =4.0,
                                    keep_only_steady_state = True,
                                    filter_data = True,col_to_filter_with="cmd_body_yaw_vel"):
    
    nb_values = df.shape[0]
    # Create the filter 
    cmd_yaw = df[col_to_filter_with].to_numpy().reshape((nb_values//size_col, size_col))

    cmd_yaw = np.median(cmd_yaw,axis=1)
    
    cmd_yaw_filter = np.array([cmd_yaw]*size_col).T
    filter = cmd_yaw_filter <=yaw_filter
    
    dico = {}
    print(df.shape)
    for col in list(df.columns):

        

        

        masked = np.zeros((nb_values,1))
        
        masked_reshape = masked.reshape((nb_values//size_col, size_col))

        
        masked_reshape[:,-nb_steady_state:] = np.ones((masked_reshape.shape[0],nb_steady_state))

        data_matrix = df[col].to_numpy().reshape((nb_values//size_col, size_col))
        
        if keep_only_steady_state:
            mask = (masked_reshape ==1)

            if filter_data:

                mask = mask&filter
            
        else:
            
            mask = np.ones_like(masked_reshape) == 1.0

            if filter_data:
                mask = filter

        final_df = data_matrix[mask]

        dico[col] = np.ravel(final_df)

    print(pd.DataFrame.from_dict(dico).shape)
    return pd.DataFrame.from_dict(dico)




if __name__ =="__main__":
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe_copy_backup/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog = pd.read_csv(path_to_raw_result)
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe_copy_backup/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky = pd.read_csv(path_to_raw_result)
    #df_husky = df_husky.drop()
    
    husky_geom_path = "drive_datasets/results_multiple_terrain_dataframe_copy_backup/husky_geom_limits_by_terrain_for_filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    #filtered_df = df_warthog[(np.abs(df_warthog["cmd_body_yaw_vel"]) < 4.0)]

    #filtered_df = keep_only_steady_state_and_filter(df_warthog,119,39)

    filtered_df_warthog = keep_only_steady_state_and_filter(df_warthog,119,39,yaw_filter =4.0,
                                    keep_only_steady_state = True,
                                    filter_data = True)
    print("df husky  shape :", df_husky.shape)
    filtered_df_husky = keep_only_steady_state_and_filter(df_husky,119,39,yaw_filter =4.0,
                                    keep_only_steady_state = True,
                                    filter_data = True)
    filtered_df_husky = filtered_df_husky[(filtered_df_husky.terrain == "grass") | (filtered_df_husky.terrain=="asphalt") | (filtered_df_husky.terrain=="mud")]
    

    df_concat = pd.concat([filtered_df_warthog,filtered_df_husky])
    #boxplot(df)
    boxplot_all_terrain_husky_warthog_robot(df_concat,robot_list=["husky","warthog"])
    
    #df_warthog_in_frame_husky = filter_warthog_command_space_to_husky(filtered_df_warthog,cols=["cmd_body_yaw_vel","cmd_body_lin_vel"])

    #df_concat_frame_husky = pd.concat([df_warthog_in_frame_husky,filtered_df_husky])
    #boxplot_all_terrain_husky_warthog_robot(df_concat_frame_husky,robot_list=["husky","warthog"])
    
    #print(df.columns)
    #plot_scatter_metric(df)
    #plot_histogramme_metric(df)
    plt.show()

    #print_color_list()

    # print(0.40732918650830996)
    # print(0.75 / 0.40732918650830996)
    print("test")
    #metric_husky(df_husky)
    plt.show()
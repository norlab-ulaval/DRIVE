import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np 




import matplotlib.gridspec as gridspec

from matplotlib.patches import Ellipse
import matplotlib as mpl
from matplotlib.lines import Line2D


def plot_metric_scatter(df_res,alpha_param=0.4,suffix="",show_ellipse=False):

    df_all = df_res.copy()
    #df.reset_index(inplace=True, names="terrain")

    # col = ["slope","slope_std","x_95","y_std_95"]

    fig, axs = plt.subplots(3,1)
    fig.set_figwidth(6)
    fig.set_figheight(6)
    fig.subplots_adjust(hspace=0.4,wspace=0.4)

    color_dict = {"asphalt":"pink", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod"}
    columns = ["rotationnal_energy_metric","translationnal_energy_metric","total_energy_metric" ]
    
    
    # Scatter plot the data points 
    list_marker = [
    'o',  # Circle
    's',  # Square
    '^',  # Triangle Up
    'v',  # Triangle Down
    '>',  # Triangle Right
    '<',  # Triangle Left
    'p',  # Pentagon
    '*',  # Star
    'H',  # Hexagon (regular)
    'h',  # Hexagon (alternate)
    '+',  # Plus
    'x',  # Cross
    'D',  # Diamond
    'd',  # Thin Diamond
    '|',  # Vertical Line
    '_',  # Horizontal Line
    ]
    i_marker = 0

    first_time_terrain = True
    for lim_yaw in df_all["lim_vel_yaw"].unique(): 
        
        df_lim_vel_yaw = df_all.loc[df_all.lim_vel_yaw == lim_yaw]
        
        for lim_vel_x in df_lim_vel_yaw["lim_vel_x"].unique(): 
            df = df_lim_vel_yaw.loc[df_lim_vel_yaw.lim_vel_x == lim_vel_x]
            
            first_time_symbol = True
            
            for column,ax in zip(columns,np.ravel(axs)):
                x = df["metric_"+column].to_numpy()
                y = df["cmd_95_"+column].to_numpy() * df["mean_slope_"+column].to_numpy() 
                color_list = [color_dict[terrain] for terrain in df.terrain]
                
                
                if first_time_terrain:
                    for i in range(len(x)):
                        ax.scatter(x[i], y[i], color=color_list[i], marker=list_marker[i_marker],label=df.terrain.loc[i],edgecolors='none')
                    first_time_terrain = False
                else:
                    ax.scatter(x, y, color=color_list, marker=list_marker[i_marker],edgecolors='none')
                
                if first_time_symbol:
                    ax.scatter(0.7, 0, color="black", marker=list_marker[i_marker], label=f"lim vel_x {lim_vel_x} \n lim vel_yaw {lim_yaw}")
                    first_time_symbol = False
                    
                # Plot uncertainty ellipses around each data point
                if show_ellipse:
                    for i in range(len(x)):

                        
                        # Covariance matrix for the uncertainty (example: variance in x and y)
                        y_std_95 = df.cmd_95_total_energy_metric.to_numpy() * df.std_slope_total_energy_metric.to_numpy() 

                        cov = np.array([[df.std_metric_total_energy_metric.loc[i]**2, 0], [0, y_std_95[i]**2]])  # Example covariance matrix, adjust as needed

                        # Create an ellipse
                        eigvals, eigvecs = np.linalg.eigh(cov)  # Eigenvalues (size of axes) and eigenvectors (orientation)
                        angle = np.arctan2(*eigvecs[:, 0][::-1])  # Angle of rotation for the ellipse
                        width, height = 1 * np.sqrt(eigvals)  # Width and height (2 * std dev, i.e., 95% confidence ellipse)
                        
                        # Create an Ellipse object
                        
                        
                        ellipse = Ellipse((x[i], y[i]), width, height, angle=np.degrees(angle), 
                                        edgecolor=color_list[i], facecolor=color_list[i], linestyle='-',
                                        label= df.terrain[i]+" 1 std",alpha = alpha_param)
                        
                        # Add the ellipse to the plot
                        ax.add_patch(ellipse)

            


                # Add labels and legend
                ax.set_xlabel("Difficulty metric (1-slope)")
                ax.set_ylabel(f"{column} @(x=95 \%) ")
                #ax.set_title('New mapping of DRIVE (mass include)'+suffix)
                
                ax.grid(True)
                ylimit = ax.get_ylim()
                ax.set_ylim(0,ylimit[1])
                ax.set_xlim(0,1)

            i_marker += 1

    axs[0].legend(ncol=5)
    #plt.tight_layout()

def plot_metric_depending_on_sampling_space(df_res,suffix=""):

    df = df_res.copy()
    #df.reset_index(inplace=True, names="terrain")

    # col = ["slope","slope_std","x_95","y_std_95"]

    fig, axs = plt.subplots(3,2)
    fig.set_figwidth(6)
    fig.set_figheight(6)
    fig.subplots_adjust(hspace=0.4,wspace=0.4)

    color_dict = {"asphalt":"pink", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod"}
    columns = ["rotationnal_energy_metric","translationnal_energy_metric","total_energy_metric" ]
    
    
    # Scatter plot the data points 
    list_marker = [
    'o',  # Circle
    's',  # Square
    '^',  # Triangle Up
    'v',  # Triangle Down
    '>',  # Triangle Right
    '<',  # Triangle Left
    'p',  # Pentagon
    '*',  # Star
    'H',  # Hexagon (regular)
    'h',  # Hexagon (alternate)
    '+',  # Plus
    'x',  # Cross
    'D',  # Diamond
    'd',  # Thin Diamond
    '|',  # Vertical Line
    '_',  # Horizontal Line
    ]
    i_marker = 0

    first_time_terrain = True
    list_x_axis = ["lim_vel_x","lim_vel_yaw"]
    col_index = [0,1]
    for x_axis, col_i in zip(list_x_axis,col_index):
        
        axs_i = axs[:,col_i]
        for column,ax in zip(columns,np.ravel(axs_i)):
            y = df["metric_"+column].to_numpy()
            x = df[x_axis].to_numpy()
            color_list = [color_dict[terrain] for terrain in df.terrain]
            
            
            if first_time_terrain:
                for i in range(len(x)):
                    ax.scatter(x[i], y[i], color=color_list[i], marker=list_marker[i_marker],label=df.terrain.loc[i])
                first_time_terrain = False
            else:
                ax.scatter(x, y, color=color_list, marker=list_marker[i_marker])
            
            
            


            # Add labels and legend
            ax.set_ylabel(f"Metric based \n {column}")
            
            ax.set_title('New mapping of DRIVE (mass include)'+suffix)
            
            ax.grid(True)
            ylimit = ax.get_ylim()
            ax.set_ylim(0,ylimit[1])
            ax.set_xlim(0,5.0)
            axs[0,0].legend(ncol=5)
    axs[-1,0].set_xlabel(f"lim_vel_x")
    axs[-1,1].set_xlabel(f"lim_vel_yaw")
        
    
    

def plot_metric_3d(df_res,suffix=""):

    df_all = df_res.copy()

    
    #df.reset_index(inplace=True, names="terrain")

    # col = ["slope","slope_std","x_95","y_std_95"]
    list_terrain = list(df_all.terrain.unique())

    fig, axs = plt.subplots(3,len(list_terrain))
    fig.set_figwidth(6)
    fig.set_figheight(6)
    fig.subplots_adjust(hspace=0.4,wspace=0.4)

    color_dict = {"asphalt":"pink", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod"}
    columns = ["rotationnal_energy_metric","translationnal_energy_metric","total_energy_metric" ]
    
    list_vmin_vmax = [[df_all.metric_rotationnal_energy_metric.min(),df_all.metric_rotationnal_energy_metric.max()],
                      [df_all.metric_translationnal_energy_metric.min(),df_all.metric_translationnal_energy_metric.max()],
                      [df_all.metric_total_energy_metric.min(),df_all.metric_total_energy_metric.max()]]
    for j,terrain in enumerate(list_terrain):
        axs_j = axs[:,j]
        df = df_all.loc[df_all.terrain==terrain]
        axs_j[0].set_title(terrain)
        shape = df.lim_vel_yaw.value_counts().loc[df.lim_vel_yaw.unique()[0]]

        for col, ax,vminmax in zip(columns,np.ravel(axs_j),list_vmin_vmax):
            
            metric_value = df["metric_"+col].to_numpy().reshape(shape,shape)
            #metric_value = df["lim_vel_yaw"].to_numpy().reshape(shape,shape)
            
            pictur = ax.imshow(metric_value, cmap='viridis', interpolation='nearest',extent = [0.5, 5, 5, 0.5],
                               )
            ax.invert_yaxis()
            
            ax.set_xlabel("lim_x")
            ax.set_ylabel("lim_yaw")
            # Add color bar to indicate the values
            fig.colorbar(pictur,label=f'metric {col}',ax=ax)

            # Add titles and labels (optional)
            #plt.title('Heatmap Example')
            #plt.xlabel('X Axis')
            #plt.ylabel('Y Axis')

            # Show the plot
    
    plt.show()
        
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
                                path_to_save = "figure/fig_metric.pdf",
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
    
    
    if only_total_energy:
        fig.set_figwidth(88/25.4)
        fig.set_figheight(3.0)
        list_y_coordinates = ["_total_energy_metric"]
        labels = ["$95^{th}$ percentile \n kinetic energy (J)"]
        # Create subplots with gridspec
        gs = gridspec.GridSpec(2, 1, wspace=0.4,height_ratios=[4,3], figure=fig)
        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])#,sharex=ax1)
        n_size= 2
        axs = [ax1,ax2]
    else:
        fig.set_figwidth(88/25.4)
        fig.set_figheight(4.4)
        list_y_coordinates = ["_rotationnal_energy_metric","_translationnal_energy_metric","_total_energy_metric"]
        labels = ["Rotationnal \n kinetic energy [J]","Translattional \n kinetic energy [J]","Total \n kinetic energy [J]"]

        gs = gridspec.GridSpec(4, 1, wspace=0.4,height_ratios=[4,4,4,4], figure=fig)

        # Create subplots with gridspec
        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])#,sharex=ax1)
        ax3 = fig.add_subplot(gs[2])#,sharex=ax1)
        axs = [ax1,ax2,ax3]
        n_size= 3
    for metric_name,ax,ylabel in zip(list_y_coordinates,axs,labels):
        
        
        for robot in df.robot.unique():
            linestyle_dict = {"husky":"--","warthog":"-"}
            linestyle = linestyle_dict[robot]
            first_time_robot=True
            df_robot = df.loc[df.robot==robot]

            
            for terrain in np.unique(df_robot.terrain):
                df_terrain = df_robot.loc[df_robot.terrain==terrain]
                x = df_terrain["cmd_metric"+metric_name].to_numpy()
                y = df_terrain[y_column+metric_name].to_numpy()

                #y_95 = np.percentile(y,95)
                #mask = y <= y_95
                #x_masked = x[mask]
                #y_masked = y[mask]
            
                x_masked = x
                y_masked = y
                color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
                
                
                if percentile_filtering:
                    x_masked,y_masked,y_windows_std = moving_average(x,y,percentile=percentile,r = radius)
                    color_list = [color_dict[terrain]]*x_masked.shape[0]
                    # Scatter plot the data points 
                    if percentile==50:
                        ax.errorbar(x_masked, y_masked,yerr=y_windows_std, color=color_list[0],alpha=0.9,label=terrain)    
                    else:
                        test=1
                    
                        ax.scatter(x_masked, y_masked, color=color_list,alpha=0.9,s=0.8,edgecolors='none')
                        #ax.plot(x_masked, y_masked, color=color_list[0],alpha=alpha_param,label=terrain,ls=linestyle)
                    
                    
                    
                else:
                    color_list = [color_dict[terrain]]*x_masked.shape[0]
                    ax.scatter(x_masked, y_masked, color=color_list,alpha=0.2,s=6,label=terrain,edgecolors='none')
                
            x = df_robot["cmd_metric"+metric_name].to_numpy()
            y = df_robot[y_column+metric_name].to_numpy()
            x_masked,y_masked,y_windows_std = moving_average(x,y,percentile=percentile,r = radius)
            label_robot = f"{robot[0].capitalize()}{robot[1:]}"
            #label_robot[0].capitalize() 
            ax.plot(x_masked, y_masked, color="black",alpha=1,label=label_robot,ls=linestyle)
            
            
        ax.set_ylabel(f"{ylabel}")
                
    # Add labels and legend
    
    
    
    #axs[0].set_title('New mapping of DRIVE (mass include)'+suffix)
    
    for ax in np.ravel(axs):
        #ax.legend()
        ax.grid(True)
        ylimit = ax.get_ylim()
        ax.set_ylim(1,ylimit[1])
        ax.set_xlim(0,1)
        #ax.set_yscale("log")

    #axs.reshape(3,1)

        # Extract legends from both axes
    legend1 = axs[0].get_legend_handles_labels()

    # Combine legends from both axes
    #handles = legend1[0] 
    #labels = legend1[1]
    #
    #husky_labels = labels[:4]
    #husky_handles = handles[:4]
    #warthog_labels = labels[5:]
    #warthog_handles = handles[5:]
    #robot_label = [warthog_labels[-1][0].upper()+ warthog_labels[-1][1:], husky_labels[-1][0].upper()+husky_labels[-1][1:]]
    #robot_handles = [warthog_handles[-1], husky_handles[-1]]
    #
#
    #final_labels = []
    #final_handles = []
#
    #
    #terrain_label = labels[:3] + labels[4:-1]
    #terrain_handle = handles[:3] + handles[4:-1]
#
    #filtered_label = []
    #filtered_handle = []
    ###
    #for label,handle  in zip(terrain_label,terrain_handle):
    #    if (label[0].upper()+label[1:]) in filtered_label:
    #        continue
    #    else:
    #        print(label)
    #        filtered_label.append(label[0].upper()+label[1:] )
    #        handle.set_linestyle("-")
    #        filtered_handle.append(handle)
    #
    #filtered_handle.append(handles[-1])
    #filtered_label.append("All \n terrain")
#
    #    
    #
#
    #y_position_l1 = 0.37
    #x_pos_l1 = 0.65
    ## Create a legend in the figure (outside the axes)
    #for i,label in enumerate(final_labels):
    #    final_labels[i] = label[0].capitalize() + label[1:]
    #
    #legend_terrain  = fig.legend(filtered_handle, filtered_label, 
    #        ncols=2,bbox_to_anchor = (x_pos_l1,y_position_l1),
    #        columnspacing=0.4,title=r"$\mathbf{Terrain}$",
    #        labelspacing=0.1,
    #        handletextpad=0.3)
    #
    # Get the bounding box of the legend
    #bbox = legend_terrain.get_window_extent()
    #legend_height = bbox.height / dpi  # Height in inches
    #legend_width =  bbox.width / dpi 
    #
    #print(x_pos_l1,legend_width)
    #fig.legend(robot_handles, robot_label, 
    #        bbox_to_anchor = (x_pos_l1+legend_width/2+0.02 ,y_position_l1),
    #        ncols=1,
    #        columnspacing=0.4,title=r"$\mathbf{Robot}$",
    #        labelspacing=0.1,
    #        handletextpad=0.3)
    
    
    #fig.legend(final_handles, final_labels, 
    #        loc='lower right',ncols=3,
    #        columnspacing=0.3,title=r"$\textbf{P95 of Energy}$",
    #        labelspacing=0.1)
    # Reput the husky line in dashe
    #for handle in husky_handles:
    #    handle.set_linestyle("--")
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
    
    axs[n_size-2].set_xlabel("Difficulty metric [SI]")

    axs[0].hlines(6250,0,1)
    axs[0].hlines(500*4**2/2,0,1)
    axs[0].hlines(500*0.1**2/2,0,1,color="red")
    fig.savefig(fname=path_to_save,dpi=dpi)
    #axs[2].set_xticklabels([])
    print(gs)
    
def plot_scatter_with_colormap(x, y, z, ax,cmap='viridis',zlabel=""):
    """
    Plot a scatter plot of x, y with colors based on z values.
    
    Parameters:
    - x: array-like, x-coordinates of the points.
    - y: array-like, y-coordinates of the points.
    - z: array-like, z-values that will determine the color of each point.
    - cmap: string, optional, colormap to use for coloring (default is 'viridis').
    
    Returns:
    - A scatter plot with a color map based on z-values.
    """
    
    # Create a scatter plot
    scatter = ax.scatter(x, y, c=z, cmap=cmap, edgecolors='k', alpha=0.7)
    
    # Add a colorbar to the plot
    plt.colorbar(scatter, label=zlabel)
    
    # Label the axes
    
    # Title of the plot
    #plt.title('Scatter Plot with Z as Color Map')
    
    # Show the plot
    #plt.show()

def plot_scatter_metric(df):
    df_ice = df.loc[df.terrain=="ice"]
    
    fig,axs = plt.subplots(3,3)
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.rotationnal_energy_metric, axs[0,0],cmap='viridis',zlabel="rotationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.translationnal_energy_metric, axs[1,0],cmap='viridis',zlabel="translationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.total_energy_metric,axs[2,0], cmap='viridis',zlabel="total")
    for ax in axs[:,0]:
        ax.set_facecolor("lightcyan")
        
        ax.set_ylabel('Cmd x vel')
    
    df_ice = df.loc[df.terrain=="asphalt"]
    
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.rotationnal_energy_metric, axs[0,1],cmap='viridis',zlabel="rotationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.translationnal_energy_metric, axs[1,1],cmap='viridis',zlabel="translationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.total_energy_metric,axs[2,1], cmap='viridis',zlabel="total")
    
    for ax in axs[:,1]:
        ax.set_facecolor("whitesmoke")

    df_ice = df.loc[df.terrain=="sand"]
    
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.rotationnal_energy_metric, axs[0,2],cmap='viridis',zlabel="rotationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.translationnal_energy_metric, axs[1,2],cmap='viridis',zlabel="translationnal")
    plot_scatter_with_colormap(df_ice.cmd_body_yaw_vel, df_ice.cmd_body_lin_vel, df_ice.total_energy_metric,axs[2,2], cmap='viridis',zlabel="total")
    
    for ax in axs[:,2]:
        ax.set_facecolor("lightyellow")

    for ax in axs[1,:]:
        ax.set_xlabel('Cmd x vel')
def plot_hist(metric_raw,ax,ylabel=""):

    ax.hist(metric_raw,range=(0,1),bins=60,density=True)
    y_lim = ax.get_ylim()
    ax.set_ylabel(ylabel)
    ax.vlines(np.median(metric_raw),ymin=y_lim[0],ymax=y_lim[1],label="median", color="red")
    ax.vlines(np.mean(metric_raw),ymin=y_lim[0],ymax=y_lim[1],label="mean", color="green" )
    ax.legend()
    
def plot_histogramme_metric(df):
    
    df_ice = df.loc[df.terrain=="ice"]
    
    fig,axs = plt.subplots(3,3)
    plot_hist( df_ice.rotationnal_energy_metric, axs[0,0],ylabel="rotationnal")
    plot_hist( df_ice.translationnal_energy_metric, axs[1,0],ylabel="translationnal")
    plot_hist( df_ice.total_energy_metric,axs[2,0], ylabel="total")
    for ax in axs[:,0]:
        ax.set_facecolor("lightcyan")

    df_ice = df.loc[df.terrain=="asphalt"]
    
    plot_hist( df_ice.rotationnal_energy_metric, axs[0,1],ylabel="rotationnal")
    plot_hist( df_ice.translationnal_energy_metric, axs[1,1],ylabel="translationnal")
    plot_hist( df_ice.total_energy_metric,axs[2,1],ylabel="total")
    
    for ax in axs[:,1]:
        ax.set_facecolor("whitesmoke")

    df_ice = df.loc[df.terrain=="sand"]
    
    plot_hist( df_ice.rotationnal_energy_metric, axs[0,2],ylabel="rotationnal")
    plot_hist( df_ice.translationnal_energy_metric, axs[1,2],ylabel="translationnal")
    plot_hist( df_ice.total_energy_metric,axs[2,2],ylabel="total")
    
    for ax in axs[:,2]:
        ax.set_facecolor("lightyellow")

def boxplot(df):

    list_array_transl = []
    list_array_rot = []
    list_array_total = []
    list_terrain = []

    for terrain in list(df.terrain.unique()):
        list_terrain.append(terrain) 
        list_array_total.append(df.total_energy_metric.loc[df["terrain"] == terrain])
        list_array_rot.append(df.rotationnal_energy_metric.loc[df["terrain"] == terrain])
        list_array_transl.append(df.translationnal_energy_metric.loc[df["terrain"] == terrain])
        
    fig, axs = plt.subplots(3,1)
    
    axs[0].boxplot(list_array_rot,showfliers=False,tick_labels=list_terrain)
    axs[0].set_ylabel("rotationnal_energy_metric")
    axs[1].boxplot(list_array_transl,showfliers=False,tick_labels=list_terrain)
    axs[1].set_ylabel("translationnal_energy_metric")
    axs[2].boxplot(list_array_total,showfliers=False,tick_labels=list_terrain)
    axs[2].set_ylabel("total_energy_metric")


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
    
    
    df_res_warthog = df_res_warthog.loc[(df_res_warthog.terrain == "ice")| (df_res_warthog.terrain == "grass")]
    df_res_husky = df_res_husky.loc[df_res_husky.terrain == "mud"]

    df_res_husky = df_res_husky.loc[df_res_husky.terrain != "tile"]
    #df_res_warthog = df_res_warthog.loc[df_res_warthog.terrain == "ice"]
    df_res = df_res_warthog#pd.concat([df_res_husky,df_res_warthog],axis=0) 
    only_total_energy = True
    print(df_res_husky.terrain.unique())
    plot_metric_scatter_scatter(df_res,alpha_param=1.0,suffix="",percentile_filtering=False,
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

    

import pandas as pd
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import pickle

TERRAIN_TO_PLOT = ["asphalt","ice"]

ROBOT_PARAMS_PER_TERRAIN = {"asphalt":  [1.08, 0.3, 4, 5, 16.6667],
                            "ice":      [1.08, 0.3, 4, 5, 16.6667],
                            "gravel":   [1.08, 0.3, 4, 5, 16.6667],
                            "grass":    [1.08, 0.3, 4, 5, 16.6667],
                            "sand":     [1.08, 0.3, 4, 5, 16.6667]}

COLOR_DICT = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew","tile":"mistyrose",
              "boreal":"lightgray","sand":"lemonchiffon","avide":"white","avide2":"white","wetgrass":"honeydew"}

PATH_DATAFRAME = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
PATH_DATAFRAME_DIAMOND = "drive_datasets/results_multiple_terrain_dataframe/warthog_geom_limits_by_terrain_for_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
PATH_ROBOT_PARAM = "robot_param.yaml"

COLOR_BODY_FRAME = "green"
COLOR_WHEEL_FRAME = "blue"
SCATTER_PLOT_TO_GENERATE = {"Body_vel": True, "Wheels_vel": True}
HIST_PLOT_TO_GENERATE = {"Accel_x": False, "Accel_y": False, "Accel_yaw": False, "Accel_yaw_imu": False, "Slip_body_y": True}

PLOT_VERTICALLY = False
MARKER_SIZE = 5

global ONLY_CMD 
ONLY_CMD = False

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 8}
plt.rc('font', **font)
plot_fs = 12
plt.rc('font', family='serif', serif='Times')
plt.rc('text', usetex=True)
plt.rc('xtick', labelsize=8)
plt.rc('ytick', labelsize=8)
plt.rc('axes', labelsize=8)
mpl.rcParams['lines.dashed_pattern'] = [2, 2]
mpl.rcParams['lines.linewidth'] = 1.0 

def extracts_appropriate_columns(df,commun_name):
    df_columns = list(df.columns)
    possible_number = ["1","2","3","4","5","6","7","8","9","0"]

    for i,column in enumerate(df_columns):
        if column[-1] in possible_number:
            if column[-2] in possible_number:
                df_columns[i] = column[:-3]
                if column[-3] in possible_number:
                    df_columns[i] = column[:-4]
            else:
                df_columns[i] = column[:-2]

    df_columns_name = pd.Series(df_columns)
    mask = [name == commun_name for name in df_columns_name ]
    kept_column = df.columns[mask]

    return df[kept_column]


def column_type_extractor(df, common_type,
                        transient_state=True,steady_state=True, verbose=False):
    """ Extract the np.matrix that represent the 40 columns of all steps of the 
    specific type. 
    column_type_extractor
    For example, icp_velx_40 is of the type icp_velx 
    
    """
    local_df = df.copy()
    if transient_state==False and steady_state==True:
        mask = local_df.steady_state_mask == 1
        local_df = local_df.loc[mask]
    elif steady_state == False and transient_state == True:
        mask = local_df.steady_state_mask == 0
        local_df = local_df.loc[mask]
    elif steady_state == False and transient_state == False:
        raise ValueError("Both steady state and transient can not be at false")
    
    local_df = extracts_appropriate_columns(pd.DataFrame(local_df),common_type)
    np_results = local_df.to_numpy().astype('float')

    if verbose == True:
        message = "_"*8+f"{common_type}"+"_"*8
        print(message)
        print(f"The column type: {common_type}")
        print(f"The resulting dataframe_shape: {np_results.shape}")
        print(f"Number of calibrating steps:{np_results.shape[0]}")
        print(f"Number of measurement by step: {np_results.shape[1]}")
        print(f"Maximum {np.max(np_results)}")
        print(f"Minimum {np.min(np_results)}")
        print("_"*len(message))
    return np_results

def plot_losange_limits(ax,geom):
    # If ax is an array of ax then plot the losange on each ax
    if isinstance(ax,np.ndarray):
        for ax_ in ax:
            plot_losange_limits(ax_,geom)
    else:
        x,y = geom.exterior.xy
        ax.plot(x,y, color="black")


def add_vehicle_limits_to_wheel_speed_graph(ax,first_time =False,robot=[1.08,0.3,4,5,16.6667], geom_df = None):
    # Les roues decluches. rad/s
    max_wheel_speed = robot[4]
    #ax.vlines(np.array([-max_wheel_speed,max_wheel_speed]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black", lw=1)
    #ax.hlines(np.array([-max_wheel_speed,max_wheel_speed]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black", lw=1)

    # Erreur est de (-5,0), (-5,0)
    #cmd_max_speed = np.array([[-5,0,5,0,-5],[0,5,0,-5,0]])
    
    v_max_lin = robot[2]
    v_max_angular = robot[3]
    cmd_max_speed = np.array([[-v_max_lin,v_max_lin,v_max_lin,-v_max_lin,-v_max_lin],[-v_max_angular,-v_max_angular,v_max_angular,v_max_angular,-v_max_angular]])
    b = robot[0] #1.08
    r = robot[1] #0.3
    jac = np.array([[1/2,1/2],[-1/b,1/b]])*r
    jac_inv = np.linalg.inv(jac)
    
    cmd_wheel = jac_inv @ cmd_max_speed

    #ax.plot(cmd_wheel[1,:],cmd_wheel[0,:],color="black",label="max lin and ang speed",lw=1)

    plot_losange_limits(ax,geom_df)
    
    return ax


def add_small_turning_radius_background(ax,first_time =False,robot=[1.08,0.3,5,5,16.6667], geom_df = None):
    """Modify the body frame velocit graph

    Args:
        ax (_type_): _description_
        first_time (bool, optional): _description_. Defaults to False.
        robot (list, optional): _description_. Defaults to [1.08,0.3,5,5,16.6667].

    Returns:
        _type_: _description_
    """
    max_wheel_speed = robot[4] # Les roues decluches. rad/s
    b = robot[0]
    r =robot[1]
    jacob = np.array([[1/2,1/2],[-1/b, 1/b]]) * r
    n_points=11
    ligne_1 = np.linspace(-max_wheel_speed,max_wheel_speed,n_points).reshape(n_points,1)
    other_coordinates = np.zeros((n_points,1))

    cmd_1 = np.hstack((ligne_1,other_coordinates))
    cmd_1_body = jacob @ cmd_1.T
    cmd_2_body = jacob @ np.hstack((other_coordinates,ligne_1)).T

    cmd_max_speed_wheel = np.array([[-max_wheel_speed,-max_wheel_speed,max_wheel_speed,max_wheel_speed,-max_wheel_speed],
                                [-max_wheel_speed,max_wheel_speed,max_wheel_speed,-max_wheel_speed,-max_wheel_speed]])
    cmd_max_speed = jacob @ cmd_max_speed_wheel
    max_speed_lin = robot[2]
    max_speed_ang = robot[3]
    max_body_slip  = np.array([(-max_speed_lin,-max_speed_ang), (-max_speed_lin, max_speed_ang), (max_speed_lin, max_speed_ang), (max_speed_lin, -max_speed_ang),(-max_speed_lin,-max_speed_ang)]).T  # A square
    
    #ax.plot(cmd_max_speed[1,:],cmd_max_speed[0,:],color="black",ls="-",lw=2)
    #ax.plot(max_body_slip[1,:],max_body_slip[0,:],color="black",ls="-",lw=2)
    plot_losange_limits(ax,geom_df)
    
    return ax


def plot_histogramme(ax,df,column_of_interest,transient_only_flag=True,nb_bins=30,x_lim=(0,0),densitybool=True, negative_values=False, color=COLOR_BODY_FRAME):
    if transient_only_flag:
        imu_acceleration_x = column_type_extractor(df,column_of_interest,verbose=False)
        steady_state_mask = column_type_extractor(df,"steady_state_mask")
        steady_state_mask = steady_state_mask[:,:imu_acceleration_x.shape[1]]
        mask = np.where(steady_state_mask==0,True, False)
        imu_acceleration_x = imu_acceleration_x[mask]
        if negative_values:
            imu_acceleration_x = -imu_acceleration_x
        labels_y = column_of_interest+"\n transient_state"
        
    else:
        imu_acceleration_x= column_type_extractor(df,column_of_interest,verbose=False,steady_state=True)
        labels_y = column_of_interest
        if negative_values:
            imu_acceleration_x = -imu_acceleration_x
    
    if x_lim == (0,0):
            ax.hist(imu_acceleration_x,bins=nb_bins,density=densitybool, color=color)
    else:
            ax.hist(imu_acceleration_x,bins=nb_bins,range=x_lim, density=densitybool, color=color)

    print(f"Col of Interest: {column_of_interest}")
    print(f"Mean: {np.mean(imu_acceleration_x)}")
    print(f"Median: {np.median(imu_acceleration_x)}")
    print(f"Maximum: {np.max(imu_acceleration_x)}")
    print(f"Minimum: {np.min(imu_acceleration_x)}")
    print(f"STD: {np.std(imu_acceleration_x)}")
    ax.set_ylabel(f"Probability density")#\n(n = {len(np.ravel(imu_acceleration_x))})")
    ax.set_xlabel(labels_y)
        

def set_commun_y_axis_lim(axs):

    if len(axs.shape) != 1:

        for row in range(axs.shape[0]):
            # Get the y-limits of the first axis
            first_ylim = axs[row,0].get_ylim()
            # Find the maximum y-limit values
            max_ylim = (min(first_ylim[0], *[ax.get_ylim()[0] for ax in axs[row,:]]),
                        max(first_ylim[1], *[ax.get_ylim()[1] for ax in axs[row,:]]))
            
            for ax in axs[row,:]:
                ax.set_ylim(max_ylim)


def scatter_diamond_displacement_graph(df_all_terrain, terrains_to_plot = [],
                                        subtitle="",x_lim=6, y_lim = 8.5,
                                        max_wheel_speed=16.667,
                                        robot={},
                                        axs=[], axis_param=[],
                                        geom_df = None):

    # Assert that the terrains to plot are in the dataframe
    assert all([terrain in df_all_terrain["terrain"].unique() for terrain in terrains_to_plot])
    # Assert that the list of axes fits the number of terrains to plot and the number of plots to make
    #assert sum(axis_param.values()) == len(axs[:,0])
    #assert len(terrains_to_plot) == len(axs[0,:])

    nbr_terrains = len(terrains_to_plot)
    alpha_parama= 0.3
    y_lim = y_lim *1.15
    x_lim = x_lim *1.15
    if geom_df is not None:
        wheel_geom = geom_df["wheel"]
        body_geom = geom_df["body"]

    if nbr_terrains == 1:
        axs = np.array([axs])

    for i in range(nbr_terrains):
        # Create a list of all the plots to do, where the value associated with the key is true
        list_of_plot_to_do = [key for key, value in axis_param.items() if value]
        for k in range(sum(axis_param.values())):
            terrain = terrains_to_plot[i]
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]
            robot_params = robot[terrain]

            if "Wheels_vel" in list_of_plot_to_do:
                axs[k,i].scatter(df["cmd_right_wheels"],df["cmd_left_wheels"],color="orange",alpha=alpha_parama+0.2, s=MARKER_SIZE, edgecolors='none')
                if not ONLY_CMD:
                    axs[k,i].scatter(df["odom_speed_right_wheels"],df["odom_speed_left_wheels"],label='Mean of wheel steady-state speed',color=COLOR_WHEEL_FRAME,alpha=alpha_parama, s=MARKER_SIZE, edgecolors='none')
                axs[k,i].set_title(f"{terrain[0].upper() + terrain[1:]}")
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                if i == 0:
                    axs[k,i].set_ylabel(r"Left wheel speed (rad/s)")
                axs[k,i].set_xlabel(r"Right wheel speed (rad/s)")
                wheels_value = max_wheel_speed *1.25
                axs[k,i].set_ylim((-wheels_value,wheels_value))
                axs[k,i].set_xlim((-wheels_value,wheels_value))
                axs[k,i].set_aspect(1)

                if i == 0 :
                    handles = axs[k,i].get_legend_handles_labels()[0] + axs[k,i].get_legend_handles_labels()[0] 
                    legends = axs[k,i].get_legend_handles_labels()[1] + axs[k,i].get_legend_handles_labels()[1] 
                    axs[k,i] = add_vehicle_limits_to_wheel_speed_graph(axs[k,i],first_time=True,robot=robot_params, geom_df=wheel_geom[terrain])
                else:
                    axs[k,i] = add_vehicle_limits_to_wheel_speed_graph(axs[k,i],robot=robot_params, geom_df=wheel_geom[terrain])
                
                list_of_plot_to_do.remove("Wheels_vel")
                continue

            if "Body_vel" in list_of_plot_to_do:
                #axs[k,i].set_title(f"Body vel on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                if not ONLY_CMD:
                    axs[k,i].scatter(df["icp_vel_yaw_smoothed"],df["icp_vel_x_smoothed"],color = COLOR_BODY_FRAME,label='Mean of body steady-state speed',alpha=alpha_parama, s=MARKER_SIZE, edgecolors='none') 
                axs[k,i].scatter(df["cmd_body_yaw_lwmean"],df["cmd_body_x_lwmean"],color = "orange",label='Command',alpha=alpha_parama+0.2, s=MARKER_SIZE, edgecolors='none')
                
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])        
                axs[k,i].set_xlabel(r"Angular speed (rad/s)")
                if i == 0:
                    axs[k,i].set_ylabel(r"Longitudinal speed (m/s)")
                #axs[k,i].set_title(r"Impact of a terrain on the body frame command ${}^{W}\mathbf{u}$")
                axs[k,i].set_ylim((-5,5))
                axs[k,i].set_xlim((-x_lim,x_lim))
                add_small_turning_radius_background(axs[k,i],robot=robot_params, geom_df=body_geom[terrain])
                list_of_plot_to_do.remove("Body_vel")
                continue
    
    return axs


def acceleration_histogram(df_all_terrain, terrains_to_plot = [],
                           subtitle="", nb_bins=30, x_lim=(-6,6),
                           densitybool=True, transientflag=True,
                           axs=[], axis_param=[],
                           geom_df = None):
    
    # Assert that the terrains to plot are in the dataframe
    assert all([terrain in df_all_terrain["terrain"].unique() for terrain in terrains_to_plot])
    # Assert that the list of axes fits the number of terrains to plot and the number of plots to make
    #assert sum(axis_param.values()) == len(axs[:,0])
    #assert len(terrains_to_plot) == len(axs[0,:])

    nbr_terrains = len(terrains_to_plot)
    
    if nbr_terrains == 1:
        axs = np.array([axs])

    for i in range(nbr_terrains):
        list_of_plot_to_do = [key for key, value in axis_param.items() if value]
        for k in range(sum(axis_param.values())):
            terrain = terrains_to_plot[i]
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]   
            param_alpha = 0.5

            if "Accel_x" in list_of_plot_to_do:
                print(f"Accel x on {terrain}")
                axs[k,i].set_title(f"{terrain[0].upper() + terrain[1:]}")
                plot_histogramme(axs[k,i],df,"imu_acceleration_x",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool, )
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                vx_acceleration_theo = column_type_extractor(df,"step_frame_vx_theoretical_acceleration")
                axs[k,i].hist(vx_acceleration_theo,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins, color="orange")
                axs[k,i].axvline(-5,0,1,color="red")
                axs[k,i].axvline(5,0,1,color="red")
                axs[k,i].set_xlabel(r"Longitudinal acceleration" + "\n" + r"(m/s\textsuperscript{2})")
                list_of_plot_to_do.remove("Accel_x")
                continue

            if "Accel_y" in list_of_plot_to_do:
                print(f"Accel y on {terrain}")
                #axs[k,i].set_title(f"acceleration_y on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                plot_histogramme(axs[k,i],df,"imu_acceleration_y",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
                vy_acceleration_theo = column_type_extractor(df,"step_frame_vy_theoretical_acceleration")
                #axs[k,i].hist(vy_acceleration_theo,density=densitybool)
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                ## compute centripete acceleration
                cmd_vyaw= np.mean(column_type_extractor(df,'cmd_body_vel_yaw'),axis=1)
                cmd_vlin = np.mean(column_type_extractor(df,'cmd_body_vel_x'),axis=1)
                centripete_acceleration = cmd_vlin * cmd_vyaw
                axs[k,i].hist(centripete_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins,color="orange")
                #axs[k,i].vlines(np.array([-5,5]),0,axs[k,i].get_ylim()[1],color="red")
                axs[k,i].set_xlabel(r"Lateral acceleration" + "\n" + r"(m/s\textsuperscript{2})")
                list_of_plot_to_do.remove("Accel_y")
                continue

            if "Accel_yaw" in list_of_plot_to_do:
                print(f"Accel yaw on {terrain}")
                #axs[k,i].set_title(f"acceleration yaw from \n deriv icp on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                plot_histogramme(axs[k,i],df,"step_frame_deriv_vyaw_acceleration",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
                vyaw_acceleration = column_type_extractor(df,"step_frame_vyaw_theoretical_acceleration")
                axs[k,i].hist(vyaw_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins, color="orange")
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                axs[k,i].axvline(-4,0,1,color="red")
                axs[k,i].axvline(4,0,1,color="red")
                list_of_plot_to_do.remove("Accel_yaw")
                continue

            if "Accel_yaw_imu" in list_of_plot_to_do:
                print(f"Accel yaw imu on {terrain}")
                #axs[k,i].set_title(f"acceleration_yaw from \n deriv imu yaw vel {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                plot_histogramme(axs[k,i],df,"imu_deriv_vyaw_acceleration",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
                vyaw_acceleration = column_type_extractor(df,"step_frame_vyaw_theoretical_acceleration")
                axs[k,i].hist(vyaw_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins, color="orange")
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                axs[k,i].axvline(-4,0,1,color="red")
                axs[k,i].axvline(4,0,1,color="red")
                axs[k,i].set_xlabel(r"Angular acceleration" + "\n" + r"(rad/s\textsuperscript{2})")
                list_of_plot_to_do.remove("Accel_yaw_imu")
                continue

            if "Slip_body_y" in list_of_plot_to_do:
                print(f"Slip body y on {terrain}")
                #axs[k,i].set_title(f"Slip body y on {terrain}\n")
                plot_histogramme(axs[k,i],df,"slip_body_y_ss",transient_only_flag=False,nb_bins=nb_bins,x_lim=(-2,2),densitybool=densitybool, negative_values=True, color=COLOR_BODY_FRAME)
                #axs[k,i].set_title(r"Impact of a terrain on the lateral speed")
                #axs[k,i].set_facecolor(COLOR_DICT[terrain])
                axs[k,i].axvline(0,0,1,color="orange", lw=2, linestyle="dashed")
                axs[k,i].set_ylim(0,axs[k,i].get_ylim()[1])
                axs[k,i].set_xlabel(r"Lateral speed (m/s)")
                list_of_plot_to_do.remove("Slip_body_y")
                continue
                
                

            # TODO: Rework the legend
            """
            if i ==0:
                ax_to_plot_2.legend(["useless","Centripetal acceleration"])
                ax_to_plot.legend(["System limits","Measured acceleration","Theoretical acceleration"])

                # Extract legends
                legend_1 = ax_to_plot.get_legend()
                legend_2 = ax_to_plot_2.get_legend()

                # Combine handles and labels
                combined_handles = legend_1.legend_handles + [legend_2.legend_handles[1]]
                combined_labels = [text.get_text() for text in legend_1.get_texts()] + [legend_2.get_texts()[1].get_text()]

                legend_1.remove()
                legend_2.remove()
            """

    #plt.tight_layout()        

    # Apply the same y-limits to all axes
    #set_commun_y_axis_lim(axs[:1])
    
    return

def plot_figure_7():
    SCATTER_PLOT_TO_GENERATE = {"Body_vel": True, "Wheels_vel": True}
    HIST_PLOT_TO_GENERATE = {"Accel_x": False, "Accel_y": False, "Accel_yaw": False, "Accel_yaw_imu": False, "Slip_body_y": True}
    PLOT_VERTICALLY = False

    # Load the data
    df_all_terrain = pd.read_pickle(PATH_DATAFRAME)
    with open(PATH_DATAFRAME_DIAMOND, 'rb') as file:
        geom_by_terrain = pickle.load(file)

    # Create a new subplot with 4 rows and 2 columns using the axes from the previous figure
    if PLOT_VERTICALLY:
        nbr_rows = sum(SCATTER_PLOT_TO_GENERATE.values()) + sum(HIST_PLOT_TO_GENERATE.values())
        fig, axs = plt.subplots(len(TERRAIN_TO_PLOT),nbr_rows)
        fig.set_figwidth(5*nbr_rows)
        fig.set_figheight(5*len(TERRAIN_TO_PLOT))
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        axs_scatter = axs[:,:sum(SCATTER_PLOT_TO_GENERATE.values())].T
        axs_hist = axs[:,sum(SCATTER_PLOT_TO_GENERATE.values()):].T
    else:
        nbr_rows = sum(SCATTER_PLOT_TO_GENERATE.values()) + sum(HIST_PLOT_TO_GENERATE.values())
        fig, axs = plt.subplots(nbr_rows,len(TERRAIN_TO_PLOT))
        fig.set_figwidth(88/25.4)
        fig.set_figheight(140/25.4)
        plt.subplots_adjust(wspace=0.05, hspace=0.05)
        axs_scatter = axs[:sum(SCATTER_PLOT_TO_GENERATE.values())]
        axs_hist = axs[sum(SCATTER_PLOT_TO_GENERATE.values()):]

    # TODO: Changer le max_wheel_speed pour être par terrain et robot
    # Robot : [base_width, wheel_radius, max_lin_speed, max_ang_speed, max_wheel_speed]
    # max_lin_speed, max_ang_speed, max_wheel_speed sont par terrain

    scatter_diamond_displacement_graph(df_all_terrain,
                                        terrains_to_plot=TERRAIN_TO_PLOT,
                                        subtitle="", 
                                        x_lim=6, y_lim=6, 
                                        max_wheel_speed=16.667, 
                                        robot=ROBOT_PARAMS_PER_TERRAIN,
                                        axs=axs_scatter,
                                        axis_param=SCATTER_PLOT_TO_GENERATE,
                                        geom_df=geom_by_terrain)

    acceleration_histogram(df_all_terrain,
                            terrains_to_plot=TERRAIN_TO_PLOT,
                            subtitle="transient",
                            nb_bins=30, x_lim=(-6,6),
                            densitybool=True,
                            transientflag=True,
                            axs=axs_hist,
                            axis_param=HIST_PLOT_TO_GENERATE)
            
    #for ax in np.ravel(axs_scatter):
    #    ax.set_aspect("equal")

    axs_hist[0,1].set_ylabel("")

    # TODO: Rework the legend
    """
    fig.legend(handles=combined_handles ,
                labels=combined_labels, 
                loc='lower center',
                #bbox_to_anchor=(3.5, 1.5),
                ncol=4 )
    """

    plt.tight_layout(rect=[0, 0.08, 1, 1])  # Leave space at the bottom for the legend
    # Add a legend at the bottom of the figure
    # Create custom elements not present in subplots
    command_marker = Line2D([0], [0], color='orange', lw=2, label='Command', linestyle=None, marker='o', alpha=0.5)
    
    green_marker = Line2D([0], [0], color=COLOR_WHEEL_FRAME, lw=2, label='Wheel velocities', linestyle=None, marker='o', alpha=0.5)
    measured_marker = Line2D([0], [0], color=COLOR_BODY_FRAME, lw=2, label='Body velocities', linestyle=None, marker='o', alpha=0.5)
    controller_limit_c = Line2D([0], [0], color='red', lw=2, label=r'Controller limit $\mathcal{C}$', linestyle='-')
    controller_limit_n = Line2D([0], [0], color='black', lw=2, label='Sampling space', linestyle='-')
    ice_patch = Patch(color='aliceblue', label='Custom Patch')
    grass_patch = Patch(color='honeydew', label='Custom Patch')
    # Add a black line around the patches
    for patch in [ice_patch, grass_patch]:
        patch.set_edgecolor('black')
        patch.set_linewidth(1)

    # Combine elements for the legend
    handles = [command_marker, green_marker, measured_marker, controller_limit_n]
    labels = [r'Command', r'Wheel velocities', r'Body velocities', r'Sampling space']
    fig.legend(handles, labels, loc='lower center', ncol=2, prop={'size': 8})
    fig.savefig("tests_figures/fig7.pdf",format="pdf")
    fig.savefig("tests_figures/fig7.png",format="png")
    fig.savefig("tests_figures/fig7.svg",format="svg")

    """
    combined_figures, axs = plt.subplots(2,1, figsize=(20,10))
    for ax, fig in zip(axs, [fig_scatter, fig_hist]):
        for child in fig.axes[0].get_children():
            ax.add_artist(child)
        ax.axis("off")
    combined_figures.savefig("tests_figures/combined_figures.pdf",format="pdf")
    """

def plot_figure_9():
    SCATTER_PLOT_TO_GENERATE = {"Body_vel": False, "Wheels_vel": False}
    HIST_PLOT_TO_GENERATE = {"Accel_x": True, "Accel_y": True, "Accel_yaw": False, "Accel_yaw_imu": True, "Slip_body_y": False}
    PLOT_VERTICALLY = False
    # Load the data
    df_all_terrain = pd.read_pickle(PATH_DATAFRAME)
    with open(PATH_DATAFRAME_DIAMOND, 'rb') as file:
        geom_by_terrain = pickle.load(file)

    # Create a new subplot with 4 rows and 2 columns using the axes from the previous figure
    if PLOT_VERTICALLY:
        nbr_rows = sum(SCATTER_PLOT_TO_GENERATE.values()) + sum(HIST_PLOT_TO_GENERATE.values())
        fig, axs = plt.subplots(len(TERRAIN_TO_PLOT),nbr_rows)
        fig.set_figwidth(5*nbr_rows)
        fig.set_figheight(5*len(TERRAIN_TO_PLOT))
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        axs_scatter = axs[:,:sum(SCATTER_PLOT_TO_GENERATE.values())].T
        axs_hist = axs[:,sum(SCATTER_PLOT_TO_GENERATE.values()):].T
    else:
        nbr_rows = sum(SCATTER_PLOT_TO_GENERATE.values()) + sum(HIST_PLOT_TO_GENERATE.values())
        fig, axs = plt.subplots(nbr_rows,len(TERRAIN_TO_PLOT))
        fig.set_figwidth(88/25.4)
        fig.set_figheight(154/25.4)
        plt.subplots_adjust(wspace=0.05, hspace=0.05)
        axs_scatter = axs[:sum(SCATTER_PLOT_TO_GENERATE.values())]
        axs_hist = axs[sum(SCATTER_PLOT_TO_GENERATE.values()):]

    # TODO: Changer le max_wheel_speed pour être par terrain et robot
    # Robot : [base_width, wheel_radius, max_lin_speed, max_ang_speed, max_wheel_speed]
    # max_lin_speed, max_ang_speed, max_wheel_speed sont par terrain

    scatter_diamond_displacement_graph(df_all_terrain,
                                        terrains_to_plot=TERRAIN_TO_PLOT,
                                        subtitle="", 
                                        x_lim=6, y_lim=8.5, 
                                        max_wheel_speed=16.667, 
                                        robot=ROBOT_PARAMS_PER_TERRAIN,
                                        axs=axs_scatter,
                                        axis_param=SCATTER_PLOT_TO_GENERATE,
                                        geom_df=geom_by_terrain)

    acceleration_histogram(df_all_terrain,
                            terrains_to_plot=TERRAIN_TO_PLOT,
                            subtitle="transient",
                            nb_bins=30, x_lim=(-6,6),
                            densitybool=True,
                            transientflag=True,
                            axs=axs_hist,
                            axis_param=HIST_PLOT_TO_GENERATE,
                            geom_df=geom_by_terrain)
    
    for ax in np.ravel(axs_scatter):
        ax.set_aspect("equal")

    axs_hist[0,1].set_ylabel("")
    axs_hist[1,1].set_ylabel("")
    axs_hist[2,1].set_ylabel("")

    plt.tight_layout(rect=[0, 0.1, 1, 1])  # Leave space at the bottom for the legend
    # Add a legend at the bottom of the figure
    # Create custom elements not present in subplots
    command_marker = Line2D([0], [0], color='orange', lw=1, label='Command', linestyle='None', marker='s', markeredgecolor='None', markersize=4)
    green_marker = Line2D([0], [0], color='magenta', lw=1, label='Magenta', linestyle='None', marker='s', markeredgecolor='None', markersize=4)
    measured_marker = Line2D([0], [0], color='blue', lw=1, label='Measured', linestyle='None', marker='s', markeredgecolor='None', markersize=4)
    controller_limit_c = Line2D([0], [0], color='red', lw=1, label=r'Controller limit $\mathcal{C}$', linestyle='-', markersize=2)
    ice_patch = Patch(color='aliceblue', label='Custom Patch')
    grass_patch = Patch(color='honeydew', label='Custom Patch')
    # Add a black line around the patches
    for patch in [ice_patch, grass_patch]:
        patch.set_edgecolor('black')
        patch.set_linewidth(1)

    # Combine elements for the legend
    handles = [command_marker, measured_marker, controller_limit_c]
    labels = [r'Command', r'Body acc.', r'Vehicle max. acc.']
    fig.legend(handles, labels, loc='lower center', ncol=3, prop={'size': 8})
    # Move up the legend a bit
    fig.subplots_adjust(bottom=0.14)
    fig.savefig("tests_figures/fig9.pdf",format="pdf")
    fig.savefig("tests_figures/fig9.svg",format="svg")
    fig.savefig("tests_figures/fig9.png",format="png")
    

if __name__ == "__main__":

    
    plot_figure_7()
    plot_figure_9()
    plt.show()
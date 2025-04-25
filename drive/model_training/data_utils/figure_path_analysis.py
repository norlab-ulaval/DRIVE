import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.gridspec import GridSpec
from matplotlib.colors import Normalize
from scipy.spatial.transform import Rotation as R
import os

NBR_STEPS = 300
TIME_DELTA = 0.05
PATH_DATAFRAME = "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"#"filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset"#"drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
#INDEX_LIST_TO_PLOT = [2, 3, 12, 20, 25, 30, 35, 41, 47, 122]
INDEX_LIST_TO_PLOT = [i for i in range(200)]
# Create a list of colors to plot the paths of the robot from the inferno colormap of equal length to the index list
COLOR_LIST_TO_PLOT = plt.cm.inferno(np.linspace(0, 1, len(INDEX_LIST_TO_PLOT)))
RANGE_LIMIT = 30
CONSIDER_INIT_TF = True
ROBOT_WIDTH = 2.0
HEATMAP_ABSOLUTE = True
TERRAIN_COLOR_DICT = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew","tile":"mistyrose",
                    "boreal":"lightgray","sand":"lemonchiffon","avide":"white","avide2":"white","wetgrass":"honeydew","mud":"cornsilk"}

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
mpl.rcParams['markers.fillstyle'] = 'full'
mpl.rcParams['scatter.edgecolors'] = 'none'

class Command:
    def __init__(self, cmd_vel, cmd_angle, delta_s, step_nb, initial_pose=[0.0, 0.0, 0.0]):
        # initial pose is [x, y, yaw]
        self.cmd_vel = cmd_vel
        self.cmd_angle = cmd_angle
        self.delta_s = delta_s
        self.step_nb = step_nb
        delta_x = cmd_vel*delta_s
        delta_z_dot = cmd_angle*delta_s
        self.cmd_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.cmd_matrix.astype(float)
        self.cmd_matrix[0, 0] = np.cos(delta_z_dot)
        self.cmd_matrix[0, 1] = -np.sin(delta_z_dot)
        self.cmd_matrix[1, 0] = np.sin(delta_z_dot)
        self.cmd_matrix[1, 1] = np.cos(delta_z_dot)
        self.cmd_matrix[0, 2] = delta_x
        self.initial_pose = initial_pose
        self.initial_transform = np.array([[np.cos(initial_pose[2]), -np.sin(initial_pose[2]), initial_pose[0]], [np.sin(initial_pose[2]), np.cos(initial_pose[2]), initial_pose[1]], [0.0, 0.0, 1.0]])


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


def process_path(cmd : Command):
    planned_path = []
    matrice_pose_init = cmd.initial_transform
    for i in range (cmd.step_nb):
        matrice_pose_init = matrice_pose_init @ cmd.cmd_matrix
        theta_z = np.arctan2(matrice_pose_init[1, 0], matrice_pose_init[0, 0])
        # [x, y, yaw]
        planned_path.append([matrice_pose_init[0, 2], matrice_pose_init[1, 2], theta_z])
    return planned_path


def setup_axis(ax, limits = (-RANGE_LIMIT, RANGE_LIMIT)):
    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")
    if limits is not None:
        min_range, max_range = limits
        ax.set_xlim(max_range, min_range)
        ax.set_ylim(min_range, max_range)
    ax.set_aspect('equal')


def draw_path(ax, limits, path, point_size=2, color='b', alpha=0.07, downsample=1, quiver=False, label='Robot pose'):
    setup_axis(ax, limits)
    path = np.array(path)
    # Find the u,v vectors of the orientation using the yaw angle
    u = np.cos(path[:, 2])
    v = np.sin(path[:, 2])
    # Plot the orientation of the robot as a quiver plot with only 1 point out of 10
    if quiver:
        im = ax.quiver(path[::downsample, 1], path[::downsample, 0], v[::downsample], u[::downsample], angles='xy', scale_units='xy', scale=1, color=color, label=label, rasterized=True)
    else:
        im = ax.scatter(path[::downsample, 1], path[::downsample, 0], edgecolor='none', facecolor=color, s=point_size, alpha=alpha, label=label, marker='.', rasterized=True)
        #im = ax.plot((path[::downsample, 1], path[::downsample, 0]), color=color, linewidth=point_size, alpha=alpha, label=label)

    return im


def draw_path_from_command(ax, limits, cmd, point_size=2, color='b', alpha=0.05, downsample=1, quiver=False, label='Robot pose'):
    planned_path = process_path(cmd)
    planned_path = np.array(planned_path)
    draw_path(ax, limits, planned_path, point_size, color, alpha, downsample, quiver, label)
    return planned_path


def update_path_with_init_tf(path_list, init_tf):
    # init_tf = [x, y, yaw]
    # Create the transformation matrix
    init_tf_matrix = np.array([[np.cos(init_tf[2]), -np.sin(init_tf[2]), init_tf[0]], [np.sin(init_tf[2]), np.cos(init_tf[2]), init_tf[1]], [0.0, 0.0, 1.0]])
    updated_path_matrix = []
    for path in path_list:
        # Path is a list of [x, y, yaw]
        path_matrix = np.array([[np.cos(path[2]), -np.sin(path[2]), path[0]], [np.sin(path[2]), np.cos(path[2]), path[1]], [0.0, 0.0, 1.0]])
        # Multiply the path matrix by the init_tf matrix
        updated_matrix = init_tf_matrix @ path_matrix
        updated_path_matrix.append([updated_matrix[0, 2], updated_matrix[1, 2], np.arctan2(updated_matrix[1, 0], updated_matrix[0, 0])])
    # Return the updated path as a list of [x, y, yaw]
    return updated_path_matrix


def extract_data_from_dataframe(df):
    data_dict = {}
    data_dict["cmd_vel_x"] = df['cmd_body_x_lwmean'].values
    data_dict["cmd_vel_yaw"] = df['cmd_body_yaw_lwmean'].values
    data_dict["max_lin_speed"] = df['max_linear_speed_sampled'].values
    data_dict["icp_x"] = column_type_extractor(df, "step_frame_interpolated_icp_x")
    data_dict["icp_y"] = column_type_extractor(df, "step_frame_interpolated_icp_y")
    data_dict["icp_yaw"] = column_type_extractor(df, "step_frame_interpolated_icp_yaw")
    data_dict["init_tf_x"] = df['init_tf_pose_x'].values
    data_dict["init_tf_y"] = df['init_tf_pose_y'].values
    data_dict["init_tf_yaw"] = df['init_tf_pose_yaw'].values
    return data_dict


def generate_heatmap_from_path(ax, path, range_limit=(-RANGE_LIMIT, RANGE_LIMIT), cmap='inferno', vmax=None):
    min_range, max_range = range_limit
    bins = int(((max_range - min_range) * 2) / ROBOT_WIDTH)
    if vmax is None:
        counts =ax.hist2d(path[:, 1], path[:, 0], bins=bins, cmap=cmap, 
                range=[[min_range, max_range], [min_range, max_range]], cmin=1)
    else:
        counts =ax.hist2d(path[:, 1], path[:, 0], bins=bins, cmap=cmap, 
                range=[[min_range, max_range], [min_range, max_range]], vmax=vmax, cmin=1)
    setup_axis(ax, None)
    # Add a colorbar to the heatmap
    #cbar = plt.colorbar(ax.collections[0], ax=ax)
    #cbar.set_label('Number of points')
    # Compute the mean, median and standard deviation of the number of points in each bin
    mean = np.nanmean(counts[0])
    median = np.nanmedian(counts[0])
    std = np.nanstd(counts[0])
    maximum_value = np.nanmax(counts[0])
    print(f"Mean: {mean}, Median: {median}, Std: {std}, Max: {maximum_value}")
    return


def overlap_planned_vs_executed(df, range_limit=(-RANGE_LIMIT, RANGE_LIMIT), consider_init_tf=True):
    # Find every terrain in the dataframe
    terrains = df['terrain'].unique()
    for terrain in terrains:
        print("Terrain: ", terrain)
        # Create a subfolder for the terrain if it does not exist with all the subfolders
        os.makedirs(f"tests_figures/{terrain}/planned_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/executed_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/combined_path", exist_ok=True)
        # Filter the dataframe to keep only the rows corresponding to the current terrain
        df_terrain = df[df['terrain'] == terrain]

        # Read the number of rows in the dataframe to get the number of commands
        cmd_nbr = df_terrain.shape[0]
        color_planned = 'orange'
        color_executed = 'blue'

        # Get the required data from the dataframe
        data_dict = extract_data_from_dataframe(df_terrain)

        # Final figures for the planned and executed paths
        fig_final, axs_final = plt.subplots(1, 1)

        # Loop over the rows of the dataframe
        for cmd_vel, cmd_angle, max_linear_speed, i in zip(data_dict["cmd_vel_x"], data_dict["cmd_vel_yaw"], data_dict["max_lin_speed"], range(cmd_nbr)):
            # Get the command from the dataframe
            if consider_init_tf:
                cmd = Command(cmd_vel, cmd_angle, TIME_DELTA, NBR_STEPS, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
                draw_path_from_command(axs_final, range_limit, cmd, color=color_planned, label='Planned path')
                path_raw = np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T
                path_with_init_tf = update_path_with_init_tf(path_raw, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
                draw_path(axs_final, range_limit, path_with_init_tf, color=color_executed, label='Executed path')
            else:
                cmd = Command(cmd_vel, cmd_angle, TIME_DELTA, NBR_STEPS)
                draw_path_from_command(axs_final[0], range_limit, cmd, color=color_planned, label='Planned path')
                draw_path(axs_final, range_limit, np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T, color=color_executed, label='Executed path')
            
        fig_final.tight_layout()
            
        # Add a custom legend where blue is executed and orange is planned
        fig_final.legend(['Planned path', 'Executed path'], loc='upper right')
        # Save the final figure with high resolution
        fig_final.savefig(f"tests_figures/{terrain}/planned_vs_executed_paths_with_init_tf_overlap.pdf", format='pdf', dpi=600)
    return


def plot_with_and_without_tf(df, range_limit=RANGE_LIMIT):
    # Find every terrain in the dataframe
    terrains = df['terrain'].unique()
    for terrain in terrains:
        print("Terrain: ", terrain)
        # Create a subfolder for the terrain if it does not exist with all the subfolders
        os.makedirs(f"tests_figures/{terrain}/planned_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/executed_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/combined_path", exist_ok=True)
        # Filter the dataframe to keep only the rows corresponding to the current terrain
        df_terrain = df[df['terrain'] == terrain]

        # Read the number of rows in the dataframe to get the number of commands
        cmd_nbr = df_terrain.shape[0]
        colormap_planned = plt.cm.inferno(np.linspace(0, 1, cmd_nbr))
        colormap_executed = plt.cm.viridis(np.linspace(0, 1, cmd_nbr))

        # Get the required data from the dataframe
        data_dict = extract_data_from_dataframe(df_terrain)

        # Final figures for the planned and executed paths
        fig_final, axs_final = plt.subplots(2, 2)

        # Loop over the rows of the dataframe
        for cmd_vel, cmd_angle, max_linear_speed, i in zip(data_dict["cmd_vel_x"], data_dict["cmd_vel_yaw"], data_dict["max_lin_speed"], range(cmd_nbr)):
            cmd = Command(cmd_vel, cmd_angle, TIME_DELTA, NBR_STEPS)
            draw_path_from_command(axs_final[0][0], range_limit, cmd, color=colormap_planned[i])
            draw_path(axs_final[0][1], range_limit, np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T, color=colormap_executed[i])

            # Get the command from the dataframe
            cmd = Command(cmd_vel, cmd_angle, TIME_DELTA, NBR_STEPS, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
            draw_path_from_command(axs_final[1][0], range_limit, cmd, color=colormap_planned[i])
            path_raw = np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T
            path_with_init_tf = update_path_with_init_tf(path_raw, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
            draw_path(axs_final[1][1], range_limit, path_with_init_tf, color=colormap_executed[i])

        fig_final.tight_layout()
            
        # Add the title to the final figure
        axs_final[0][0].set_title("Planned paths")
        axs_final[0][1].set_title("Executed paths")
        # Remove the axis labels for the combined figure
        axs_final[0][1].set_ylabel("")
        axs_final[1][1].set_ylabel("")
        axs_final[0][1].set_xlabel("")
        axs_final[0][0].set_xlabel("")
        # Save the final figure with high resolution
        fig_final.savefig(f"tests_figures/{terrain}/planned_vs_executed_paths_with_and_without_init_tf.pdf", format='pdf', dpi=600)
    return


def test_heatmap(df, range_limit=RANGE_LIMIT, absolute=True):
    # Find every terrain in the dataframe
    terrains = df['terrain'].unique()
    for terrain in terrains:
        print("Terrain: ", terrain)
        # Create a subfolder for the terrain if it does not exist with all the subfolders
        os.makedirs(f"tests_figures/{terrain}/planned_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/executed_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/combined_path", exist_ok=True)
        # Filter the dataframe to keep only the rows corresponding to the current terrain
        df_terrain = df[df['terrain'] == terrain]
        # Read the number of rows in the dataframe to get the number of commands
        cmd_nbr = df_terrain.shape[0]
        # Create a figure for the heatmap
        fig_heatmap_capped, axs_heatmap_capped = plt.subplots(1, 1)
        fig_heatmap_uncapped, axs_heatmap_uncapped = plt.subplots(1, 1)
        # Get the required data from the dataframe
        data_dict = extract_data_from_dataframe(df_terrain)
        x_pos_list = []
        y_pos_list = []
        x_bin_list = []
        y_bin_list = []
        for i in range(cmd_nbr):
            cmd = Command(data_dict["cmd_vel_x"][i], data_dict["cmd_vel_yaw"][i], TIME_DELTA, NBR_STEPS, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
            planned_path = process_path(cmd)
            planned_path = np.array(planned_path)
            if absolute:
                x_pos_list.append(planned_path[:, 0])
                y_pos_list.append(planned_path[:, 1])
            else:
                x_pos_list.append([])
                y_pos_list.append([])
                x_bin_list.append([])
                y_bin_list.append([])
                # Compute the bin that each point belongs to
                bins = int((range_limit * 2) / ROBOT_WIDTH)
                x_pos = np.digitize(planned_path[:, 0], np.linspace(-range_limit, range_limit, bins+1))
                y_pos = np.digitize(planned_path[:, 1], np.linspace(-range_limit, range_limit, bins+1))
                # Add the x and y positions to the list if there is not already a point at that position
                for pos in range(len(x_pos)):
                    if ((x_pos[pos],y_pos[pos]) in list(zip(x_bin_list[i], y_bin_list[i]))):
                        continue
                    else:
                        x_pos_list[i].append(planned_path[pos, 0])
                        y_pos_list[i].append(planned_path[pos, 1])
                        x_bin_list[i].append(x_pos[pos])
                        y_bin_list[i].append(y_pos[pos])

        x_pos = np.concatenate(x_pos_list)
        y_pos = np.concatenate(y_pos_list)
        planned_path = np.array([x_pos, y_pos]).T
        generate_heatmap_from_path(axs_heatmap_capped, planned_path, range_limit=range_limit, cmap='inferno', vmax=200)
        generate_heatmap_from_path(axs_heatmap_uncapped, planned_path, range_limit=range_limit, cmap='inferno')

        # Add the title to the final figure
        axs_heatmap_capped.set_title("Planned path heatmap capped")
        axs_heatmap_uncapped.set_title("Planned path heatmap uncapped")
        # Save the final figure with high resolution
        fig_heatmap_capped.savefig(f"tests_figures/{terrain}/heatmap_planned_capped.pdf", format='pdf', dpi=600)
        fig_heatmap_uncapped.savefig(f"tests_figures/{terrain}/heatmap_planned_uncapped.pdf", format='pdf', dpi=600)
    return


def create_figure(df, range_limit=RANGE_LIMIT, absolute=True):
    plt.rcParams['text.usetex'] = True
    plt.rcParams['font.family'] = 'serif'  # Use a LaTeX-compatible font
    terrains = df['terrain'].unique()
    for terrain in terrains:
        print("Terrain: ", terrain)
        # Create a subfolder for the terrain if it does not exist with all the subfolders
        os.makedirs(f"tests_figures/{terrain}/planned_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/executed_path", exist_ok=True)
        os.makedirs(f"tests_figures/{terrain}/combined_path", exist_ok=True)
        # Filter the dataframe to keep only the rows corresponding to the current terrain
        df_terrain = df[df['terrain'] == terrain]
        # Read the number of rows in the dataframe to get the number of commands
        cmd_nbr = df_terrain.shape[0]
        colormap_planned = plt.cm.viridis(np.linspace(0, 1, cmd_nbr))
        colormap_executed = plt.cm.viridis(np.linspace(0, 1, cmd_nbr))
        # Create a figure for the heatmap
        fig = plt.figure(figsize=(88/25.4, 98/25.4))
        b_u_ax = fig.add_axes((0.06, 0.55, 0.53, 0.35))
        b_p_ax = fig.add_axes((0.50, 0.55, 0.53, 0.35))
        g_p_ax = fig.add_axes((0.06, 0.1, 0.53, 0.35))
        g_p_heatmap_ax = fig.add_axes((0.50, 0.1, 0.53, 0.35))
        # Get the required data from the dataframe
        data_dict = extract_data_from_dataframe(df_terrain)
        x_pos_list = []
        y_pos_list = []
        x_bin_list = []
        y_bin_list = []
        max_value = 0
        min_value = 100
        # Loop over the rows of the dataframe
        print("Nbr of commands: ", cmd_nbr)
        for cmd_vel, cmd_angle, max_linear_speed, i in zip(data_dict["cmd_vel_x"], data_dict["cmd_vel_yaw"], data_dict["max_lin_speed"], range(cmd_nbr)):
            cmd = Command(cmd_vel, cmd_angle, TIME_DELTA, NBR_STEPS)
            planned_path_global = draw_path_from_command(b_u_ax, range_limit, cmd, color='orange', downsample=1, alpha=0.14)
            draw_path(b_p_ax, range_limit, np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T, color='blue', downsample=1)

            cmd = Command(data_dict["cmd_vel_x"][i], data_dict["cmd_vel_yaw"][i], TIME_DELTA, NBR_STEPS, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
            path_raw = np.array([data_dict["icp_x"][i,:], data_dict["icp_y"][i,:], data_dict["icp_yaw"][i,:]]).T
            path_with_init_tf = update_path_with_init_tf(path_raw, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])
            planned_path_global_with_init_tf = update_path_with_init_tf(planned_path_global, [data_dict["init_tf_x"][i], data_dict["init_tf_y"][i], data_dict["init_tf_yaw"][i]])

            planned_path = np.array(path_with_init_tf)
            max_value = max([np.max(planned_path[:, 0]), np.max(planned_path[:, 1]), max_value])
            min_value = min([np.min(planned_path[:, 0]), np.min(planned_path[:, 1]), min_value])

            draw_path(g_p_heatmap_ax, None, path_with_init_tf, color='blue', downsample=1)
            draw_path(g_p_ax, None, planned_path_global_with_init_tf, color='orange', downsample=1, alpha=0.14)

            if absolute:
                x_pos_list.append(planned_path[:, 0])
                y_pos_list.append(planned_path[:, 1])
            else:
                x_pos_list.append([])
                y_pos_list.append([])
                x_bin_list.append([])
                y_bin_list.append([])
                # Compute the bin that each point belongs to
                bins = int(((max_value-min_value) * 2) / ROBOT_WIDTH)
                x_pos = np.digitize(planned_path[:, 0], np.linspace(min_value, max_value, bins+1))
                y_pos = np.digitize(planned_path[:, 1], np.linspace(min_value, max_value, bins+1))
                # Add the x and y positions to the list if there is not already a point at that position
                for pos in range(len(x_pos)):
                    if ((x_pos[pos],y_pos[pos]) in list(zip(x_bin_list[i], y_bin_list[i]))):
                        continue
                    else:
                        x_pos_list[i].append(planned_path[pos, 0])
                        y_pos_list[i].append(planned_path[pos, 1])
                        x_bin_list[i].append(x_pos[pos])
                        y_bin_list[i].append(y_pos[pos])

        x_pos = np.concatenate(x_pos_list)
        y_pos = np.concatenate(y_pos_list)
        planned_path = np.array([x_pos, y_pos]).T
        #generate_heatmap_from_path(g_p_heatmap_ax, planned_path, range_limit=(min_value,max_value), cmap='inferno')
        g_p_ax.set_xlim(min_value-1, max_value+1)
        g_p_ax.set_ylim(min_value-1, max_value+1)
        g_p_heatmap_ax.set_xlim(min_value-1, max_value+1)
        g_p_heatmap_ax.set_ylim(min_value-1, max_value+1)

        fontsize_label = 8
        labelpad = -0.5
        # Remove the axis labels for the combined figure
        #b_u_ax.set_xlabel(r"${}^{B}y$ (m)", labelpad=labelpad)
        b_u_ax.set_xlabel(r"", labelpad=labelpad)
        b_u_ax.set_ylabel(r"Position $x$ (m)", labelpad=labelpad)
        b_p_ax.set_xlabel(r"", labelpad=labelpad)
        # Add text in the upper right corner of the axes
        b_p_ax.text(0.95, 0.95, r'${}^{B}$', verticalalignment='top', horizontalalignment='right', transform=b_p_ax.transAxes, fontsize=fontsize_label+6)
        b_u_ax.text(0.95, 0.95, r'${}^{B}$', verticalalignment='top', horizontalalignment='right', transform=b_u_ax.transAxes, fontsize=fontsize_label+6)
        b_p_ax.set_ylabel(r"", labelpad=labelpad)
        b_p_ax.set_yticks([])
        val = 25
        b_u_ax.set_xlim(val, -val)
        b_u_ax.set_ylim(-val, val)
        b_p_ax.set_xlim(val, -val)
        b_p_ax.set_ylim(-val, val)
        g_p_ax.set_xlabel(r"Position $y$ (m)", labelpad=labelpad)
        g_p_ax.set_ylabel(r"Position $x$ (m)", labelpad=labelpad)
        g_p_ax.text(0.95, 0.95, r'${}^{G}$', verticalalignment='top', horizontalalignment='right', transform=g_p_ax.transAxes, fontsize=fontsize_label+6)
        g_p_heatmap_ax.text(0.95, 0.95, r'${}^{G}$', verticalalignment='top', horizontalalignment='right', transform=g_p_heatmap_ax.transAxes, fontsize=fontsize_label+6)
        g_p_heatmap_ax.set_xlabel(r"Position $y$ (m)", labelpad=labelpad)
        g_p_heatmap_ax.set_ylabel(r"", labelpad=labelpad)
        g_p_heatmap_ax.set_yticks([])
        
        #axs[0].set_title(r'${}^{B}$')
        #b_u_ax.set_title(r'${}^{B} f(\mathbf{u_\tau})$')
        b_u_ax.set_title(r'Commanded position')
        #b_p_ax.set_title(r'${}^{B} \mathbf{p_\tau}$')
        b_p_ax.set_title(r'Executed position')
        #g_p_ax.set_title(r'${}^{G} \mathbf{p_\tau}$')
        #g_p_ax.set_title(r'Commanded position')
        #g_p_heatmap_ax.set_title(r'${}^{G} \mathbf{p_\tau}$ density')
        #g_p_heatmap_ax.set_title(r'Executed position')
        # Set the background color according to the terrain
        #for ax in [b_u_ax, b_p_ax, g_p_ax, g_p_heatmap_ax]:
        #    ax.set_facecolor(TERRAIN_COLOR_DICT[terrain])
        # Add colorbar to the center bottom of the whole figure
        #cbar_cmd_ax = fig.add_axes((0.16, 0.10, 0.35, 0.01))
        #norm = Normalize(vmin=0, vmax=cmd_nbr)
        #sm = plt.cm.ScalarMappable(norm=norm, cmap=plt.cm.viridis)
        #cbar_cmd = fig.colorbar(sm, cax=cbar_cmd_ax, orientation='horizontal')
        #cbar_heatmap_ax = fig.add_axes((0.60, 0.10, 0.35, 0.01))
        #cbar_heatmap = fig.colorbar(g_p_heatmap_ax.collections[0], cax=cbar_heatmap_ax, orientation='horizontal')
        #cbar_cmd.set_label('Command iteration')
        #cbar_heatmap.set_label('Number of positions')
        # Remove all but the max and min ticks for the colorbars
        #cbar_cmd.ax.set_xticks([])
        #cbar_heatmap.ax.set_xticks([])
        # Reduce the size of the ticks for all the axes
        for ax in [b_u_ax, b_p_ax, g_p_ax, g_p_heatmap_ax]:
            ax.tick_params(axis='both', which='major')
        # Save the final figure with high resolution
        fig.savefig(f"tests_figures/{terrain}/fig_path_analysis.png", format='png', dpi=1200)
        fig.savefig(f"tests_figures/{terrain}/fig_path_analysis.pdf", format='pdf', dpi=300)
        fig.savefig(f"tests_figures/{terrain}/fig_path_analysis.svg", format='svg', dpi=300)

    return


# Load the dataframe
df_all_terrain = pd.read_pickle(PATH_DATAFRAME)

#overlap_planned_vs_executed(df_all_terrain, range_limit = RANGE_LIMIT, consider_init_tf=CONSIDER_INIT_TF)
#plot_with_and_without_tf(df_all_terrain, range_limit = RANGE_LIMIT)
#test_heatmap(df_all_terrain, range_limit = 30, absolute=HEATMAP_ABSOLUTE)

create_figure(df_all_terrain, range_limit = (-RANGE_LIMIT, RANGE_LIMIT), absolute=HEATMAP_ABSOLUTE)
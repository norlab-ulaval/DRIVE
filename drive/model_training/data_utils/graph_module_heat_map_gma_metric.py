import matplotlib.pyplot as plt
import numpy as np 
import pandas as pd
from drive.model_training.data_utils.extractors import *
import pickle 
import shapely
import pathlib
import argparse
import matplotlib.colors as mcolors
import matplotlib as mpl
from matplotlib import gridspec

ROBOT = "warthog"

if ROBOT == "husky":
    DATASET_PICKLE = "drive_datasets/results_multiple_terrain_dataframe/husky_metric_to_watermelon.csv"
    GEOM_PICKLE = "drive_datasets/results_multiple_terrain_dataframe/husky_geom_limits_by_terrain_for_filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    AXIS_LIM = (-2,2)
    # Gaussian parameters
    MU_X = 0
    MU_Y = 0
    SIGMA_X = 0.25
    SIGMA_Y = 0.25
    RHO = 0
elif ROBOT == "warthog":
    DATASET_PICKLE = "drive_datasets/results_multiple_terrain_dataframe/warthog_metric_to_watermelon.csv"
    GEOM_PICKLE = "drive_datasets/results_multiple_terrain_dataframe/warthog_geom_limits_by_terrain_for_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    AXIS_LIM = (-5,5)
    # Gaussian parameters
    MU_X = 0
    MU_Y = 0
    SIGMA_X = 0.8
    SIGMA_Y = 0.8
    RHO = 0
TOGGLE_CLINE = True
TOGGLE_PROPORTIONNAL = False
LIST_OF_TERRAINS_TO_PLOT = ["grass","gravel","mud","sand","ice","asphalt"]
#LIST_OF_TERRAINS_TO_PLOT = ["ice", "grass"]
#LIST_OF_TERRAINS_TO_PLOT = ["ice","asphalt"]

SQUARES_TO_ANALYZE = [{'x': -0.5, 'y':4, 'width': 1, 'height': 1, 'axis': 'linear'},
                      {'x': 3.5, 'y':-1, 'width': 0.5, 'height': 2, 'axis': 'yaw'},]

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

# List of cline factor
#CLINE_DICT = {"slip_body_x_ss":[-0.3, 0.3],
#               "slip_body_y_ss":[-0.2, 0.2], 
#               "slip_body_yaw_ss":[-3, 3]}
LIST_COLUMN_OF_INTEREST  = ["last_window_metric", "last_window_cmd_total_energy_metric"]

CLINE_DICT = {"first_window_metric":[],
              "last_window_metric":[],
              "last_window_cmd_total_energy_metric":[],
              "last_window_cmd_rotationnal_energy_metric":[],
              "last_window_cmd_translationnal_energy_metric":[]}

CLINE_DICT = {}
for col in LIST_COLUMN_OF_INTEREST:
    CLINE_DICT[col] = []


def gaussian_2d(x, y, mu_x=MU_X, mu_y=MU_Y, sigma_x=SIGMA_X, sigma_y=SIGMA_Y, rho=RHO):
    norm_const = 1 / (2 * np.pi * sigma_x * sigma_y * np.sqrt(1 - rho**2))
    exp_part = -1 / (2 * (1 - rho**2)) * (
        ((x - mu_x)**2 / sigma_x**2) + 
        ((y - mu_y)**2 / sigma_y**2) - 
        (2 * rho * (x - mu_x) * (y - mu_y) / (sigma_x * sigma_y))
    )
    return norm_const * np.exp(exp_part)


import math

# Function to count points within elliptical radius
def count_points_within_ellipse(points, target = (0,0), radius_x = SIGMA_X, radius_y = SIGMA_Y):
    count = 0
    for (x, y) in points:
        # Scaled distance using elliptical formula
        distance = ((x - target[0]) / radius_x) ** 2 + ((y - target[1]) / radius_y) ** 2
        if distance <= 1:
            count += 1
    return count


def filtered_mesgrid_cmd_based_on_geom(geom_by_terrain,terrain,x_mesh,y_mesh):

    geom = geom_by_terrain[terrain]

    list_fitler = []

    x_ravel = np.ravel(x_mesh)
    y_ravel = np.ravel(y_mesh)

    for x,y in zip(x_ravel,y_ravel):
        pt = shapely.geometry.Point(x,y)
        if True:#if shapely.within(pt,geom):
            list_fitler.append(True)
        else:
            list_fitler.append(False)

    filter = np.array(list_fitler).reshape(x_mesh.shape)

    return filter


def plot_losange_limits(ax,geom):
    # If ax is an array of ax then plot the losange on each ax
    if isinstance(ax,np.ndarray):
        for ax_ in ax:
            plot_losange_limits(ax_,geom)
    else:
        x,y = geom.exterior.xy
        ax.plot(x,y, color="black")
        # Create a list of points [(x1,y1),(x2,y2),...]
        points = [(x[i],y[i]) for i in range(len(x))]
        # Create a rectangle over the whole plot
        full_rect = [(-6,-6), (6,-6), (6,6), (-6,6), (-6,-6)]
        # Combine the rectangle and the polygon in a single path
        vertices = full_rect + points
        codes = (
            [mpl.path.Path.MOVETO] + [mpl.path.Path.LINETO] * (len(full_rect) - 2) + [mpl.path.Path.CLOSEPOLY] + 
            [mpl.path.Path.MOVETO] + [mpl.path.Path.LINETO] * (len(points) - 2) + [mpl.path.Path.CLOSEPOLY]
        )
        path = mpl.path.Path(vertices, codes)
        mask = mpl.patches.PathPatch(path, facecolor='white', edgecolor='none', alpha=1)
        ax.add_patch(mask)


def plot_image(ax, X_train, mean_prediction, y, x_2_eval, cline_list = [], filter = {},
               shape = (100,100), colormap = "PuOr", x_lim = AXIS_LIM, y_lim = AXIS_LIM,
               vmax = 6, proportionnal = False):
    if ax == None:
        fig, ax = plt.subplots(1,1)

    norm = mcolors.Normalize(vmin=0, vmax=vmax)
    if proportionnal:
        norm = mcolors.LogNorm(vmin=0.1, vmax=vmax)

    if isinstance(filter,np.ndarray): 
        filtered_prediction = np.where(filter,mean_prediction.reshape(shape),0)
        
        im = ax.imshow(filtered_prediction,extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, norm=norm)
        #scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=-vmax, vmax=vmax)
        final_shape = int(np.sqrt(mean_prediction.shape[0]))

    else:
        im = ax.imshow(mean_prediction.reshape(shape),extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, norm=norm)
        #scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=-vmax, vmax=vmax)
        final_shape = int(np.sqrt(mean_prediction.shape[0]))

    
    if cline_list:
        round_vmax = np.round(vmax)
        CS = ax.contour(x_2_eval[:,0].reshape((final_shape,final_shape)),
                x_2_eval[:,1].reshape((final_shape,final_shape)),
                mean_prediction.reshape((final_shape,final_shape)),
                #np.linspace(-round_vmax,round_vmax, int((cline_factor*round_vmax)+1)),
                cline_list,
                colors="black", linewidths=0.5)
        
        ax.clabel(CS, inline=True, fontsize=10)

    return im

def process_gma_meshgrid(X, y, x_2_eval, geom):
    slip_list_mean = []
    slip_list_std = []
    count_list = []
    # Compute the slip for the meshgrid
    for i in x_2_eval:
        # Recenter all the values around the point i
        X_centered = X - i
        # Compute the number of points within a radius of SIGMA_X and SIGMA_Y (assuming they are equal)
        count = count_points_within_ellipse(X_centered, target=(0,0), radius_x=SIGMA_X, radius_y=SIGMA_Y)
        pt = shapely.geometry.Point(i)
        if shapely.within(pt,geom):
            count_list.append(count)

        # Compute the amplitude of the point X_centered with the gaussian function
        amplitude = gaussian_2d(X_centered[:,0],X_centered[:,1])
        slip = np.sum(amplitude*y)/np.sum(amplitude)
        slip_list_mean.append(slip)
        relative_y = y - slip
        slip_list_std.append(np.sqrt(np.sum(amplitude*(relative_y**2))/np.sum(amplitude)))
        
    data_mean = np.array(slip_list_mean)
    data_std = np.array(slip_list_std)
    print(f"Mean count: {np.mean(count_list)}")

    return data_mean, data_std


def process_data(df, list_col_interest,terrain,geom_to_filter = {}, 
                list_colormap = None, col_x_y = ["cmd_right_wheels","cmd_left_wheels"],
                x_lim = AXIS_LIM, y_lim = AXIS_LIM, proportionnal = False,
                nbr_of_samples_to_consider = None):
    
    # Assert that the number of element in list_ax_mean, list_ax_std and list_col_interest are the same
    assert len(list_col_interest) == len(list_colormap)

    # Extract the values 
    vx = np.ravel(column_type_extractor(df,col_x_y[1])) # y
    vyaw = np.ravel(column_type_extractor(df,col_x_y[0])) # x
    X = np.array((vyaw,vx)).T

    # Predict the grid 
    # Define the ranges for x and y
    x_lim_to_plot = np.linspace(x_lim[0], x_lim[1], 100)  # 100 points from -5 to 5
    y_lim_to_plot = np.linspace(y_lim[0], y_lim[1], 100)  # 100 points from -5 to 5

    # Create the meshgrid
    X_2do, Y_2do = np.meshgrid(x_lim_to_plot, y_lim_to_plot)

    if geom_to_filter != {}:
        filter = filtered_mesgrid_cmd_based_on_geom(geom_to_filter,terrain,X_2do,Y_2do)
    else:
        filter = {} 

    x_2_eval = np.array((np.ravel(X_2do),np.ravel(Y_2do))).T

    # Create a list for the data mean and std
    list_data_mean = []
    list_data_std = []
    list_data_std_std = []
    list_y = []

    for i in range(len(list_col_interest)):
        y = np.ravel(column_type_extractor(df, list_col_interest[i]))
        if nbr_of_samples_to_consider is not None and nbr_of_samples_to_consider < len(y):
            X = X[:nbr_of_samples_to_consider]
            y = y[:nbr_of_samples_to_consider]
        data_mean, data_std = process_gma_meshgrid(X, y, x_2_eval, geom_to_filter[terrain])
        if proportionnal:
            # If list_col_interest contains yaw in the name then we need to divide by the angular velocity
            if "yaw" in list_col_interest[i]:
                data_mean = abs(data_mean/np.ravel(X_2do))
                data_std = abs(data_std/np.ravel(X_2do))
            else:
                data_mean = abs(data_mean/np.ravel(Y_2do))
                data_std = abs(data_std/np.ravel(Y_2do))
        list_data_mean.append(data_mean)
        list_data_std.append(data_std)
        # The STD of the std. The lower this number, the more stable the model
        list_data_std_std.append(np.std(list_data_std))
        list_y.append(y)
    
    # Create a dictionary to store the results
    dict_results = {}
    dict_results["list_data_mean"] = list_data_mean
    dict_results["list_data_std"] = list_data_std
    dict_results["list_y"] = list_y
    dict_results["X"] = X
    dict_results["x_2_eval"] = x_2_eval
    dict_results["X_2do"] = X_2do
    dict_results["x_lim"] = x_lim
    dict_results["y_lim"] = y_lim
    dict_results["filter"] = filter
    dict_results["list_data_std_std"] = list_data_std_std
    return dict_results


def plot_heat_map_gaussian_moving_average(data_path, geom_path, cline = True, proportionnal = False, nbr_of_samples_to_consider = None):
    with open(geom_path, 'rb') as file:
        geom_by_terrain = pickle.load(file)["body"]

    #df = pd.read_pickle(data_path)
    df = pd.read_csv(data_path)

    list_col_interest = LIST_COLUMN_OF_INTEREST
    list_colormap = ["plasma_r", "plasma"]
    #list_col_interest = ["last_window_metric"]
    #list_colormap = ["Oranges"]

    list_terrain = list(df.terrain.unique())
    # Remove any terrain that is not in the list of terrain to plot
    list_terrain = [terrain for terrain in list_terrain if terrain in LIST_OF_TERRAINS_TO_PLOT]
    size = len(list_terrain)
    # Add one for the colorbars
    ratio_list = [20 for i in range(size)]
    ratio_list.append(1)
    nbr_rows = len(list_col_interest)
    gs = gridspec.GridSpec(nbr_rows, size+1, width_ratios=ratio_list)
    fig_mean = plt.figure()
    fig_std = plt.figure()
    fig_mean.set_figwidth(88/25.4)
    fig_std.set_figwidth(88/25.4)
    fig_mean.set_figheight(2)
    fig_std.set_figheight(2)
    axs_mean = []
    axs_std = []
    for j in range(nbr_rows):
        axs_mean.append([])
        axs_std.append([])
        for i in range(size+1):
            axs_mean[j].append(fig_mean.add_subplot(gs[j, i]))
            axs_std[j].append(fig_std.add_subplot(gs[j, i]))
    axs_mean = np.array(axs_mean)
    axs_std = np.array(axs_std)
    #axs_mean = np.array([[fig_mean.add_subplot(gs[i, j]) for i in range(3)] for j in range(size+1)])
    #axs_std = np.array([[fig_std.add_subplot(gs[i, j]) for i in range(3)] for j in range(size+1)])
    fig_mean.canvas.manager.set_window_title('Mean Heat Map')
    fig_std.canvas.manager.set_window_title('Standard Deviation Heat Map')
    
    # Create a list by terrain for the data mean, std and y
    terrain_dict = {}

    # Loop over the terrain
    for i in range(size):
        terrain = list_terrain[i]
        print(f"Processing terrain: {terrain}")
        
        df_terrain = df.loc[df["terrain"]==terrain]
        col_x_y = ["cmd_body_yaw_mean","cmd_body_x_mean"]
        
        dict_results = process_data(df_terrain, list_col_interest, terrain, geom_to_filter = geom_by_terrain, 
                                    list_colormap = list_colormap, col_x_y = col_x_y, proportionnal = proportionnal,
                                    nbr_of_samples_to_consider=nbr_of_samples_to_consider)
        
        terrain_dict[terrain] = dict_results

    # Create a dictionary to store the vmax for each colormap
    dict_vmax_mean = {}
    dict_vmax_std = {}
    for j in range(size):
        terrain = list_terrain[j]
        for i in range(len(list_colormap)):
            mean_list = terrain_dict[terrain]["list_data_mean"][i]
            std_list = terrain_dict[terrain]["list_data_std"][i]
            vmax_mean = np.max(np.abs(mean_list[~np.isnan(mean_list)]))
            vmax_std = np.max(np.abs(std_list[~np.isnan(std_list)]))
            if list_colormap[i] in dict_vmax_mean:
                if dict_vmax_mean[list_colormap[i]] < vmax_mean:
                    dict_vmax_mean[list_colormap[i]] = vmax_mean
            else:
                dict_vmax_mean[list_colormap[i]] = vmax_mean
            if list_colormap[i] in dict_vmax_std:
                if dict_vmax_std[list_colormap[i]] < vmax_std:
                    dict_vmax_std[list_colormap[i]] = vmax_std
            else:
                dict_vmax_std[list_colormap[i]] = vmax_std

    # Create a csv for all the data
    data = {"terrain":[], "cmd_body_yaw_mean":[], "cmd_body_x_mean":[]} # "last_window_metric":[], "last_window_cmd_total_energy_metric":[]}
    
    for col in list_col_interest:
        data[col] = []

    for i in range(size):
        terrain = list_terrain[i]
        print(f"Processing terrain: {terrain}")
        if size == 1:
            axs_mean_plot = axs_mean[:]
            axs_std_plot = axs_std[:]
        else:
            axs_mean_plot = axs_mean[:,i]
            axs_std_plot = axs_std[:,i]
                        
            geom = geom_by_terrain[terrain]
            plot_losange_limits(axs_mean_plot,geom)
            plot_losange_limits(axs_std_plot,geom)

        X = terrain_dict[terrain]["X"]
        x_2_eval = terrain_dict[terrain]["x_2_eval"]
        X_2do = terrain_dict[terrain]["X_2do"]
        x_lim = terrain_dict[terrain]["x_lim"]
        y_lim = terrain_dict[terrain]["y_lim"]
        filter = terrain_dict[terrain]["filter"]
        list_im_mean = []
        list_im_std = []
        for j in range(len(list_col_interest)):
            print(f"Processing column: {list_col_interest[j]}")
            data_mean = terrain_dict[terrain]["list_data_mean"][j]
            data_std = terrain_dict[terrain]["list_data_std"][j]
            y = terrain_dict[terrain]["list_y"][j]
            print(f"P95: {np.percentile(abs(data_mean), 95)}")
            print(f"Max: {np.max(data_mean)}")
            print(f"Min: {np.min(data_mean)}")
            list_im_mean.append(plot_image(axs_mean_plot[j], X, data_mean, y, x_2_eval, cline_list = CLINE_DICT[list_col_interest[j]], filter = filter,
                shape = X_2do.shape, colormap = list_colormap[j], x_lim = x_lim, y_lim = y_lim, vmax = dict_vmax_mean[list_colormap[j]], proportionnal = proportionnal))
            list_im_std.append(plot_image(axs_std_plot[j], X, data_std, y, x_2_eval, cline_list = [], filter = filter,
                shape = X_2do.shape, colormap = list_colormap[j], x_lim = x_lim, y_lim = y_lim, vmax = dict_vmax_std[list_colormap[j]], proportionnal = proportionnal))

        for x in range(len(np.ravel(x_2_eval[:,0]))):
            data["terrain"].append(terrain)
            data["cmd_body_yaw_mean"].append(np.ravel(x_2_eval[:,0])[x])
            data["cmd_body_x_mean"].append(np.ravel(x_2_eval[:,1])[x])
            for j in range(len(list_col_interest)):
                data[f"{list_col_interest[j]}"].append(np.ravel(terrain_dict[terrain]["list_data_mean"][j])[x])

        axs_mean_plot[0].set_title(f"{terrain[0].upper() + terrain[1:]}")
        axs_std_plot[0].set_title(f"{terrain[0].upper() + terrain[1:]}")
        axs_mean_plot[-1].set_xlabel("Angular speed\ncommand (rad/s)", labelpad=2)
        axs_mean_plot[-1].set_xlabel("Angular speed\ncommand (rad/s)", labelpad=2)

        if i == 0:
            for ax in axs_mean_plot:
                ax.set_ylabel("Longitudinal speed command (m/s)", labelpad=0.1)
            for ax in axs_std_plot:
                ax.set_ylabel("Longitudinal speed command (m/s)", labelpad=0.1)

    df = pd.DataFrame(data)
    df.to_csv(f"tests_figures/mean_heat_map_gma_{ROBOT}_metric.csv")

    # Add white rectangles on all the plots from -5 to -4 and 4 to 5 on x axis with the full height of the plot
    #for i in range(nbr_rows):
    #    for j in range(size):
    #        axs_mean[i,j].add_patch(plt.Rectangle((-5, -5), 1, 10, fill=True, color='white', alpha=1))
    #        axs_mean[i,j].add_patch(plt.Rectangle((4, -5), 1, 10, fill=True, color='white', alpha=1))
    #        axs_std[i,j].add_patch(plt.Rectangle((-5, -5), 1, 10, fill=True, color='white', alpha=1))
    #        axs_std[i,j].add_patch(plt.Rectangle((4, -5), 1, 10, fill=True, color='white', alpha=1))
            
    for i in range(nbr_rows):
        for j in range(size):
            axs_mean[i,j].set_facecolor("black")
            #axs_mean[i,j].set_aspect('equal', 'box')
            axs_mean[i,j].set_ylim(-5,5)
            axs_std[i,j].set_facecolor("black")
            #axs_std[i,j].set_aspect('equal', 'box')
            if j != 0:
                axs_mean[i,j].set_yticks([])
            if i != 2:
                axs_mean[i,j].set_xticks([-4, 0, 4])

    # Draw the squares to analyze
    #for i in range(size):
    #    terrain = list_terrain[i]
    #    geom = geom_by_terrain[terrain]
    #    for square in SQUARES_TO_ANALYZE:
    #        for j in range(nbr_rows):
    #            axs_mean[j,i].add_patch(plt.Rectangle((square['x'], square['y']), square['width'], square['height'], fill=False, color='black', alpha=1))
    #            axs_std[j,i].add_patch(plt.Rectangle((square['x'], square['y']), square['width'], square['height'], fill=False, color='black', alpha=1))


    if size == 1:
        # Add a colorbar
        cbar = plt.colorbar(list_im_mean[0], cax=axs_mean_plot[0], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean transient\\state metric", labelpad=0.1)  
        cbar = plt.colorbar(list_im_mean[1], cax=axs_mean_plot[1], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean steady\\state metric", labelpad=0.1)
        cbar = plt.colorbar(list_im_mean[2], cax=axs_mean_plot[2], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean steady\\state energy", labelpad=0.1)
        cbar = plt.colorbar(list_im_std[0], cax=axs_std_plot[0], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean transient\\state metric", labelpad=0.1)
        cbar = plt.colorbar(list_im_std[1], cax=axs_std_plot[1], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean steady\\state metric", labelpad=0.1)
        cbar = plt.colorbar(list_im_std[2], cax=axs_mean_plot[2], pad = 0.1, shrink=0.5)
        cbar.set_label(r"Mean steady\\state energy", labelpad=0.1)
    else:
        # Add a colorbar
        cbar = plt.colorbar(list_im_mean[0], cax=axs_mean[0,axs_mean.shape[1]-1], fraction=0.1, pad=0.04)
        cbar.set_label("Steady state unpredictability", labelpad=2)
        # Widden the colorbar
        #cbar.ax.set_aspect(40)
        # Set the width of the colorbar
        #cbar = plt.colorbar(list_im_mean[1], cax=axs_mean[1,axs_mean.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state metric", labelpad=0.1)
        #cbar = plt.colorbar(list_im_mean[2], cax=axs_mean[2,axs_mean.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state total energy", labelpad=0.1)
        #cbar = plt.colorbar(list_im_mean[3], cax=axs_mean[3,axs_mean.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state rotationnal\\energy", labelpad=0.1)
        #cbar = plt.colorbar(list_im_mean[4], cax=axs_mean[4,axs_mean.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state translationnal\\energy", labelpad=0.1)
        #cbar = plt.colorbar(list_im_std[0], cax=axs_std[0,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean transient\\state metric", labelpad=0.1)
        #cbar = plt.colorbar(list_im_std[1], cax=axs_std[1,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state metric", labelpad=0.1)
        #cbar = plt.colorbar(list_im_std[2], cax=axs_std[2,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state energy", labelpad=0.1)
        #cbar = plt.colorbar(list_im_std[3], cax=axs_std[3,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state rotationnal\\energy", labelpad=0.1)
        #cbar = plt.colorbar(list_im_std[4], cax=axs_std[4,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
        #cbar.set_label(r"Mean steady\\state translationnal\\energy", labelpad=0.1)


    # Optional label for the colorbar
    mean_filename = f"mean_heat_map_gma_{ROBOT}_metric.pdf"
    std_filename = f"std_heat_map_gma_{ROBOT}_metric.pdf"
    
    # Increase the width spacing between the subplots
    #fig_mean.tight_layout()
    fig_mean.subplots_adjust(wspace=0.05, hspace=0.25)
    fig_mean.subplots_adjust(left=.15, bottom=.16, right=.99, top=.97)
    # Manually offset the colorbar
    fig_mean.subplots_adjust(right=0.85)
    #pos = axs_mean[0,2].get_position()
    #axs_mean[0,2].set_position([pos.x0, pos.y0, pos.width, pos.height*0.7])
    
    fig_mean.savefig(f"tests_figures/{mean_filename}",format="pdf")
    fig_std.savefig(f"tests_figures/{std_filename}",format="pdf")


def compute_data_statistics(data_path):
    df = pd.read_pickle(data_path)

    list_terrain = list(df.terrain.unique())
    list_col_interest = LIST_COLUMN_OF_INTEREST
    for terrain in list_terrain:
        print(f"Terrain: {terrain}")
        df_terrain = df.loc[df["terrain"]==terrain]
        for i in range(len(list_col_interest)):
            print(f"Data column: {list_col_interest[i]}")
            y = np.ravel(column_type_extractor(df_terrain, list_col_interest[i]))
            print(f"Max: {np.max(y)}")
            print(f"Min: {np.min(y)}")


if __name__=="__main__":
    # Arg parser for the pickle file
    parser = argparse.ArgumentParser(description="Process some arguments.")
    parser.add_argument("--dataset",type=str,help="Path to the pickle file", default=DATASET_PICKLE)
    parser.add_argument("--geom",type=str,help="Path to the pickle file containing the geom", default=GEOM_PICKLE)
    parser.add_argument("--cline",type=bool,help="Plot the cline", default=TOGGLE_CLINE)
    parser.add_argument("--proportionnal", type=bool, help="Plot the proportionnal slip instead of the absolute", default=TOGGLE_PROPORTIONNAL)

    # Parse the arguments
    args = parser.parse_args()

    path = pathlib.Path(parser.parse_args().dataset)
    path_to_geom = pathlib.Path(parser.parse_args().geom)
    cline = parser.parse_args().cline
    proportionnal = parser.parse_args().proportionnal

    plot_heat_map_gaussian_moving_average(path, path_to_geom, cline, proportionnal, nbr_of_samples_to_consider=None)
    #compute_data_statistics(path)
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

result_folder = "results_multiple_terrain_dataframe"

if ROBOT == "husky":
    DATASET_PICKLE = "drive_datasets/"+result_folder+"/filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    GEOM_PICKLE = "drive_datasets/"+result_folder+"/husky_geom_limits_by_terrain_for_filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
    AXIS_LIM = (-2.5,2.5)
    MAX_TRANSLATION = 0.5
    MAX_ROTATION = 1.5
    # Gaussian parameters
    MU_X = 0
    MU_Y = 0
    SIGMA_X = 0.25
    SIGMA_Y = 0.25
    RHO = 0
    XAXIS_TICKS = [-2,0,2]
    # List of cline factor
    CLINE_DICT = {"slip_body_x_ss":[-0.2, 0.2],
                "slip_body_y_ss":[-0.2, 0.2], 
                "slip_body_yaw_ss":[-1, 1]}

    
elif ROBOT == "warthog":
    DATASET_PICKLE = "drive_datasets/"+result_folder+"/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    GEOM_PICKLE = "drive_datasets/"+result_folder+"/warthog_geom_limits_by_terrain_for_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
    AXIS_LIM = (-5,5)
    MAX_TRANSLATION = 2
    MAX_ROTATION = 4
    # Gaussian parameters
    MU_X = 0
    MU_Y = 0
    SIGMA_X = 0.8
    SIGMA_Y = 0.8
    RHO = 0
    XAXIS_TICKS = [-4,0,4]
    # List of cline factor
    CLINE_DICT = {"slip_body_x_ss":[-0.2, 0.2],
               "slip_body_y_ss":[-0.1, 0.1], 
               "slip_body_yaw_ss":[-3, 3]}

LIST_COLORMAP = ["PuOr", "PuOr", "PiYG"]
LIST_COL_INTEREST = ["slip_body_x_ss","slip_body_y_ss","slip_body_yaw_ss"]
LIST_LABELS_STD = [r"${}^{B}g_x$ (m/s)",r"${}^{B}g_y$ (m/s)", r"${}^{B}g_\theta$ (rad/s)"]
LIST_LABELS_MEAN = ["Longitudinal slip (m/s)", "Lateral slip (m/s)", "Yaw slip (rad/s)"]

OVERWRITE_HEIGHT = False
FIG_HEIGHT_INCHES = 3*len(LIST_COL_INTEREST)



TOGGLE_CLINE = True
TOGGLE_PROPORTIONNAL = False
SHOW_DATA = False
MARKER_SIZE =  10
#LIST_OF_TERRAINS_TO_PLOT = ["grass","gravel","mud","sand","ice","asphalt","tile"]
LIST_OF_TERRAINS_TO_PLOT = ["ice", "asphalt"]
#LIST_OF_TERRAINS_TO_PLOT = ["mud", "asphalt"]
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
        if shapely.within(pt,geom):
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


def plot_image(ax, X_train, mean_prediction, y, x_2_eval, cline_list = [], filter = {},
               shape = (100,100), colormap = "PuOr", x_lim = AXIS_LIM, y_lim = AXIS_LIM,
               vmax = 6, proportionnal = False):
    if ax == None:
        fig, ax = plt.subplots(1,1)

    norm = mcolors.Normalize(vmin=-vmax, vmax=vmax)
    if proportionnal:
        norm = mcolors.LogNorm(vmin=0.1, vmax=vmax)

    if isinstance(filter,np.ndarray): 
        filtered_prediction = np.where(filter,mean_prediction.reshape(shape),0)
        
        im = ax.imshow(filtered_prediction,extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, norm=norm)
        
        if SHOW_DATA:
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=-vmax, vmax=vmax,s=MARKER_SIZE)
        final_shape = int(np.sqrt(mean_prediction.shape[0]))

    else:
        im = ax.imshow(mean_prediction.reshape(shape),extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, norm=norm)
        if SHOW_DATA:
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=-vmax, vmax=vmax,s=MARKER_SIZE)
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

    df = pd.read_pickle(data_path)

    list_terrain = list(df.terrain.unique())
    # Remove any terrain that is not in the list of terrain to plot
    list_terrain = [terrain for terrain in list_terrain if terrain in LIST_OF_TERRAINS_TO_PLOT]
    size = len(list_terrain)
    # Add one for the colorbars
    ratio_list = [30 for i in range(size)]
    ratio_list.append(1)
    gs = gridspec.GridSpec(len(LIST_COL_INTEREST), size+1, width_ratios=ratio_list)
    fig_mean = plt.figure()
    fig_std = plt.figure()
    fig_mean.set_figwidth(88/25.4)
    fig_std.set_figwidth(88/25.4)
    if OVERWRITE_HEIGHT:
        fig_mean.set_figheight(FIG_HEIGHT_INCHES)
        fig_std.set_figheight(FIG_HEIGHT_INCHES)
    #fig_mean.set_figheight(3*3)
    #fig_std.set_figheight(3*3)
    axs_mean = []
    axs_std = []
    for j in range(len(LIST_COL_INTEREST)):
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

    if proportionnal:
        list_col_interest = LIST_COL_INTEREST#["slip_body_x_ss","slip_body_yaw_ss"]
        list_colormap = LIST_COLORMAP
    else:
        list_col_interest = LIST_COL_INTEREST #["slip_body_x_ss","slip_body_y_ss","slip_body_yaw_ss"]
        list_colormap = LIST_COLORMAP
    
    # Create a list by terrain for the data mean, std and y
    terrain_dict = {}

    # Loop over the terrain
    for i in range(size):
        terrain = list_terrain[i]
        print(f"Processing terrain: {terrain}")
        
        df_robot = df.loc[df["robot"]==ROBOT]
        df_terrain = df_robot.loc[df_robot["terrain"]==terrain]
        col_x_y = ["cmd_body_yaw_lwmean","cmd_body_x_lwmean"]
        
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

    dict_vmax_mean["PuOr"] = MAX_TRANSLATION
    dict_vmax_mean["PiYG"] = MAX_ROTATION
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


        axs_mean_plot[0].set_title(f"{terrain[0].upper() + terrain[1:]}")
        axs_std_plot[0].set_title(f"{terrain[0].upper() + terrain[1:]}")
        axs_mean_plot[-1].set_xlabel("Angular speed\ncommand (rad/s)", labelpad=0.1)
        axs_mean_plot[-1].set_xlabel("Angular speed\ncommand (rad/s)", labelpad=0.1)

        if i == 0:
            for ax in axs_mean_plot:
                ax.set_ylabel("Longitudinal speed\ncommand (m/s)", labelpad=0.1)
            for ax in axs_std_plot:
                ax.set_ylabel("Longitudinal speed\ncommand (m/s)", labelpad=0.1)

    for i in range(len(LIST_COL_INTEREST)):
        for j in range(size):
            axs_mean[i,j].set_facecolor("black")
            #axs_mean[i,j].set_aspect('equal', 'box')
            axs_mean[i,j].set_ylim(AXIS_LIM[0], AXIS_LIM[1])
            axs_mean[i,j].set_xlim(AXIS_LIM[0], AXIS_LIM[1])
            
            axs_std[i,j].set_facecolor("black")
            #axs_std[i,j].set_aspect('equal', 'box')
            if j != 0:
                axs_mean[i,j].set_yticks([])
            if i != 2:
                axs_mean[i,j].set_xticks([])
            if i == 2:
                axs_mean[i,j].set_xticks(XAXIS_TICKS)

    #axs_mean[0,2].set_xticks([-4,0,4])
    #axs_mean[1,2].set_xticks([-4,0,4])

    if size == 1:

        for i in range(len(LIST_COL_INTEREST)):
            
            # Add a colorbar
            cbar = plt.colorbar(list_im_mean[i], cax=axs_mean_plot[i], pad = 0.1, shrink=0.5)
            cbar.set_label(LIST_LABELS_MEAN[i], labelpad=0.1)  
        
            cbar = plt.colorbar(list_im_std[i], cax=axs_std_plot[i], pad = 0.1, shrink=0.5)
            cbar.set_label(LIST_LABELS_STD[i], labelpad=0.1)
    else:
        # Add a colorbar
        for i in range(len(LIST_COL_INTEREST)):
            
            # Add a colorbar
            cbar = plt.colorbar(list_im_mean[i], cax=axs_mean[i,axs_mean.shape[1]-1], pad = 0.1, shrink=0.5)
            cbar.set_label(LIST_LABELS_MEAN[i], labelpad=0.1)  
        
            cbar = plt.colorbar(list_im_std[i], cax=axs_std[i,axs_std.shape[1]-1], pad = 0.1, shrink=0.5)
            cbar.set_label(LIST_LABELS_STD[i], labelpad=0.1)


    # Optional label for the colorbar
    mean_filename = f"mean_heat_map_gma_{ROBOT}.pdf"
    std_filename = f"std_heat_map_gma_{ROBOT}.pdf"
    
    fig_mean.tight_layout()
    if not OVERWRITE_HEIGHT:
        # Increase the width spacing between the subplots
        
        fig_mean.subplots_adjust(wspace=0.05, hspace=0.25)
        # Manually offset the colorbar
        fig_mean.subplots_adjust(right=0.85)
    
    fig_mean.savefig(f"tests_figures/{mean_filename}",format="pdf")
    fig_std.savefig(f"tests_figures/{std_filename}",format="pdf")
    fig_mean.savefig(f"tests_figures/{(mean_filename[:-4]+'.svg')}",format="svg")
    fig_std.savefig(f"tests_figures/{(std_filename[:-4]+'.svg')}",format="svg")
    


def compute_data_statistics(data_path):
    df = pd.read_pickle(data_path)

    list_terrain = list(df.terrain.unique())
    list_col_interest = LIST_COL_INTEREST
    for terrain in list_terrain:
        print(f"Terrain: {terrain}")
        df_terrain = df.loc[df["terrain"]==terrain]
        for i in range(len(list_col_interest)):
            print(f"Data column: {list_col_interest[i]}")
            y = np.ravel(column_type_extractor(df_terrain, list_col_interest[i]))
            print(f"Max: {np.max(y)}")
            print(f"Min: {np.min(y)}")


def plot_statistics_by_nbr_samples(data_path, geom_path):
    with open(geom_path, 'rb') as file:
        geom_by_terrain = pickle.load(file)["body"]

    df = pd.read_pickle(data_path)

    list_terrain = list(df.terrain.unique())
    # Remove any terrain that is not in the list of terrain to plot
    list_terrain = [terrain for terrain in list_terrain if terrain in LIST_OF_TERRAINS_TO_PLOT]
    size = len(list_terrain)
    
    list_col_interest = ["slip_body_x_ss","slip_body_y_ss","slip_body_yaw_ss"]
    list_colormap = ["PuOr", "PuOr", "PiYG"]
    
    # Create a figure with subplots for each terrain
    fig, axs = plt.subplots(len(list_col_interest),size)
    axs = np.array(axs)

    # Loop over the terrain
    for i in range(len(list_terrain)):
        print(f"Processing terrain: {list_terrain[i]}")
        terrain = list_terrain[i]
        df_terrain = df.loc[df["terrain"]==terrain]
        col_x_y = ["cmd_body_yaw_lwmean","cmd_body_x_lwmean"]
        nbr_samples = []
        std_std_x = []
        std_std_y = []
        std_std_yaw = []

        for j in range(2, 250, 1):
            if(j % 2 == 0):
                print(f"Processing {j} samples")
            dict_results = process_data(df_terrain, list_col_interest, terrain, geom_to_filter = geom_by_terrain, 
                                        list_colormap = list_colormap, col_x_y = col_x_y, proportionnal = proportionnal,
                                        nbr_of_samples_to_consider=j)
            nbr_samples.append(j)
            std_std_x.append(dict_results["list_data_std_std"][0])
            std_std_y.append(dict_results["list_data_std_std"][1])
            std_std_yaw.append(dict_results["list_data_std_std"][2])

        # Set the title of the subplots
        axs[0, i].set_title(f"{terrain[0].upper() + terrain[1:]}")
        axs[0, i].scatter(nbr_samples, std_std_x, s=2)
        axs[1, i].scatter(nbr_samples, std_std_y, s=2)
        axs[2, i].scatter(nbr_samples, std_std_yaw, s=2)
        # Save the nbr_samples and the std_std to a csv file for each terrain for further analysis
        data = {"nbr_samples":nbr_samples, "std_std_x":std_std_x, "std_std_y":std_std_y, "std_std_yaw":std_std_yaw}
        df_csv = pd.DataFrame(data)
        df_csv.to_csv(f"tests_figures/std_std_{ROBOT}_{terrain}.csv")
    
    plt.ylim(0, 0.35)
    fig.savefig(f"tests_figures/std_std_{ROBOT}.pdf",format="pdf")
    plt.show()


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
    #plot_statistics_by_nbr_samples(path, path_to_geom)
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
from drive.model_training.data_utils.metric_energy_boxplot import keep_only_steady_state_and_filter
from matplotlib.colors import Colormap
ROBOT = "warthog"
TERRAIN = "asphalt"

def plot_rotation_energy_ratio(df,terrain,ax,cmap):
    
    df = df[(df.terrain == terrain)]

    # Normalize df["rot_and_weight_measured_energy"]
    rot_normalized = (df["rot_and_weight_measured_energy"] - df["rot_and_weight_measured_energy"].min()) / \
                                           (df["rot_and_weight_measured_energy"].max() - df["rot_and_weight_measured_energy"].min())
    trans_normalized = (df["trans_and_weight_measured_energy"] - df["trans_and_weight_measured_energy"].min()) / \
                                           (df["trans_and_weight_measured_energy"].max() - df["trans_and_weight_measured_energy"].min())
    sc = ax.scatter(rot_normalized, trans_normalized, 
                    c=df["total_energy_metric"],cmap=cmap, 
                    alpha= 1)
    ax.axis('equal')
    ax.title.set_text(terrain)
    ax.set_xlabel("K_X_rot")
    #ax.set_ylim(0,1)
    #ax.set_xlim(0,1)


def plot_energy_metric_correlation(df,robot,terrain):

    df = df[(df.terrain == terrain)]
    df = df[(df.robot == robot)]

    print(df.columns)

    fig, ax = plt.subplots(2,2)

    sc = ax[1,0].scatter(df["measured_rot_ratio"], df["cmd_rot_ratio"], 
                    c=df["total_energy_metric"], cmap='plasma', label="translational slip",
                    alpha= 1)
    ax[1,0].set_xlabel("K_X_rot / K_X ")
    ax[1,0].set_ylabel("K_U_rot / K_U ")
    ax[1,0].axis('equal')

    sc = ax[0,0].scatter(df["measured_rot_ratio"], df["cmd_trans_ratio"], 
                    c=df["total_energy_metric"], cmap='plasma', label="translational slip",
                    alpha= 1)
    ax[0,0].set_ylabel("K_U_trans / K_U ")
    ax[0,0].axis('equal')

    sc = ax[1,1].scatter(df["measured_trans_ratio"], df["cmd_rot_ratio"], 
                    c=df["total_energy_metric"], cmap='plasma', label="translational slip",
                    alpha= 1)
    ax[1,1].set_xlabel("K_X_trans / K_X ")
    ax[1,1].axis('equal')
    ax[1,1].axis('equal')
    
    
    #ax[0,1].set_xlabel("K_X_trans / K_X ")
    #ax[0,1].set_ylabel("K_U_rot / K_U ")
    

    sc = ax[0,1].scatter(df["measured_trans_ratio"], df["cmd_trans_ratio"], 
                    c=df["total_energy_metric"], cmap='plasma', label="translational slip",
                    alpha= 1)
    ax[0,1].axis('equal')
    #ax[1,1].set_xlabel("K_X_trans / K_X ")
    #ax[1,1].set_ylabel("K_U_trans / K_U ")
    
    
    fig.colorbar(sc, ax=ax, label="Unpredictability metric")
   
    #fig.set_title("Energy (K) metric correlation")
    #ax.scatter(df["slip_magnitude_rotation"],df["slip_magnitude_transl"],label = "translational slip")

def koy_fish_graph_by_terrain(df,robot):

    df = df.loc[df["robot"] == robot]
    list_terrain = df.terrain.unique()
    fig, axs = plt.subplots(1,len(list_terrain))

    inches_by_graph = 3
    fig.set_size_inches(len(list_terrain)*inches_by_graph,inches_by_graph )
    print(list_terrain)

    cmap = plt.cm.plasma_r  # Use the same colormap for all graphs

    i = 0
    for ax, terrain in zip(axs, list_terrain):
        if i == 0:
            ax.set_ylabel("K_U_rot / K_U ")
        i+=1
        plot_rotation_energy_ratio(df, terrain, ax, cmap)

    fig.colorbar(plt.cm.ScalarMappable(cmap=cmap), ax=axs, label="Unpredictability metric")
    
    #fig.tight_layout()

def compute_slip_magnitude(df):

    df["slip_x"] = df["cmd_body_lin_vel"] -  df["gt_body_lin_vel"]
    df["slip_y"] = -  df["gt_body_y_vel"] 
    df["slip_magnitude_transl"] = np.sqrt(df["slip_x"]**2 + df["slip_y"]**2)
    df["slip_magnitude_rotation"] = np.abs(df["cmd_body_yaw_vel"] - df["gt_body_yaw_vel"])
    
    df["cmd_rot_ratio"] = df["cmd_metric_idd_rotationnal_j"]/df["cmd_metric_idd_total_j"]
    df["cmd_trans_ratio"] = df["cmd_metric_idd_translationnal_j"]/df["cmd_metric_idd_total_j"]

    df["rot_and_weight_measured_energy"] = df["cmd_metric_total_energy_metric_rotationnal_j_components"] * \
                                df["cmd_metric_total_energy_metric_rotationnal_weights"] 
    df["trans_and_weight_measured_energy"] = df["cmd_metric_total_energy_metric_translationnal_j_components"] * \
                                df["cmd_metric_total_energy_metric_translationnal_weights"] 
    

    df["total_measured_energy"] = df["trans_and_weight_measured_energy"] + df["rot_and_weight_measured_energy"]
    df["measured_rot_ratio"] = df["rot_and_weight_measured_energy"]/df["total_measured_energy"]
    df["measured_trans_ratio"] = df["trans_and_weight_measured_energy"]/df["total_measured_energy"]


    return df

if __name__ == "__main__":
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog = pd.read_csv(path_to_raw_result)
    df_warthog = compute_slip_magnitude(df_warthog)
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky = pd.read_csv(path_to_raw_result)
    df_husky = compute_slip_magnitude(df_husky)
    print(df_husky.columns)

    filtered_df_warthog = keep_only_steady_state_and_filter(df_warthog,119,39,yaw_filter =4.0,
                                    keep_only_steady_state = True,
                                    filter_data = True)
    print("df husky  shape :", df_husky.shape)
    filtered_df_husky = keep_only_steady_state_and_filter(df_husky,119,39,yaw_filter =4.0,
                                    keep_only_steady_state = True,
                                    filter_data = True)
    print("df husky  shape :", filtered_df_husky.shape)
    
   

    df_concat = pd.concat([filtered_df_warthog,filtered_df_husky])
    
    df_concat = df_concat[(df_concat.terrain == "asphalt") | (df_concat.terrain == "ice")]
    #plot_energy_metric_correlation(df_concat,ROBOT,TERRAIN)
    koy_fish_graph_by_terrain(df_concat,ROBOT)
    plt.show()
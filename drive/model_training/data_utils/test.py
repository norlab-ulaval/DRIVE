print(0.092/0.043, 0.092/0.052 )

print( (0.092/0.043 + 0.092/0.052 )/2)


print( (1/2 * (0.177/0.5 +  0.154/0.5))**(-1))




import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
path_old_husky = "drive_datasets/results_multiple_terrain_dataframe_copy_backup/filtered_cleared_path_husky_following_robot_param_all_terrain_steady_state_dataset.pkl"
path_old_warthog = "drive_datasets/results_multiple_terrain_dataframe_copy_backup/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"

path_new_warthog = "drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
df_all = pd.read_pickle("drive_datasets/results_multiple_terrain_dataframe_copy_backup/all_terrain_steady_state_dataset.pkl")


df_warthog_old = pd.read_pickle(path_old_warthog)
df_warthog_new = pd.read_pickle(path_new_warthog)

filtered_df = df_warthog_new.loc[(np.abs(df_warthog_new["cmd_body_yaw_lwmean"]) <=4.0) & (np.abs(df_warthog_new["cmd_body_x_lwmean"]) <=4.0) ]

columns = ["cmd_body_x_lwmean","cmd_body_yaw_lwmean"]
columns = ["terrain"]
print("old warthog")
print(df_warthog_old[columns].describe())
print("______________counts")
print(df_warthog_old[columns].value_counts())

print("_____ NEW")

print(df_warthog_new[columns].describe())
print("______________counts")
print(df_warthog_new[columns].value_counts())

print("_____ NEW filtered by yaw")

print(filtered_df[columns].describe())
print("______________counts")
print(filtered_df[columns].value_counts())



for terrain in df_all.terrain.unique():
    df = df_all.loc[df_all["terrain"] == terrain]
    fig, axs = plt.subplots(1,1)
    axs.scatter(df["cmd_body_yaw_lwmean"],df["cmd_body_x_lwmean"],label=terrain,alpha=0.1)
    axs.set_title(terrain)
    plt.show()
filtered_df.plot.scatter("cmd_body_yaw_lwmean","cmd_body_x_lwmean",label="filtered",c="orange",alpha=0.1)
df_warthog_old.plot.scatter("cmd_body_yaw_lwmean","cmd_body_x_lwmean",label="old",c="red",alpha=0.1)
df_warthog_new.plot.scatter("cmd_body_yaw_lwmean","cmd_body_x_lwmean",label="new",alpha=0.1)

plt.show()
import pandas as pd 

from extractors import *

df = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/steady_state_results.pkl")
df_torch = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/torch_ready_dataframe.pkl")

df_raw = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/raw_dataframe.pkl")
df_raw = df_raw.loc[1:]
df_raw.reset_index(inplace=True)

df_raw.cmd_vel_x = df_raw.cmd_vel_x.astype(float)
df_raw.ros_time = df_raw.ros_time.astype(float)
time_raw = (df_raw.ros_time - df_raw.ros_time[0]) * 10**(-9) 


print(df_raw.columns)

starting_step = 3
print_column_unique_column(df)

cmd_body_vel_x = column_type_extractor(df,"cmd_body_vel_x")[starting_step:,:]
cmd_body_vel_yaw = np.ravel(column_type_extractor(df,"cmd_body_vel_yaw")[starting_step:,:])

step_frame_vx = column_type_extractor(df,"step_frame_vx")[starting_step:,:]
step_frame_vy = column_type_extractor(df,"step_frame_vy")[starting_step:,:]
step_frame_vel = np.sqrt(step_frame_vx**2 + step_frame_vy**2)
start_time = column_type_extractor(df,"start_time")[starting_step:,:]


time_matrix = np.array([np.arange(0,120) *0.05] * start_time.shape[0])
print(time_matrix.shape)
time_matrix = time_matrix + np.array(start_time) - start_time[0,0]
print(time_matrix.shape)


time_axis = np.arange(0,cmd_body_vel_x.shape[0]) *0.05

fig, ax = plt.subplots(2,1)

#ax[0].plot(np.ravel(time_axis),cmd_body_vel_x,label="cmd",alpha=0.2)
ax[0].plot(time_raw,df_raw.cmd_vel_x,label="cmd_raw",alpha=0.2)
ax[0].legend()
ax[0].set_xlim(0,400)

for i in range(step_frame_vx.shape[0]):
    if i ==0:
        ax[1].plot(time_matrix[i,:],step_frame_vel[i,:],label="measure speed norm",alpha=0.2)
        ax[1].plot(time_matrix[i,:],cmd_body_vel_x[i,:],label="cmd vel x",alpha=0.2)
    else:
        ax[1].plot(time_matrix[i,:],step_frame_vel[i,:], alpha=0.8)
        ax[1].plot(time_matrix[i,:],abs(cmd_body_vel_x[i,:]), alpha=0.8)
ax[1].legend()
ax[1].set_xlim(20,55)
ax[1].set_ylim(-1,5)


print_column_unique_column(df_torch)

print(time_matrix)
plt.show()

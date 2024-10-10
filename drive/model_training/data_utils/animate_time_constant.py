import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
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
from matplotlib import cm
import pathlib
cur_dir = pathlib.Path().cwd()
parent_dir = cur_dir.parent

import sys
print(sys.version)
sys.path.append('../')




import matplotlib.animation as animation
from matplotlib.backend_bases import KeyEvent



# Update the line data
def update(anim_i,dummy,time_axis,line,line2,line3,predictions,gt_of_interest_reshpae,cmd_of_interest_reshape, ax,names):
    line.set_data(time_axis, predictions[anim_i,:])
    line2.set_data(time_axis, gt_of_interest_reshpae[anim_i,:])
    line3.set_data(time_axis, cmd_of_interest_reshape[anim_i,:])
    #ax.set_title(f"time constant {time_constants_computed[anim_i]} \n time_delay {time_delay_computed} \n gains {gains_computed} ")
    ax.set_title(f"{names[0]} Step={anim_i}")
    return line,line2,line3

# Function to handle key presses
def produce_video(predictions,time_axis,cmd_of_interest_reshape,gt_of_interest_reshpae ,names=["cmd","model","measurement"],video_saving_path=""): 
    
    # Initialize data
    data = {'x': [], 'y': []}

    # Create a figure and axis
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2,label ="Model")
    line2, = ax.plot([],[],label = "GT")
    line3, = ax.plot([],[],label="Cmd")
    # Set up the plot limits
    ax.set_xlim(0, 6.2)
    
    ax.legend()
    
    y_lim_min = np.min(np.array([np.min(predictions),np.min(cmd_of_interest_reshape),np.min(gt_of_interest_reshpae)])) 
    y_lim_max = np.max(np.array([np.max(predictions),np.max(cmd_of_interest_reshape),np.max(gt_of_interest_reshpae)])) 
    ax.set_ylim(y_lim_min, y_lim_max)
    ax.set_xlim(0, 6)
    ax.vlines(np.array([2,4,6]),y_lim_min,y_lim_max)
    anim_i = 0

    dump_thing_of_the_corrected_size = np.zeros((time_axis.shape[0]))
    ani = animation.FuncAnimation(fig, update,fargs=[dump_thing_of_the_corrected_size,time_axis,line,line2,line3,predictions,gt_of_interest_reshpae,cmd_of_interest_reshape, ax,names], frames=predictions.shape[0], interval=1000, blit=True)

    # Save the animation as a video
    final_path_2_save_video = f'{names[0]}_step_by_step_visualisation.mp4'

    if video_saving_path!="":
        path_2_save_video = video_saving_path/final_path_2_save_video

    
    ani.save(path_2_save_video, writer='ffmpeg')



# Update the line data
def update_traj(anim_i,x,y,x_interpolated,y_interpolated,x_corrected,y_corrected,scat,scat2,scat3, ax,names):

    array_like = (np.vstack((x[anim_i,:],y[anim_i,:]))).T
    #print(array_like.shape)
    scat.set_offsets(array_like)
    scat.set_array(np.arange(x.shape[1]))


    array_like = (np.vstack((x_interpolated[anim_i,:],y_interpolated[anim_i,:]))).T
    #print(array_like.shape)
    scat2.set_offsets(array_like)
    scat2.set_array(np.arange(x.shape[1]))


    array_like = (np.vstack((x_corrected[anim_i,:],y_corrected[anim_i,:]))).T
    #print(array_like.shape)
    scat3.set_offsets(array_like)
    scat3.set_array(np.arange(x.shape[1]))
    #ax.set_title(f"time constant {time_constants_computed[anim_i]} \n time_delay {time_delay_computed} \n gains {gains_computed} ")
    #ax.set_title(f"{names[0]} Step={anim_i}")
    return scat,

# Function to handle key presses
def produce_video_traj(x,y,x_interpolated,y_interpolated,x_corrected,y_corrected,imu_yaw,names=["trajectory"],video_saving_path=""): 
    
    # Initialize data
    data = {'x': [], 'y': []}

    # Create a figure and axis
    fig, axs = plt.subplots(3,1)
    fig.set_figheight(3*4)
    fig.set_figwidth(4)

   

    quiver_x_y = np.array([[np.cos(yaw), np.sin(yaw)] for yaw in imu_yaw])


    scat = axs[0].scatter([], [],c=[], lw=2,label ="raw", cmap="viridis")
    scat2 = axs[1].scatter([], [],c=[], lw=2,label ="interpolated", cmap="plasma")
    scat3 = axs[2].scatter([], [],c=[], lw=2,label ="corrected", cmap="jet")
    
    # Set up the plot limits
    #ax.set_xlim(0, 6.2)

    names = ["traj raw", "traj interpolated", "traj imu corrected"]
    
    for ax,name in zip(axs,names):
        ax.set_ylim(np.min(y), np.max(y))
        ax.set_xlim(np.min(x), np.max(x))
        ax.set_title(name)
    #ax.set_xlim(0, 6)

    ani = animation.FuncAnimation(fig, update_traj,fargs=[x,y,x_interpolated,y_interpolated,x_corrected,y_corrected,scat,scat2,scat3,ax,names], frames=x.shape[0], interval=1000, blit=True)

    # Save the animation as a video
    final_path_2_save_video = f'{names[0]}_step_by_step_visualisation.mp4'

    if video_saving_path!="":
        path_2_save_video = video_saving_path/final_path_2_save_video

    
    ani.save(path_2_save_video, writer='ffmpeg')









# Update the line data
def update_traj_quiver(anim_i,list_x,list_y,list_scat,u_v_quiver, ax,names):
    i_ax = 0
    cmap = cm.viridis
    for x,y,scat in zip(list_x,list_y,list_scat):
        array_like = (np.vstack((x[anim_i,:],y[anim_i,:]))).T
        scat.set_offsets(array_like)
        scat.set(color=cmap(np.arange(x.shape[1])))

        min = np.min(np.array([np.min(y[anim_i,:]),np.min(x[anim_i,:]) ]))
        max = np.max(np.array([np.max(y[anim_i,:]),np.max(x[anim_i,:]) ]))
        ax[i_ax].set_ylim(np.min(y[anim_i,:]), np.max(y[anim_i,:]) )
        ax[i_ax].set_xlim(np.min(x[anim_i,:]), np.max(x[anim_i,:]) )
        #ax[i_ax].set_aspect('equal', adjustable='box')
        scat.set_UVC(u_v_quiver[:,0],u_v_quiver[:,1])


        ax[i_ax].set_title(f"{names[i_ax]} Step={anim_i}")
        i_ax += 1 

    
    return scat,

# Function to handle key presses
def produce_video_traj_quiver(x,y,x_interpolated,y_interpolated,x_corrected,y_corrected,imu_yaw,names=["trajectory"],video_saving_path=""): 
    
    # Initialize data
    data = {'x': [], 'y': []}

    # Create a figure and axis
    fig, axs = plt.subplots(3,1)
    fig.set_figheight(4*3)
    fig.set_figwidth(4)

    quiver_x_y = np.array([[np.cos(yaw), np.sin(yaw)] for yaw in imu_yaw])

    zero_like_ = np.zeros(x.shape)
    scat = axs[0].quiver(zero_like_, zero_like_,zero_like_,zero_like_, label ="raw", cmap="viridis",animated=True)
    scat2 = axs[1].quiver(zero_like_, zero_like_,zero_like_,zero_like_, label ="interpolated", cmap="plasma",animated=True)
    scat3 = axs[2].quiver(zero_like_, zero_like_,zero_like_,zero_like_, label ="corrected", cmap="jet",animated=True)
    
    # Set up the plot limits
    #ax.set_xlim(0, 6.2)

    names = ["traj raw", "traj interpolated", "traj imu corrected"]
    
    for ax,name in zip(axs,names):
        ax.set_ylim(np.min(y), np.max(y))
        ax.set_xlim(np.min(x), np.max(x))
        ax.set_title(name)
    #ax.set_xlim(0, 6)

    ani = animation.FuncAnimation(fig, update_traj_quiver,fargs=[[x,x_interpolated,x_corrected], [y,y_interpolated,y_corrected],[scat,scat2,scat3],quiver_x_y,
                                axs,names], frames=x.shape[0], interval=1000, blit=True)

    # Save the animation as a video
    final_path_2_save_video = f'{names[0]}_step_by_step_visualisation.mp4'

    if video_saving_path!="":
        path_2_save_video = video_saving_path/final_path_2_save_video

    
    ani.save(path_2_save_video, writer='ffmpeg')
import pandas as pd
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib as mpl
import pathlib


import matplotlib.animation as animation
from matplotlib import colormaps as cm
from drive.model_training.data_utils.extractors import *
from tqdm import tqdm
import yaml


class GraphicProductionDrive():

    def __init__(self,path_to_dataframe_slip,path_to_dataframe_diamond,path_to_config_file="",result_folder_prefix="",rate=20):
        """open the dataframe

        Args:
            path_to_dataframe (_type_): path_to_dataframe
            dataframe_type (_type_): model_training_datasets [slip_dataset_all, torch_ready_dataset]
        """
        if isinstance(path_to_dataframe_slip,str):
            path_to_dataframe_slip = pathlib.Path(path_to_dataframe_slip)
            print("test")

        if isinstance(path_to_dataframe_diamond,str):
            path_to_dataframe_diamond = pathlib.Path(path_to_dataframe_diamond)
            print("test2")

        self.path_to_analysis = path_to_dataframe_slip.parent/(result_folder_prefix+"analysis")

        if self.path_to_analysis.is_dir() == False:
            self.path_to_analysis.mkdir()
        
        self.df_slip = pd.read_pickle(path_to_dataframe_slip)
        self.df_diamond = pd.read_pickle(path_to_dataframe_diamond)

        self.step_shape = column_type_extractor(self.df_diamond,"step_frame_vx_predictions").shape
        self.time_axis = np.arange(self.step_shape[1]) * 1/rate
        #print_column_unique_column(self.df_slip)
        self.n_iteration_by_windows = column_type_extractor(self.df_slip,"step_frame_vx").shape[1]

        self.n_windows = int(self.step_shape[1]//self.n_iteration_by_windows)

        #[print(column) for column in print_column_unique_column(self.df_slip)]

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

        self.color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew","tile":"mistyrose",
                        "boreal":"lightgray","sand":"lemonchiffon","avide":"white","avide2":"white","wetgrass":"honeydew","mud":"cornsilk"}

        param_path_ = "drive/model_training/data_utils/robot_param.yaml"
        
        with open(param_path_ ) as file:
            self.param_robot = yaml.safe_load(file)["robot"]
        

    def create_window_filter_axis(self):
        increment = 1/(self.n_iteration_by_windows-1)

    def add_vehicle_limits_to_wheel_speed_graph(self,ax,first_time =False,robot=[1.08,0.3,5,5,16.6667]):
        # Les roues decluches. rad/s
        max_wheel_speed = robot[4]
        ax.vlines(np.array([-max_wheel_speed,max_wheel_speed]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black")
        ax.hlines(np.array([-max_wheel_speed,max_wheel_speed]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black")
        
        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([-max_wheel_speed,max_wheel_speed]),color="black",ls="--",lw=2)
        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([max_wheel_speed,-max_wheel_speed]),color="black",ls="dotted",lw=2)
        
        if first_time:
            ax.vlines(np.array([0]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black",ls="dashdot",label=r"$r = \frac{b}{2}$",lw=2)
            ax.hlines(np.array([0]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black",ls="dashdot",lw=2)
        else:
            ax.vlines(np.array([0]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black",ls="dashdot",lw=2)
            ax.hlines(np.array([0]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black",ls="dashdot",lw=2)

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

        ax.plot(cmd_wheel[1,:],cmd_wheel[0,:],color="red",label="max lin and ang speed",lw=2)

        
        return ax
    
    def add_small_turning_radius_background(self,ax,first_time =False,robot=[1.08,0.3,5,5,16.6667]):
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


        if first_time:
            ax.plot(cmd_1_body[0,:],cmd_1_body[1,:],color="black",ls="dashdot",label=r"$r = \frac{b}{2}$",lw=2)
            ax.plot(cmd_2_body[0,:],cmd_2_body[1,:],color="black",ls="dashdot",lw=2)
            
            ax.plot(cmd_max_speed[1,:],cmd_max_speed[0,:],color="black",ls="dashdot",lw=2)
            ax.plot(max_body_slip[1,:],max_body_slip[0,:],color="red",ls="dashdot",lw=2)
            
        else:
            ax.plot(cmd_1_body[0,:],cmd_1_body[1,:],color="black",lw=2)
            ax.plot(cmd_2_body[0,:],cmd_2_body[1,:],color="black",ls="dashdot",lw=2)
            
            ax.plot(cmd_max_speed[1,:],cmd_max_speed[0,:],color="black",ls="dashdot",lw=2)
            ax.plot(max_body_slip[1,:],max_body_slip[0,:],color="red",ls="dashdot",lw=2)
        
        return ax
    
    def wheel_graph_info(self,ax,x_lim=6, y_lim = 8.5,max_wheel_speed=16.667):
        
        max_wheel_speed = max_wheel_speed # Les roues decluches. rad/s
        
        ax.vlines(np.array([-max_wheel_speed,max_wheel_speed]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black",label="vehicle limits")
        ax.hlines(np.array([-max_wheel_speed,max_wheel_speed]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black")

        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([-max_wheel_speed,max_wheel_speed]),label="r = inf :straight line",color="black",ls="--",lw=2)
        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([max_wheel_speed,-max_wheel_speed]),label="r = 0 : Turning on spot",color="black",ls="dotted")

        x_array = np.array([-max_wheel_speed,0,max_wheel_speed,0])
        y_array = np.array([0,-max_wheel_speed,0,max_wheel_speed])
        s_array = [r"$r=\frac{-b}{2}$", r"$r=\frac{b}{2}$",r"$r=$$\frac{-b}{2}$",r"$r=\frac{b}{2}$"]

        repetition = 5
        offset = -1
        x_array = np.linspace(0,0,repetition)+offset
        y_array = np.linspace(-max_wheel_speed,0,repetition)
        s_array = [r"$\frac{-b}{2}$"]*repetition
        #for x,y,s in zip(x_array,y_array,s_array):
        #    ax.text(x,y,s)\frac{b}{2}

        repetition=3
        x = np.linspace(-max_wheel_speed,0,repetition)
        y2 = np.linspace(-max_wheel_speed,0,repetition)
        y1 = np.zeros(x.shape)
        #ax.text(x[1],y2[1]*3/2,r"$r>\frac{b}{2}$")
        #ax.text(x[1]*3/2,y2[1]*1/2,r"$r<\frac{-b}{2}$")
        ax.fill_betweenx(x,y1,y2,label=r"$\frac{b}{2}<r<inf$",color="blue",alpha=0.1)
        
        
        
        
        x2 = np.linspace(0,max_wheel_speed,repetition)
        
        y4 = np.zeros(x.shape)
        y3 = np.ones(x.shape) * -max_wheel_speed
        
        ax.fill_between(x2,y3,y4,label=r"$\frac{-b}{2}<r<\frac{b}{2}$",color="red",alpha=0.1)
        ax.text(x2[1]-2,y3[1]/2,r"$r=0$")
        ax.text(-x2[1]-2,-y3[1]/2,r"$r=0$")
        ax.text(-2,2-max_wheel_speed,r"$r=\frac{b}{2}$")
        ax.text(-2,-2+max_wheel_speed,r"$r=\frac{b}{2}$")
        ax.text(-5+max_wheel_speed,-0.5,r"$r=\frac{-b}{2}$")
        ax.text(-5-max_wheel_speed,-0.5,r"$r=\frac{-b}{2}$")
        
        ax.fill_between(x,y2,0,label=r"$-inf<r<\frac{-b}{2}$",color="yellow",alpha=0.1)
        
        
        ax.fill_between(-x2,-y3,y4,color="red",alpha=0.1)
        x = np.linspace(0,max_wheel_speed,repetition)
        
        
        ax.fill_between(x,x,max_wheel_speed,color="blue",alpha=0.1)
        
        ax.fill_between(x,x,0,color="yellow",alpha=0.1)
        
        #ax.legend(ncols=2,bbox_to_anchor=(0, 1))

        ax.set_ylabel("left_wheel speed [rad/s]")
        ax.set_xlabel("right wheel speed [rad/s]")

               
            
        return ax, ax.get_legend_handles_labels()[0], ax.get_legend_handles_labels()[1]


    def scatter_diamond_displacement_graph(self,df_all_terrain,subtitle="",x_lim=6, y_lim = 8.5,max_wheel_speed=16.667,robot=[1.08,0.3,5,5,16.6667]):
        
        list_terrain = df_all_terrain["terrain"].unique()
        size = len(list_terrain)+1
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(3*3)
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        alpha_parama= 0.3
        y_lim = y_lim *1.15
        x_lim = x_lim *1.15

        for i in range(size):  
            if size == 1:
                ax_to_plot = axs[0]
                ax_to_plot_2 = axs[1]
            else:
                ax_to_plot = axs[0,i]
                ax_to_plot_2 = axs[1,i]
            
            if i == size-1:
                ax,handle,legend = self.wheel_graph_info(axs[1,i],x_lim=x_lim, y_lim = y_lim,max_wheel_speed=robot[4])
                ax_to_plot_2.set_title(f"Graph analyzer helper")
                self.add_small_turning_radius_background(ax_to_plot,first_time=True,robot=robot)
            else:
                terrain = list_terrain[i]
                df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]   

                ax_to_plot.set_title(f"Body vel on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                ax_to_plot.scatter(df["cmd_body_yaw_lwmean"],df["cmd_body_x_lwmean"],color = "orange",label='Command',alpha=alpha_parama)
                ax_to_plot.scatter(df["icp_vel_yaw_smoothed"],df["icp_vel_x_smoothed"],color = "blue",label='Mean of body steady-state speed',alpha=alpha_parama) 
                ax_to_plot.set_facecolor(self.color_dict[terrain])
            
                ax_to_plot_2.scatter(df["cmd_right_wheels"],df["cmd_left_wheels"],color="orange",alpha=alpha_parama)
                ax_to_plot_2.scatter(df["odom_speed_right_wheels"],df["odom_speed_left_wheels"],label='Mean of wheel steady-state speed',color="green",alpha=alpha_parama)
                #axs[0][1].set_title("Command VS Body vel \n (ICP derivate)")
                ax_to_plot_2.set_title(f"Wheels vel on {terrain}")


                self.add_small_turning_radius_background(ax_to_plot,robot=robot)
                ax_to_plot_2.set_facecolor(self.color_dict[terrain])

            ax_to_plot.set_xlabel("Angular velocity (omega) [rad/s]")
            ax_to_plot.set_ylabel("Forward velocity (V_x) [m/s]")
            ax_to_plot.set_ylim((-y_lim,y_lim))
            ax_to_plot.set_xlim((-x_lim,x_lim))
            #back_ground_color = df.color .unique()

            ax_to_plot_2.set_ylabel("left_wheel speed [rad/s]")
            ax_to_plot_2.set_xlabel("right wheel speed [rad/s]")

            wheels_value = max_wheel_speed *1.25
            ax_to_plot_2.set_ylim((-wheels_value,wheels_value))
            ax_to_plot_2.set_xlim((-wheels_value,wheels_value))
            ax_to_plot_2.set_aspect(1)
            

            if i ==0 :
                
                handles = ax_to_plot.get_legend_handles_labels()[0] + ax_to_plot_2.get_legend_handles_labels()[0] 
                legends = ax_to_plot.get_legend_handles_labels()[1] + ax_to_plot_2.get_legend_handles_labels()[1] 
                ax_to_plot_2 = self.add_vehicle_limits_to_wheel_speed_graph(ax_to_plot_2,first_time=True,robot=robot)
            else:
                ax_to_plot_2 = self.add_vehicle_limits_to_wheel_speed_graph(ax_to_plot_2,robot=robot)
                
        fig.legend(handles+handle,legends+legend, loc='center', bbox_to_anchor=(0.5, 0.45), ncol=3)

        
        if subtitle=="":
            fig.suptitle(f"Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        #fig.patch.set_facecolor(color_background)
        #plt.tight_layout()
        
        return fig 
    
    def plot_histogramme(self,ax,df,column_of_interest,transient_only_flag=True,nb_bins=30,x_lim=(0,0),densitybool=True):


        if transient_only_flag:
            imu_acceleration_x = column_type_extractor(df,column_of_interest,verbose=False)
            steady_state_mask = column_type_extractor(df,"steady_state_mask")

            steady_state_mask = steady_state_mask[:,:imu_acceleration_x.shape[1]]

            mask = np.where(steady_state_mask==0,True, False)
            imu_acceleration_x = imu_acceleration_x[mask]

            labels_y = column_of_interest+"\n transient_state"

            
        else:
            imu_acceleration_x= column_type_extractor(df,column_of_interest,verbose=False,steady_state=True)
            labels_y = column_of_interest


        
        if x_lim == (0,0):
                ax.hist(imu_acceleration_x,bins=nb_bins,density=densitybool)
        else:
                ax.hist(imu_acceleration_x,bins=nb_bins,range=x_lim, density=densitybool)
        ax.set_ylabel(f"Probability density function (n = {len(np.ravel(imu_acceleration_x))})")

        ax.set_xlabel(labels_y)
    def set_commun_y_axis_lim(self,axs):

        if len(axs.shape) != 1:

            for row in range(axs.shape[0]):
                # Get the y-limits of the first axis
                first_ylim = axs[row,0].get_ylim()
                # Find the maximum y-limit values
                max_ylim = (min(first_ylim[0], *[ax.get_ylim()[0] for ax in axs[row,:]]),
                            max(first_ylim[1], *[ax.get_ylim()[1] for ax in axs[row,:]]))
                
                for ax in axs[row,:]:
                    ax.set_ylim(max_ylim)
        
    def acceleration_histogram(self,df_all_terrain,subtitle="",nb_bins=30,x_lim=(-6,6),densitybool=True,transientflag=True):
        
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)

        fig, axs = plt.subplots(4,size)
        fig.set_figwidth(3*size)
        fig.set_figheight(size*2.5)
        plt.subplots_adjust(wspace=0.5, hspace=0.8)     
        
        for i in range(size):  
            if size == 1:
                ax_to_plot = axs[0]
                ax_to_plot_2 = axs[1]
                ax_to_plot_3 = axs[2]
                ax_to_plot_4 = axs[3]
            else:
                ax_to_plot = axs[0,i]
                ax_to_plot_2 = axs[1,i]
                ax_to_plot_3 = axs[2,i]
                ax_to_plot_4 = axs[3,i]
            
            terrain = list_terrain[i]
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]   
            
            param_alpha = 0.5
            ax_to_plot.set_title(f"acceleration_x on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
            self.plot_histogramme(ax_to_plot,df,"imu_acceleration_x",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
            ax_to_plot.set_facecolor(self.color_dict[terrain])
            
            vx_acceleration_theo = column_type_extractor(df,"step_frame_vx_theoretical_acceleration")
            ax_to_plot.hist(vx_acceleration_theo,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins)
            ax_to_plot.vlines(np.array([-5,5]),0,ax_to_plot.get_ylim()[1],color="red")
            

            

            ax_to_plot_2.set_title(f"acceleration_y on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
            
            
            self.plot_histogramme(ax_to_plot_2,df,"imu_acceleration_y",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
            vy_acceleration_theo = column_type_extractor(df,"step_frame_vy_theoretical_acceleration")
            #ax_to_plot_2.hist(vy_acceleration_theo,density=densitybool)
            ax_to_plot_2.set_facecolor(self.color_dict[terrain])

            ## compute centripete acceleration
            cmd_vyaw= np.mean(column_type_extractor(df,'cmd_body_vel_yaw'),axis=1)
            cmd_vlin = np.mean(column_type_extractor(df,'cmd_body_vel_x'),axis=1)
            
            centripete_acceleration = cmd_vlin * cmd_vyaw

            ax_to_plot_2.hist(centripete_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins,color="green")



            
            #ax_to_plot_2.vlines(np.array([-5,5]),0,ax_to_plot_2.get_ylim()[1],color="red")
            

            ax_to_plot_3.set_title(f"acceleration yaw from \n deriv icp on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
            self.plot_histogramme(ax_to_plot_3,df,"step_frame_deriv_vyaw_acceleration",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
            
            vyaw_acceleration = column_type_extractor(df,"step_frame_vyaw_theoretical_acceleration")
            ax_to_plot_3.hist(vyaw_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins)

            ax_to_plot_3.set_facecolor(self.color_dict[terrain])
            ax_to_plot_3.vlines(np.array([-4,4]),0,ax_to_plot_3.get_ylim()[1],color="red")
            
            
            #ax_to_plot_2.vlines(np
            # 

            ax_to_plot_4.set_title(f"acceleration_yaw from \n deriv imu yaw vel {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
            self.plot_histogramme(ax_to_plot_4,df,"imu_deriv_vyaw_acceleration",transient_only_flag=transientflag,nb_bins=nb_bins,x_lim=x_lim,densitybool=densitybool)
            vyaw_acceleration = column_type_extractor(df,"step_frame_vyaw_theoretical_acceleration")
            ax_to_plot_4.hist(vyaw_acceleration,density=densitybool,alpha=param_alpha,range=x_lim,bins=nb_bins)
            ax_to_plot_4.set_facecolor(self.color_dict[terrain])
            ax_to_plot_4.vlines(np.array([-4,4]),0,ax_to_plot_4.get_ylim()[1],color="red")

            if i ==0:
                ax_to_plot_2.legend(["useless","Centripetal acceleration"])
                ax_to_plot.legend(["System limits","Measured acceleration","Theoretical acceleration"],
                                  )
                

                # Extract legends
                legend_1 = ax_to_plot.get_legend()
                legend_2 = ax_to_plot_2.get_legend()

                # Combine handles and labels
                combined_handles = legend_1.legend_handles + [legend_2.legend_handles[1]]
                combined_labels = [text.get_text() for text in legend_1.get_texts()] + [legend_2.get_texts()[1].get_text()]

                legend_1.remove()
                legend_2.remove()

                ax_to_plot.legend(handles=combined_handles ,
                                labels=combined_labels, 
                                loc='center',bbox_to_anchor=(3.5, 1.5),ncol=4 )



        if subtitle=="":
            fig.suptitle(f"Acceleration_histogram for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"Acceleration_histogram for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        #fig.patch.set_facecolor(color_background)
        #plt.tight_layout()
        
                    

        # Apply the same y-limits to all axes
        self.set_commun_y_axis_lim(axs)
        
        return fig 
    
    def scatter_diamond_displacement_graph_diff(self,df_all_terrain,subtitle=""):
        
        list_terrain = df_all_terrain["terrain"].unique()
        size = len(list_terrain)+1
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(3*3)
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        alpha_parama= 0.3
        y_lim = 6 * 2
        x_lim = 8.5 * 2
        
        for i in range(size):  
            if size == 1:
                ax_to_plot = axs[0]
                ax_to_plot_2 = axs[1]
            else:
                ax_to_plot = axs[0,i]
                ax_to_plot_2 = axs[1,i]
            
            if i == size-1:
                ax,handle,legend = self.wheel_graph_info(axs[1,i])
                ax_to_plot_2.set_title(f"Graph analyzer helper")
                self.add_small_turning_radius_background(ax_to_plot,first_time=True)
            else:
                terrain = list_terrain[i]
                df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]   


                #
                ax_to_plot.set_title(f"Body vel on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
                ax_to_plot.scatter(df["cmd_body_yaw_lwmean"]-df["step_frame_vyaw_operation_points"],df["cmd_body_x_lwmean"]-df["step_frame_vx_operation_points"],color = "orange",label='Command',alpha=alpha_parama)
                ax_to_plot.scatter(df["icp_vel_yaw_smoothed"]-df["step_frame_vyaw_operation_points"],df["icp_vel_x_smoothed"]-df["step_frame_vx_operation_points"],color = "blue",label='Mean of body steady-state speed',alpha=alpha_parama) 
                ax_to_plot.set_facecolor(self.color_dict[terrain])
            
                ax_to_plot_2.scatter(df["cmd_right_wheels"]-df["right_wheel_vel_operation_points"],df["cmd_left_wheels"]-df["left_wheel_vel_operation_points"],color="orange",alpha=alpha_parama)
                ax_to_plot_2.scatter(df["odom_speed_right_wheels"]-df["right_wheel_vel_operation_points"],df["odom_speed_left_wheels"]-df["left_wheel_vel_operation_points"],label='Mean of wheel steady-state speed',color="green",alpha=alpha_parama)
                #axs[0][1].set_title("Command VS Body vel \n (ICP derivate)")
                ax_to_plot_2.set_title(f"Centered Wheels vel on {terrain}")


                #self.add_small_turning_radius_background(ax_to_plot)
                ax_to_plot_2.set_facecolor(self.color_dict[terrain])



            ax_to_plot.set_xlabel("Angular velocity (omega) [rad/s]")
            ax_to_plot.set_ylabel("Forward velocity (V_x) [m/s]")
            ax_to_plot.set_ylim((-y_lim,y_lim))
            ax_to_plot.set_xlim((-x_lim,x_lim))
            #back_ground_color = df.color .unique()
            
            
            
            ax_to_plot_2.set_ylabel("left_wheel speed [rad/s]")
            ax_to_plot_2.set_xlabel("right wheel speed [rad/s]")
            
            

            
            
            
            wheels_value = 20 * 2
            ax_to_plot_2.set_ylim((-wheels_value,wheels_value))
            ax_to_plot_2.set_xlim((-wheels_value,wheels_value))
            ax_to_plot_2.set_aspect(1)
            

            if i ==0 :
                
                handles = ax_to_plot.get_legend_handles_labels()[0] + ax_to_plot_2.get_legend_handles_labels()[0] 
                legends = ax_to_plot.get_legend_handles_labels()[1] + ax_to_plot_2.get_legend_handles_labels()[1] 
                #ax_to_plot_2 = self.add_vehicle_limits_to_wheel_speed_graph(ax_to_plot_2,first_time=True)
            else:
                #ax_to_plot_2 = self.add_vehicle_limits_to_wheel_speed_graph(ax_to_plot_2)
                test =2 
        fig.legend(handles+handle,legends+legend, loc='center', bbox_to_anchor=(0.5, 0.45), ncol=3)

            #fig.patches .set_facecolor()
        
        #l, legends_label = axs[0][0].get_legend_handles_labels()
        
        #l2, legends_label_2 = axs[1][0].get_legend_handles_labels()
        #axs[0][-1].set_axis_off()
        #axs[1][-1].set_axis_off()
        
        #legends_labels = legends_label+legends_label_2

        #lines = l+l2
        #axs[0][-1].legend(lines, legends_labels, loc = 'center', bbox_to_anchor = (0, -0.55, 1, 1),
        #            bbox_transform = plt.gcf().transFigure,ncol=3)
        if subtitle=="":
            fig.suptitle(f"Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        #fig.patch.set_facecolor(color_background)
        #plt.tight_layout()
        
        return fig 

    def box_plot(self,df, column_of_interest):

        text = "step_frame_vx_time_constants_to_show"
        
        list_terrain = list(df["terrain"].unique())
        size = len(list_terrain)

        # 1. Find all the sub dataframe and compute median 

        # 2. Reorder them by median  
        
        # 3. Boxplot them by order of median 

        # 4. 
        
        
    def scatter_plot_heat_map(self,df,column_x_y_z, ax_to_plot, cmap_name,background_terrain_dict,ylim,xlim,global_cmap,labels_xyz,alpha,show_x_label=False,list_operation_points = []):

        norm_slip = mpl.colors.Normalize(0, vmax=np.abs(global_cmap[1][column_x_y_z[2]].max()))
        #norm_slip = mpl.colors.Normalize(0, vmax=2)
    
        if not global_cmap[0]:

            if list_operation_points != []:
                scatter_ax = ax_to_plot.scatter(df[column_x_y_z[0]]-df[list_operation_points[0]],df[column_x_y_z[1]]-df[list_operation_points[1]],c = np.abs(df[column_x_y_z[2]]),cmap=cmap_name,alpha=alpha)
            else:
                scatter_ax = ax_to_plot.scatter(df[column_x_y_z[0]],df[column_x_y_z[1]],c = np.abs(df[column_x_y_z[2]]),cmap=cmap_name,alpha=alpha)
        else:    
            
            if list_operation_points != []:
                scatter_ax = ax_to_plot.scatter(df[column_x_y_z[0]]-df[list_operation_points[0]],df[column_x_y_z[1]]-df[list_operation_points[1]],c = np.abs(df[column_x_y_z[2]]),cmap=cmap_name,norm=norm_slip,alpha=alpha)
            else:
                scatter_ax = ax_to_plot.scatter(df[column_x_y_z[0]],df[column_x_y_z[1]],c = np.abs(df[column_x_y_z[2]]),cmap=cmap_name,norm=norm_slip,alpha=alpha)

        if show_x_label:
            ax_to_plot.set_xlabel(labels_xyz[0])  
        ax_to_plot.set_ylabel(labels_xyz[1]) 
        cbar = plt.colorbar(scatter_ax, ax=ax_to_plot)
        cbar.set_label(labels_xyz[2])
        ax_to_plot.set_ylim((-ylim,ylim))
        ax_to_plot.set_xlim((-xlim,xlim))
        ax_to_plot.set_facecolor(mpl.colors.to_rgba(background_terrain_dict,0.3))

    def plot_diamond_graph_slip_heat_map(self,df_all_terrain,subtitle="",diff_referential= False,global_cmap = True, y_lim= 6, x_lim =8.5):

        
        
        #print_column_unique_column(df_all_terrain)
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)
        fig, axs = plt.subplots(3,size)
        plt.subplots_adjust(wspace=0.25, hspace=0.25)
        fig.set_figwidth(3.5*size)
        fig.set_figheight(3*3)
        alpha_parama= 0.3

        if diff_referential == False:
            scale_axes = 1.15
        else:
            scale_axes = 2

        y_lim = y_lim * scale_axes # m/s
        x_lim = x_lim* scale_axes#m/s
        cmap = 'viridis' # 
        norm_slip_yaw = mpl.colors.Normalize(0, vmax=df_all_terrain["slip_body_yaw_ss"].max())

        norm_slip_wheel = mpl.colors.Normalize(0, 0)

        scatter_colored_plot_slip_x = []
        scatter_colored_plot_slip_yaw = []  

        global_cmap = [global_cmap,df_all_terrain]
        
        alpha_scatter = 0.5
        
        for i in range(size):  
            terrain = list_terrain[i]
            
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]
            
            labels_xyz  = ["Angular velocity (omega) [rad/s]" , "Forward velocity (V_x) [m/s]",""] 
            column_x_y_z = ["cmd_body_yaw_lwmean","cmd_body_x_lwmean", "slip_body_x_ss"]
            
            col_operation_points = ["step_frame_vyaw_operation_points","step_frame_vx_operation_points"]
            
            show_x_label = False
            #### Slip x 
            
            if size == 1:
                ax_to_plot = axs[0]
            else:
                ax_to_plot = axs[0,i]
            
            labels_xyz[2] = r"$\textbf{Slip x [m/s]}$" 

            if diff_referential:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label,list_operation_points=col_operation_points)
            else:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            ax_to_plot.set_title(f"Steady-state slip for {terrain}") # (ICP smooth by spline,yaw=imu)
            ##### Slip y 
            
            column_x_y_z[2] = "slip_body_y_ss"

            if size == 1:
                ax_to_plot = axs[1]
            else:
                ax_to_plot = axs[1,i]
            labels_xyz[2] = r"$\textbf{Slip y [m/s]}$"
            ######### slip y  ####################
            if diff_referential:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label,list_operation_points=col_operation_points)
            else:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            ### Slip yaw 
            column_x_y_z[2] = "slip_body_yaw_ss"
            show_x_label = True
            if size == 1:
                ax_to_plot = axs[2]
            else:
                ax_to_plot = axs[2,i]
            ######### slip y  ####################
            labels_xyz[2] = r"$\textbf{Slip yaw [rad/s]}$"
            if diff_referential:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label,list_operation_points=col_operation_points)
            else:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
        
        
        #axs[0][-1].legend(lines, legends_labels, loc = 'center', bbox_to_anchor = (0, -0.55, 1, 1),
        #            bbox_transform = plt.gcf().transFigure,ncol=3)
        if subtitle=="":
            fig.suptitle(f"Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        fig.tight_layout()

        #print('Note that the yaw angle in the the smooth version is the IMU')

        return fig
    
    def plot_diamond_graph_wheel_slip_heat_map(self,df_all_terrain,diff_referential= False,subtitle="",global_cmap = True,x_lim=20,y_lim=20):


        #print_column_unique_column(df_all_terrain)
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(2*3)
        alpha_parama= 0.3

        if diff_referential == False:
            scale_axes = 1.15
        else:
            scale_axes = 2


        y_lim = y_lim  *scale_axes # m/s
        x_lim = x_lim *scale_axes#m/s
        cmap = 'viridis' # 
        norm_slip_yaw = mpl.colors.Normalize(0, vmax=df_all_terrain["slip_body_yaw_ss"].max())

        norm_slip_wheel = mpl.colors.Normalize(0, 0)

        scatter_colored_plot_slip_x = []
        scatter_colored_plot_slip_yaw = []  

        global_cmap = [global_cmap,df_all_terrain]
        
        alpha_scatter = 0.5
        
        for i in range(size):  
            terrain = list_terrain[i]
            
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]
            col_operation_points = ["right_wheel_vel_operation_points","left_wheel_vel_operation_points"]
            
            labels_xyz  = ["Right wheel angular velocity [rad/s]","Left wheel angular velocity [rad/s]",""] 
            column_x_y_z = ["cmd_right_wheels","cmd_left_wheels", "slip_wheel_left_ss"]
            show_x_label = False
            #### Slip x 
            if size == 1:
                ax_to_plot = axs[0]
            else:
                ax_to_plot = axs[0,i]
            
            labels_xyz[2] = r"$\textbf{Slip left wheel [m/s]}$"
            if diff_referential:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label,list_operation_points=col_operation_points)
            else:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
            
            ax_to_plot.set_title(f"Steady-state slip for {terrain}") # (ICP smooth by spline,yaw=imu)
            ##### Slip y 
            
            column_x_y_z[2] = "slip_wheel_right_ss"

            if size == 1:
                ax_to_plot = axs[1]
            else:
                ax_to_plot = axs[1,i]
            labels_xyz[2] = r"$\textbf{Slip right wheel [m/s]}$"
            ######### slip y  ####################
            if diff_referential:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label,list_operation_points=col_operation_points)
            else:
                self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", self.color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
            
            
        
        #axs[0][-1].legend(lines, legends_labels, loc = 'center', bbox_to_anchor = (0, -0.55, 1, 1),
        #            bbox_transform = plt.gcf().transFigure,ncol=3)
        if subtitle=="":
            fig.suptitle(f"Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        fig.tight_layout()

        #print('Note that the yaw angle in the the smooth version is the IMU')

        return fig
    def produce_slip_histogramme_by_roboticist_for_a_specific_linear_sampling_speed(self,robiticis_specific=True,densitybool=True):

        df_diamond = self.df_diamond

        list_df_to_use = []
        list_title = [] 
        list_file_name = []
        fig_prefix= "histogram_slip"

        path_to_save = self.path_to_analysis/"heatmap_slip"

        if path_to_save.is_dir() == False:
            path_to_save.mkdir()
        for sampling_lin_speed in df_diamond["max_linear_speed_sampled"].unique():
            df_sampling_speed = df_diamond.loc[df_diamond["max_linear_speed_sampled"]==sampling_lin_speed]

            if robiticis_specific:
                for roboticist in df_sampling_speed["roboticist"].unique():
                
                
                    df_sampling_speed_roboticist = df_sampling_speed.loc[df_sampling_speed["roboticist"] == roboticist]
                    list_df_to_use.append(df_sampling_speed_roboticist)
                    list_title.append(f"Roboticist {roboticist} with a maximum linear sampling speed of {sampling_lin_speed} m/s")
                    list_file_name.append(f"{fig_prefix}_max_sampling_lin_speed_{sampling_lin_speed}_{roboticist}.pdf")
            else:
                list_df_to_use.append(df_sampling_speed)
                list_title.append(f"All roboticist with a maximum linear sampling speed of {sampling_lin_speed} m/s")
                list_file_name.append(f"{fig_prefix}_max_sampling_lin_speed_{sampling_lin_speed}_all.pdf")
        
        
        # Produce the graphs 
        for title, df,file_name in tqdm(zip(list_title,list_df_to_use,list_file_name),colour="green"):

            fig = self.plot_diamond_graph_slip_heat_map(df,global_cmap = True,subtitle=title)
            fig.savefig(path_to_save/("body_slip_"+file_name),format="pdf")
            plt.close('all')
            print("fig1 done")
            fig2 = self.plot_diamond_graph_wheel_slip_heat_map(df,subtitle=title,global_cmap = True)
            fig2.savefig(path_to_save/("wheel_slip_"+file_name),format="pdf")
            plt.close('all')
            print("fig2 done")
            fig3 = self.scatter_diamond_displacement_graph(df,subtitle=title)
            fig3.savefig(path_to_save/("displacement_diamond_"+file_name),format="pdf")
            plt.close('all')
            print("fig3 done")

            #fig4 = self.scatter_diamond_displacement_graph_diff(df,subtitle=title)
            #fig4.savefig(path_to_save/("diff_displacement_diamond_"+file_name),format="pdf")
            #plt.close('all')
            #fig5 = self.plot_diamond_graph_slip_heat_map(df,global_cmap = True,subtitle=title,diff_referential=True)
            #fig5.savefig(path_to_save/("diff_frame_body_slip_"+file_name),format="pdf")
            #plt.close('all')
            #fig6 = self.plot_diamond_graph_wheel_slip_heat_map(df,subtitle=title,global_cmap = True,diff_referential=True)
            #fig6.savefig(path_to_save/("diff_frame_wheel_slip_"+file_name),format="pdf")
            #plt.close('all')

            fig7 = self.acceleration_histogram(df,subtitle="transient",nb_bins=30,x_lim=(-6,6),densitybool=densitybool,transientflag=True)
            fig7.savefig(path_to_save/("acceleration_transient_hist"+file_name),format="pdf")
            plt.close('all')
            print("fig7 done")
            #fig8 = self.acceleration_histogram(df,subtitle="all",nb_bins=30,x_lim=(-6,6),densitybool=densitybool,transientflag=False)
            #fig8.savefig(path_to_save/("acceleration_all_hist"+file_name),format="pdf")
            #plt.close('all')
            #print("fig8 done")


    def add_all_labels(self,axs, list_y_label,list_x_labels):

        for ax,ylabel,xlabel in zip(np.ravel(axs),list_y_label,list_x_labels):
            ax.set_ylabel(ylabel)
            ax.set_xlabel(xlabel)
    # Function to handle key presses
    def initiate_time_constant_graph(self,ax,predictions,cmd_of_interest_reshape,gt_of_interest_reshpae,line_label = ["Model","GT","CMD"],legend=False): 
        line, = ax.plot([], [], lw=2,label =line_label[0])
        line2, = ax.plot([],[],label = line_label[1])
        line3, = ax.plot([],[],label=line_label[2])
        
        # Set up the plot limitsc
        ax.set_xlim(-0.5, 6.5)
        
        
        
        y_lim_min = np.min(np.array([np.min(predictions),np.min(cmd_of_interest_reshape),np.min(gt_of_interest_reshpae)])) 
        y_lim_max = np.max(np.array([np.max(predictions),np.max(cmd_of_interest_reshape),np.max(gt_of_interest_reshpae)])) 
        ax.set_ylim(y_lim_min, y_lim_max)
        ax.set_xlim(-0.5, 6.5)
        ax.vlines(np.array([2,4,6]),y_lim_min,y_lim_max)

        line4 = ax.vlines(0,y_lim_min,y_lim_max,colors="red",label="time constant")
        line5 = ax.vlines(0,y_lim_min,y_lim_max,colors="red",linestyle="--",label="time delay")

        if legend:
            ax.legend(ncol=5, loc='center', bbox_to_anchor=(1.1, 1.1))

        
        return line5, line4,line,line2,line3
    
    def initiate_traj_graph(self,ax,list_traj,quiver_label = ["filtered","unfiltered"]):

        list_quiver = []
        i_traj = 0
        n_traj_cols = len(list_traj)
        n_col_traj = 3
        n_traj =  int(n_traj_cols//n_col_traj)

        list_traj_new = [] 
        shape = list_traj[0].shape[1]

        
        cmap = cm.get_cmap("Blues")
        cmap2 = cm.get_cmap("Reds")

        list_color = [cmap([i for i in range(shape)]),cmap2([i for i in range(shape)])]#cmap([i for i in range(n_traj)])

        for i in range(n_traj):
            label = list_traj[i]
            x,y,yaws = list_traj[i*n_col_traj:(i+1)*n_col_traj]
            
            quiver_x_y = [np.cos(yaws),np.sin(yaws)]
            #quiver_x_y = np.array([[np.cos(yaw), np.sin(yaw)]for yaw in yaws][0] )
            zero_like_ = np.zeros(x[0,:].shape)
            quiv = ax.quiver(zero_like_, zero_like_,zero_like_,zero_like_, label =quiver_label[i], color=list_color[i],animated=True)
            
            list_quiver.append(quiv)
            
            list_traj_new.append([x,y,yaws,quiver_x_y])
        ax.legend(loc='lower center', bbox_to_anchor=(1, 1))  # Place it outside on the upper left
        return list_traj_new, list_quiver
    
    def load_data(self, df,df_diamond,list_columns):
        """_summary_

        Args:
            df (_type_): _description_
            df_diamond (_type_): _description_
            list_columns (_type_): _description_

        Returns:
            _type_: _description_
        """

        list_dataframe = []

        df_unique_col = print_column_unique_column(df,verbose=False)
        df_diamond_col = print_column_unique_column(df_diamond,verbose=False)
        
        for graph_cols in list_columns:


            temp_list = []
            for col in graph_cols:
                if col in df_unique_col:

                    data = reshape_into_6sec_windows(column_type_extractor(df,col))
                    #print(col,data.shape)
                    #print(np.unique(data[0,:]))
                    temp_list.append(data)
                elif col in df_diamond_col:
                    temp_list.append(column_type_extractor(df_diamond,col))
            list_dataframe.append(temp_list)
        
        return list_dataframe
    
    def initiate_all_figure(self,axs, list_data,list_graph_type):

        list_scatter_or_quiver = []
        j = 0
        for ax , graph_data,graph_type in zip(np.ravel(axs),list_data,list_graph_type):
            
            if graph_type == "time_constant":
                if j==0:
                    legend = True
                else:
                    legend=False
                lines = self.initiate_time_constant_graph(ax,graph_data[0],graph_data[1],graph_data[2],line_label = ["Model","GT","CMD"],legend=legend)
                list_scatter_or_quiver.append(lines)
            elif graph_type == "traj":
                graph_data,list_quiver = self.initiate_traj_graph(ax,graph_data,quiver_label = ["filtered","unfiltered"])
                list_scatter_or_quiver.append(list_quiver)
                list_data[j] = graph_data
            else:
                raise ValueError("Wrong type of graph")
            j +=1 
        
        return list_scatter_or_quiver, list_data
    

    def update_time_constant(self,anim_i,data_columns, lines):
        i = 0
        for data, line in zip(data_columns,lines):
            if i <= 1:
                new_vline_x = data[anim_i]

                segment = []
                for lin, new_x_pos in zip(line.get_segments(), new_vline_x):
                    lin[0][0] = new_x_pos  # Update start x position
                    lin[1][0] = new_x_pos
                    segment.append(lin)
                
                line.set_segments(segment)
                
            else:
                line.set_data(self.time_axis, data[anim_i,:])
            i+=1 
            #print(data.shape) 
            
        return lines
    def set_y_scale(self,anim_i,ax,x,y):
        min = np.min(np.array([np.min(y[anim_i,:]),np.min(x[anim_i,:]) ]))
        max = np.max(np.array([np.max(y[anim_i,:]),np.max(x[anim_i,:]) ]))
        scale = 1.10
        lim_y_min = np.min(y[anim_i,:]) * scale
        lim_y_max = np.max(y[anim_i,:]) * scale
        delta_y = lim_y_max - lim_y_min

        lim_x_min = np.min(x[anim_i,:]) * scale
        lim_x_max = np.max(x[anim_i,:]) * scale
        delta_x = lim_x_max - lim_x_min                
        
        
        ax.set_xlim(lim_x_min, lim_x_max)
        
        ax.set_ylim(lim_y_min,lim_y_max)

        ax.set_aspect('equal') # ,adjustable= 'datalim'
        
        
    def update_traj_quiver(self,anim_i,data_columns,list_scat, ax):
        
        nb_traj = len(data_columns) # 2 pour 2 traj

        list_x = []
        list_y = []
        for traj,quiver_lines in zip(data_columns,list_scat):
               
            x,y,yaws,quiver_u_v =  traj 
            list_x.append(x)
            list_y.append(y)
            array_like = (np.vstack((x[anim_i,:],y[anim_i,:]))).T
            quiver_lines.set_offsets(array_like)
            #quiver_lines.set(color=color)

            
            quiver_lines.set_UVC(quiver_u_v[0][anim_i,:],quiver_u_v[1][anim_i,:])

        self.set_y_scale(anim_i,ax,np.concat(list_x),np.concat(list_y))
        return list_scat
    
    def update_time_constant_traj_dashboard(self,i_frame,list_axs, list_data,list_scatter_or_quiver,list_graph_style,fig):
        lines_to_return = []
        for ax, data_columns, lines, graph_style in zip (list_axs, list_data,list_scatter_or_quiver,list_graph_style):

            if graph_style == "time_constant":

                lines_to_return += self.update_time_constant(i_frame,data_columns, lines)

            elif graph_style == "traj":
                lines_to_return += self.update_traj_quiver(i_frame,data_columns,lines,ax)
            else: 
                raise ValueError("wrong graph style") 
        fig.suptitle(f"Current step :{i_frame}")

        return lines_to_return

    def on_key(self,event,frame_index,list_axs, list_data,list_scatter_or_quiver,list_graph_style,fig,max_frames):
        """Handle key press events."""
        
        if event.key == 'space':
            frame_index = (frame_index + 1) % max_frames  # Cycle through frames
            self.update_time_constant_traj_dashboard(frame_index,list_axs, list_data,list_scatter_or_quiver,list_graph_style,fig)
            plt.draw()  # Redraw the current figure
    
    def produce_video_time_constants(self, video_saving_path="",live_observation=False): 
        
        # Initialize data
        data = {'x': [], 'y': []}

        # Create a figure and axis
        n_rows = 3
        n_col = 2 
        scale_of_each_graph = 3*1.10
        fig, axs = plt.subplots(n_rows,n_col)
        fig.set_figheight(n_rows* scale_of_each_graph)
        fig.set_figwidth(n_col * scale_of_each_graph)
        plt.subplots_adjust(wspace=0.25, hspace=0.25)

        # List labels 
        list_y_label = ["Left wheel speed [rad/s]", "Right wheel speed [rad/s]", "Vx speed [m/s]", "Vyaw [rad/s]","Vy speed [m/s]", "y position [m]"]
        list_x_label = ["Time [s]"]*5 + ["Position X [m]"]
        
        self.add_all_labels(axs, list_y_label,list_x_label)
        
        graph_style = ["time_constant"]*5+["traj"]

        #print_column_unique_column(self.df_slip)
        list_columns = [["left_wheel_vel_time_delay","left_wheel_vel_time_for_95_percent_ss_value","left_wheel_vel_predictions","left_wheel_vel","cmd_left"],
            ["right_wheel_vel_time_delay","right_wheel_vel_time_for_95_percent_ss_value","right_wheel_vel_predictions","right_wheel_vel","cmd_right"],
            ["step_frame_vx_time_delay",'step_frame_vx_time_for_95_percent_ss_value',"step_frame_vx_predictions","step_frame_vx","cmd_body_vel_x"], # cmd_body_vel_x
            ["step_frame_vyaw_time_delay","step_frame_vyaw_time_for_95_percent_ss_value","step_frame_vyaw_predictions","step_frame_vyaw","cmd_body_vel_yaw"], #  cmd_body_vel_yaw
            ["step_frame_vy_time_delay","step_frame_vy_time_for_95_percent_ss_value","step_frame_vy_predictions","step_frame_vy","cmd_body_vel_y"],
            ["step_frame_interpolated_icp_x","step_frame_interpolated_icp_y","step_frame_interpolated_icp_yaw",
                "step_frame_icp_x","step_frame_icp_y","step_frame_icp_yaw"] # If i Want to use the imu, I can just use the imu_yaw col.
            ]
        
        list_data_array = self.load_data(self.df_slip,self.df_diamond,list_columns)
        
        
        list_scatter_or_quiver, list_data = self.initiate_all_figure(axs, list_data_array,graph_style)
        # (self,i_frame,                                                                    list_axs, list_data,list_scatter_or_quiver,list_graph_style):

        if live_observation == False:
            ani = animation.FuncAnimation(fig, self.update_time_constant_traj_dashboard,fargs=[np.ravel(axs),list_data,list_scatter_or_quiver,graph_style,fig], frames=self.step_shape[0], interval=1000, blit=True)
            print("saving_video")
            # Save the animation as a video
            final_path_2_save_video = f'dashboard_step_by_step_visualisation.mp4'

            if video_saving_path!="":
                path_2_save_video = video_saving_path/final_path_2_save_video

            
            ani.save(path_2_save_video, writer='ffmpeg')

        else:
            # Connect the key press event to the on_key function
            frame_index = 0
            fig.canvas.mpl_connect('key_press_event', lambda event: self.on_key(event, frame_index,np.ravel(axs),list_data,list_scatter_or_quiver,graph_style,fig,self.step_shape[0]))
            plt.show()

    def produce_slip_histogramme_by_roboticist_by_robot(self,robiticis_specific=True,densitybool=True):

        df_diamond = self.df_diamond

        list_df_to_use = []
        list_title = [] 
        list_file_name = []

        list_robot  = []

        fig_prefix= "histogram_slip"

        path_to_save = self.path_to_analysis/"heatmap_slip"/"by_robot"

        if path_to_save.is_dir() == False:
            path_to_save.mkdir(parents=True, exist_ok=True)

        

        for robot in df_diamond["robot"].unique():
            df_sampling_speed = df_diamond.loc[df_diamond["robot"]==robot]

            if robiticis_specific:
                for roboticist in df_sampling_speed["roboticist"].unique():
                
                    df_sampling_speed_roboticist = df_sampling_speed.loc[df_sampling_speed["roboticist"] == roboticist]
                    list_df_to_use.append(df_sampling_speed_roboticist)
                    list_title.append(f"Roboticist {roboticist} with the robot {robot} m/s")
                    list_file_name.append(f"{fig_prefix}_robot_{robot}_{roboticist}.pdf")
                    list_robot.append(robot)
            else:
                list_df_to_use.append(df_sampling_speed)
                list_title.append(f"All roboticist with the {robot} m/s")
                list_file_name.append(f"{fig_prefix}_robot_{robot}_all_roboticist.pdf")
                list_robot.append(robot)
        # Produce the graphs 
        for title, df,file_name,robot_name in tqdm(zip(list_title,list_df_to_use,list_file_name,list_robot),colour="green"):

            

            robot_param= self.param_robot[robot_name]

            robot_param_list = [robot_param["basewidth"],robot_param["wheel_radius"],
                        robot_param["maximum_linear_speed"],robot_param["maximum_angular_speed"],
                        robot_param["maximum_wheel_speed_empty"]]
            
            fig = self.plot_diamond_graph_slip_heat_map(df,global_cmap = True,subtitle=title,
                                                        y_lim=robot_param["maximum_linear_speed"],
                                                        x_lim=robot_param["maximum_angular_speed"])
            fig.savefig(path_to_save/("body_slip_"+file_name),format="pdf")
            plt.close('all')
            print("fig1 done")
            fig2 = self.plot_diamond_graph_wheel_slip_heat_map(df,subtitle=title,global_cmap = True,
                                                            y_lim=robot_param["maximum_wheel_speed_empty"],
                                                            x_lim=robot_param["maximum_wheel_speed_empty"])
            fig2.savefig(path_to_save/("wheel_slip_"+file_name),format="pdf")
            plt.close('all')
            print("fig2 done")

            print(robot_name,robot_param)
            fig3 = self.scatter_diamond_displacement_graph(df,subtitle=title, y_lim=robot_param["maximum_linear_speed"],
                                                        x_lim=robot_param["maximum_angular_speed"],max_wheel_speed=robot_param["maximum_wheel_speed_empty"],
                                                        robot = robot_param_list)
            fig3.savefig(path_to_save/("displacement_diamond_"+file_name),format="pdf")
            plt.close('all')
            print("fig3 done")

            #fig4 = self.scatter_diamond_displacement_graph_diff(df,subtitle=title)
            #fig4.savefig(path_to_save/("diff_displacement_diamond_"+file_name),format="pdf")
            #plt.close('all')
            #fig5 = self.plot_diamond_graph_slip_heat_map(df,global_cmap = True,subtitle=title,diff_referential=True)
            #fig5.savefig(path_to_save/("diff_frame_body_slip_"+file_name),format="pdf")
            #plt.close('all')
            #fig6 = self.plot_diamond_graph_wheel_slip_heat_map(df,subtitle=title,global_cmap = True,diff_referential=True)
            #fig6.savefig(path_to_save/("diff_frame_wheel_slip_"+file_name),format="pdf")
            #plt.close('all')

            fig7 = self.acceleration_histogram(df,subtitle="transient",nb_bins=30,x_lim=(-6,6),densitybool=densitybool,transientflag=True)
            fig7.savefig(path_to_save/("acceleration_transient_hist"+file_name),format="pdf")
            plt.close('all')
            print("fig7 done")
            #fig8 = self.acceleration_histogram(df,subtitle="all",nb_bins=30,x_lim=(-6,6),densitybool=densitybool,transientflag=False)
            #fig8.savefig(path_to_save/("acceleration_all_hist"+file_name),format="pdf")
            #plt.close('all')
            #print("fig8 done")


def plot_all_unfiltered_data():
    path_to_dataframe_slip = "drive_datasets/results_multiple_terrain_dataframe/all_terrain_slip_dataset.pkl" 
    path_to_dataframe_diamond= "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"
    path_to_config_file=""

    graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,path_to_dataframe_diamond,path_to_config_file="")
    graphic_designer.produce_slip_histogramme_by_roboticist_by_robot(robiticis_specific=True)
    graphic_designer.produce_slip_histogramme_by_roboticist_by_robot(robiticis_specific=False)


def plot_robot_filtered_results(robot):

    ### TODO add prefiltered_prefix_to_results. 
    path_to_dataframe_slip =   f"drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_{robot}_following_robot_param_all_terrain_slip_dataset.pkl" 
    path_to_dataframe_diamond= f"drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_{robot}_following_robot_param_all_terrain_steady_state_dataset.pkl"
    path_to_config_file=""

    graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,path_to_dataframe_diamond,path_to_config_file="",result_folder_prefix="filtered_warthog_results")
    graphic_designer.produce_slip_histogramme_by_roboticist_by_robot(robiticis_specific=False)
    graphic_designer.produce_slip_histogramme_by_roboticist_by_robot(robiticis_specific=True)
    
if __name__ == "__main__":


    #path_2_training_folder = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets")

    #path_slip_df = path_2_training_folder/"slip_dataset_all.pkl"
    #path_steady_state_df = path_2_training_folder/"steady_state_results.pkl"
    
    #graphic_producer = GraphicProductionDrive(path_to_dataframe_diamond=path_steady_state_df,path_to_dataframe_slip=path_slip_df)

    #path = path_2_training_folder/"video"
    #graphic_producer.produce_video_time_constants(video_saving_path=path,live_observation=False)
    
    #path_to_dataframe_slip = "" 
    #dataframe_type = ""
    #path_to_dataframe_diamond= "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/steady_state_results.pkl"
    #path_to_config_file=""
#
    #graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,dataframe_type,path_to_dataframe_diamond,path_to_config_file="")
    #fig = graphic_designer.plot_diamond_graph_slip_heat_map(global_cmap=True)
    #plt.show()
    #
    
    plot_all_unfiltered_data()
    plot_robot_filtered_results("warthog")
    plot_robot_filtered_results("husky")
    #fig = graphic_designer.plot_diamond_graph_slip_heat_map(graphic_designer.df_diamond,diff_referential=True)

    #fig3 = graphic_designer.scatter_diamond_displacement_graph_diff(graphic_designer.df_diamond,subtitle="")

    #fig3.savefig(path_to_save/("displacement_diamond_"+file_name),format="pdf")


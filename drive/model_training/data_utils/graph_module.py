
import pandas as pd
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib as mpl
import pathlib


import matplotlib.animation as animation
from matplotlib import colormaps as cm
from extractors import *
class GraphicProductionDrive():

    def __init__(self,path_to_dataframe_slip,path_to_dataframe_diamond,path_to_config_file="",rate=20):
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

        self.path_to_analysis = path_to_dataframe_slip.parent/"analysis"

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

    def create_window_filter_axis(self):
        increment = 1/(self.n_iteration_by_windows-1)

    

    def scatter_diamond_displacement_graph(self,df_all_terrain,subtitle=""):
        
        color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew"}
        list_terrain = df_all_terrain["terrain"].unique()
        size = len(list_terrain)
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(3*3)
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        alpha_parama= 0.3
        y_lim = 6
        x_lim = 8.5
        
        
            
        
        for i in range(size):  
            if size == 1:
                ax_to_plot = axs[0]
                ax_to_plot_2 = axs[1]
            else:
                ax_to_plot = axs[0,i]
                ax_to_plot_2 = axs[1,i]
            
            terrain = list_terrain[i]
            df = df_all_terrain.loc[df_all_terrain["terrain"]==terrain]   

            ax_to_plot.set_title(f"Body vel on {terrain}\n ") # (ICP smooth by spline,yaw=imu)     
            ax_to_plot.scatter(df["cmd_body_yaw_lwmean"],df["cmd_body_x_lwmean"],color = "orange",label='Command',alpha=alpha_parama)
            ax_to_plot.scatter(df["icp_vel_yaw_smoothed"],df["icp_vel_x_smoothed"],color = "blue",label='Mean of body steady-state speed',alpha=alpha_parama) 
            ax_to_plot.set_xlabel("Angular velocity (omega) [rad/s]")
            ax_to_plot.set_ylabel("Forward velocity (V_x) [m/s]")

            ax_to_plot.set_ylim((-y_lim,y_lim))
            ax_to_plot.set_xlim((-x_lim,x_lim))
            #back_ground_color = df.color .unique()
            ax_to_plot.set_facecolor(color_dict[terrain])
            
            ax_to_plot_2.scatter(df["cmd_right_wheels"],df["cmd_left_wheels"],color="orange",alpha=alpha_parama)
            ax_to_plot_2.scatter(df["odom_speed_right_wheels"],df["odom_speed_left_wheels"],label='Mean of wheel steady-state speed',color="green",alpha=alpha_parama)
            ax_to_plot_2.set_ylabel("left_wheel speed [rad/s]")
            ax_to_plot_2.set_xlabel("right wheel speed [rad/s]")
            
            

            
            #axs[0][1].set_title("Command VS Body vel \n (ICP derivate)")
            ax_to_plot_2.set_title(f"Wheels vel on {terrain}")

            
            wheels_value = 20
            ax_to_plot_2.set_ylim((-wheels_value,wheels_value))
            ax_to_plot_2.set_xlim((-wheels_value,wheels_value))
            ax_to_plot_2.set_aspect(1)
            
            ax_to_plot_2.set_facecolor(color_dict[terrain])

            if i ==0 :
                
                handles = ax_to_plot.get_legend_handles_labels()[0] + ax_to_plot_2.get_legend_handles_labels()[0]
                legends = ax_to_plot.get_legend_handles_labels()[1] + ax_to_plot_2.get_legend_handles_labels()[1]
                
                fig.legend(handles,legends, loc='center', bbox_to_anchor=(0.5, 0.45), ncol=3)

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


        
    def scatter_plot_heat_map(self,df,column_x_y_z, ax_to_plot, cmap_name,background_terrain_dict,ylim,xlim,global_cmap,labels_xyz,alpha,show_x_label=False):

        norm_slip = mpl.colors.Normalize(0, vmax=np.abs(global_cmap[1][column_x_y_z[2]].max()))
        norm_slip = mpl.colors.Normalize(0, vmax=2)
    
        if not global_cmap[0]:
            scatter_ax = ax_to_plot.scatter(df[column_x_y_z[0]],df[column_x_y_z[1]],c = np.abs(df[column_x_y_z[2]]),cmap=cmap_name,alpha=alpha)
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

    def plot_diamond_graph_slip_heat_map(self,df_all_terrain,subtitle="",global_cmap = True):

        
        color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew"}
        #print_column_unique_column(df_all_terrain)
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)
        fig, axs = plt.subplots(3,size)
        plt.subplots_adjust(wspace=0.25, hspace=0.25)
        fig.set_figwidth(3.5*size)
        fig.set_figheight(3*3)
        alpha_parama= 0.3

        y_lim = 6 # m/s
        x_lim = 8.5 #m/s
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
            show_x_label = False
            #### Slip x 
            
            if size == 1:
                ax_to_plot = axs[0]
            else:
                ax_to_plot = axs[0,i]
            
            labels_xyz[2] = r"$\textbf{Slip x [m/s]}$"
            self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            ax_to_plot.set_title(f"Steady-state slip for {terrain}") # (ICP smooth by spline,yaw=imu)
            ##### Slip y 
            
            column_x_y_z[2] = "slip_body_y_ss"

            if size == 1:
                ax_to_plot = axs[1]
            else:
                ax_to_plot = axs[1,i]
            labels_xyz[2] = r"$\textbf{Slip y [m/s]}$"
            ######### slip y  ####################
            self.scatter_plot_heat_map(df,column_x_y_z, ax_to_plot, "inferno",color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
            ### Slip yaw 
            column_x_y_z[2] = "slip_body_yaw_ss"
            show_x_label = True
            if size == 1:
                ax_to_plot = axs[2]
            else:
                ax_to_plot = axs[2,i]
            ######### slip y  ####################
            labels_xyz[2] = r"$\textbf{Slip yaw [rad/s]}$"
            self.scatter_plot_heat_map(df,column_x_y_z, ax_to_plot, "inferno",color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
            
            
        
        
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
    
    def plot_diamond_graph_wheel_slip_heat_map(self,df_all_terrain,subtitle="",global_cmap = True):


        color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew"}
        #print_column_unique_column(df_all_terrain)
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(2*3)
        alpha_parama= 0.3

        y_lim = 20 # m/s
        x_lim = 20 #m/s
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
            
            labels_xyz  = ["Left wheel angular velocity [rad/s]" , "Right wheel angular velocity [rad/s]",""] 
            column_x_y_z = ["cmd_left_wheels","cmd_right_wheels", "slip_wheel_left_ss"]
            show_x_label = False
            #### Slip x 
            if size == 1:
                ax_to_plot = axs[0]
            else:
                ax_to_plot = axs[0,i]
            
            labels_xyz[2] = r"$\textbf{Slip left wheel [m/s]}$"
            self.scatter_plot_heat_map(df, column_x_y_z, ax_to_plot, "inferno", color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            ax_to_plot.set_title(f"Steady-state slip for {terrain}") # (ICP smooth by spline,yaw=imu)
            ##### Slip y 
            
            column_x_y_z[2] = "slip_wheel_right_ss"

            if size == 1:
                ax_to_plot = axs[1]
            else:
                ax_to_plot = axs[1,i]
            labels_xyz[2] = r"$\textbf{Slip right wheel [m/s]}$"
            ######### slip y  ####################
            self.scatter_plot_heat_map(df,column_x_y_z, ax_to_plot, "inferno",color_dict[terrain],y_lim,x_lim,global_cmap,labels_xyz,alpha_scatter,show_x_label)
            
            
            
        
        #axs[0][-1].legend(lines, legends_labels, loc = 'center', bbox_to_anchor = (0, -0.55, 1, 1),
        #            bbox_transform = plt.gcf().transFigure,ncol=3)
        if subtitle=="":
            fig.suptitle(f"Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        fig.tight_layout()

        print('Note that the yaw angle in the the smooth version is the IMU')

        return fig
    def produce_slip_histogramme_by_roboticist_for_a_specific_linear_sampling_speed(self,robiticis_specific=True):

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
        for title, df,file_name in zip(list_title,list_df_to_use,list_file_name):
            fig = self.plot_diamond_graph_slip_heat_map(df,global_cmap = True,subtitle=title)
            fig.savefig(path_to_save/("body_slip_"+file_name),format="pdf")

            fig2 = self.plot_diamond_graph_wheel_slip_heat_map(df,subtitle=title,global_cmap = True)
            fig2.savefig(path_to_save/("wheel_slip_"+file_name),format="pdf")

            fig3 = self.scatter_diamond_displacement_graph(df,subtitle=title)
            fig3.savefig(path_to_save/("displacement_diamond_"+file_name),format="pdf")

            plt.close('all')



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
        ax.set_xlim(0, 6.2)
        
        
        
        y_lim_min = np.min(np.array([np.min(predictions),np.min(cmd_of_interest_reshape),np.min(gt_of_interest_reshpae)])) 
        y_lim_max = np.max(np.array([np.max(predictions),np.max(cmd_of_interest_reshape),np.max(gt_of_interest_reshpae)])) 
        ax.set_ylim(y_lim_min, y_lim_max)
        ax.set_xlim(0, 6)
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
        list_columns = [["left_wheel_vel_time_delay","left_wheel_vel_time_constants_to_show","left_wheel_vel_predictions","left_wheel_vel","cmd_left"],
            ["right_wheel_vel_time_delay","right_wheel_vel_time_constants_to_show","right_wheel_vel_predictions","right_wheel_vel","cmd_right"],
            ["step_frame_vx_time_delay",'step_frame_vx_time_constants_to_show',"step_frame_vx_predictions","step_frame_vx","cmd_body_vel_x"], # cmd_body_vel_x
            ["step_frame_vyaw_time_delay","step_frame_vyaw_time_constants_to_show","step_frame_vyaw_predictions","step_frame_vyaw","cmd_body_vel_yaw"], #  cmd_body_vel_yaw
            ["step_frame_vy_time_delay","step_frame_vy_time_constants_to_show","step_frame_vy_predictions","step_frame_vy","cmd_body_vel_y"],
            ["step_frame_interpolated_icp_x","step_frame_interpolated_icp_y","step_frame_interpolated_icp_yaw",
                "step_frame_icp_x","step_frame_icp_y","step_frame_icp_yaw"] # If i Want to use the imu, I can just use the imu_yaw col.
            ]
        
        list_data_array = self.load_data(self.df_slip,self.df_diamond,list_columns)
        
        
        list_scatter_or_quiver, list_data = self.initiate_all_figure(axs, list_data_array,graph_style)
        # (self,i_frame,                                                                    list_axs, list_data,list_scatter_or_quiver,list_graph_style):

        if live_observation == False:
            ani = animation.FuncAnimation(fig, self.update_time_constant_traj_dashboard,fargs=[np.ravel(axs),list_data,list_scatter_or_quiver,graph_style,fig], frames=self.step_shape[0], interval=1000, blit=True)

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
#
    path_to_dataframe_slip = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/results_multiple_terrain_dataframe/all_terrain_slip_dataset.pkl" 
    path_to_dataframe_diamond= "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"
    path_to_config_file=""

    
    graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,path_to_dataframe_diamond,path_to_config_file="")
    fig = graphic_designer.plot_diamond_graph_slip_heat_map(global_cmap=True)
    plt.show()

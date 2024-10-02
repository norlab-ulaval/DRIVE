
import pandas as pd
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib as mpl

class GraphicProductionDrive():

    def __init__(self,path_to_dataframe_slip,dataframe_type,path_to_dataframe_diamond,path_to_config_file=""):
        """open the dataframe

        Args:
            path_to_dataframe (_type_): path_to_dataframe
            dataframe_type (_type_): model_training_datasets [slip_dataset_all, torch_ready_dataset]
        """
        
        #self.df_slip = pd.read_pickle(path_to_dataframe_slip)
        
        self.df_diamond = pd.read_pickle(path_to_dataframe_diamond)


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






    def plot_diamond_graph_slip_heat_map(self,global_cmap = True):

        df_all_terrain = self.df_diamond

        color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew"}
        list_terrain = list(df_all_terrain["terrain"].unique())
        size = len(list_terrain)
        fig, axs = plt.subplots(3,size)
        
        fig.set_figwidth(3*size)
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
            column_x_y_z = ["cmd_body_yaw","cmd_body_x", "slip_body_x_ss"]
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
        fig.suptitle(f"Absolute Steady-state slip (x,y,yaw) for all_types_of_terrain",fontsize=16)

        #fig.patch.set_facecolor(color_background)
        
        fig.tight_layout()

        print('Note that the yaw angle in the the smooth version is the IMU')

        return fig


if __name__ == "__main__":

    path_to_dataframe_slip = "" 
    dataframe_type = ""
    path_to_dataframe_diamond= "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h9s5/model_training_datasets/steady_state_results.pkl"
    path_to_config_file=""

    graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,dataframe_type,path_to_dataframe_diamond,path_to_config_file="")
    fig = graphic_designer.plot_diamond_graph_slip_heat_map(global_cmap=True)
    plt.show()
    

    path_to_dataframe_slip = "" 
    dataframe_type = ""
    path_to_dataframe_diamond= "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/results_multiple_terrain_dataframe/all_terrain_diamond_shape_graph_data.pkl"
    path_to_config_file=""

    
    graphic_designer = GraphicProductionDrive(path_to_dataframe_slip,dataframe_type,path_to_dataframe_diamond,path_to_config_file="")
    fig = graphic_designer.plot_diamond_graph_slip_heat_map(global_cmap=True)
    plt.show()

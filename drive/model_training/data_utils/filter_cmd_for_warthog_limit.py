import shapely
import numpy as np 
from shapely.geometry import Polygon, Point
from shapely import intersection
from drive.model_training.data_utils.extractors import *
import matplotlib.pyplot as plt
import pathlib
import pickle 
import yaml


PATH_ROBOT_CONFIG_FILE = "drive/model_training/data_utils/robot_param.yaml"
PATH_TO_SAVED_CONSTRAINTS = "drive_datasets/results_multiple_terrain_dataframe/filtered_warthog_resultsanalysis/"


def scatter_diamond_displacement_graph(df_all_terrain,list_shape,subtitle=""):
        
        list_terrain = df_all_terrain["terrain"].unique()
        size = len(list_terrain)
        fig, axs = plt.subplots(2,size)
        
        fig.set_figwidth(3*size)
        fig.set_figheight(3*3)
        plt.subplots_adjust(wspace=0.5, hspace=0.5)
        alpha_parama= 0.3
        y_lim = 6
        x_lim = 8.5

        color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew","tile":"mistyrose","boreal":"lightgray",
                      "sand":"lemonchiffon","avide":"white","avide2":"white","mud":"white"}

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
            ax_to_plot.set_facecolor(color_dict[terrain])
        
            ax_to_plot_2.scatter(df["cmd_right_wheels"],df["cmd_left_wheels"],color="orange",alpha=alpha_parama)
            ax_to_plot_2.scatter(df["odom_speed_right_wheels"],df["odom_speed_left_wheels"],label='Mean of wheel steady-state speed',color="green",alpha=alpha_parama)
            #axs[0][1].set_title("Command VS Body vel \n (ICP derivate)")
            ax_to_plot_2.set_title(f"Wheels vel on {terrain}")


            for geom in list_shape:
                x, y = geom.exterior.xy
                ax_to_plot_2.plot(x,y)

            
            ax_to_plot_2.set_facecolor(color_dict[terrain])

        ax_to_plot.set_xlabel("Angular velocity (omega) [rad/s]")
        ax_to_plot.set_ylabel("Forward velocity (V_x) [m/s]")
        ax_to_plot.set_ylim((-y_lim,y_lim))
        ax_to_plot.set_xlim((-x_lim,x_lim))
        #back_ground_color = df.color .unique()

        ax_to_plot_2.set_ylabel("left_wheel speed [rad/s]")
        ax_to_plot_2.set_xlabel("right wheel speed [rad/s]")

        wheels_value = 25
        ax_to_plot_2.set_ylim((-wheels_value,wheels_value))
        ax_to_plot_2.set_xlim((-wheels_value,wheels_value))
        ax_to_plot_2.set_aspect(1)
        

        if i ==0 :
            
            handles = ax_to_plot.get_legend_handles_labels()[0] + ax_to_plot_2.get_legend_handles_labels()[0] 
            legends = ax_to_plot.get_legend_handles_labels()[1] + ax_to_plot_2.get_legend_handles_labels()[1] 
            
            
        
        
        if subtitle=="":
            fig.suptitle(f"Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        else:
            fig.suptitle(subtitle + f"\n Cmd vs steady-state results for all_types_of_terrain",fontsize=14)
        #fig.patch.set_facecolor(color_background)
        
        #fig.patch.set_facecolor(color_background)
        #plt.tight_layout()
        
        return fig 


def reverse_engineer_clearpath_max_speed(df,robot_param,filter_data=True,debug=False):
    

    
    
    
    min_ang_speed_limmit = robot_param["maximum_angular_speed"] #np.max(df.max_ang_speed_sampled)#robot_param["maximum_angular_speed"] # The low-level_limit is constant # min(list(df['max_ang_speed_sampled'].unique())) ### Assume that the df was already prefiltered for the max lin speed
    min_lin_speed_limmit = robot_param["maximum_linear_speed"]#np.max(df.max_linear_speed_sampled)#robot_param["maximum_linear_speed"]  #The low-level_limit is constant min(list(df['max_linear_speed_sampled'].unique()))
    #b = robot_param["basewidth"]
    #r = robot_param["wheel_radius"] 
    #jacobians = np.array([[1/2,1/2],[-1/b, 1/b]]) * r
    #inv_jac = np.linalg.inv(jacobians)

    
    #if np.max(np.abs(df["cmd_body_x_lwmean"])) <= min_lin_speed_limmit:

    #    min_lin_speed_limmit = np.max(np.abs(df["cmd_body_x_lwmean"]))
    # Define the coordinates of the polygon
    #max_body_slip  = np.array([(-min_lin_speed_limmit,-min_ang_speed_limmit,), 
    #                  ( min_lin_speed_limmit,-min_ang_speed_limmit,), 
    #                  ( min_lin_speed_limmit,min_ang_speed_limmit,), 
    #                  (-min_lin_speed_limmit, min_ang_speed_limmit),
    #                  (-min_lin_speed_limmit, -min_ang_speed_limmit)]) # A square
    
    #max_body_in_wheel_constraints = (inv_jac @ max_body_slip.T)

    # Create the polygon
    #rectangle = Polygon(zip(max_body_in_wheel_constraints[1,:],max_body_in_wheel_constraints[0,:]))

    #min_ang_speed_limmit = dict_terrain["min_ang_speed_limmit"] #5.0 # The low-level_limit is constant # min(list(df['max_ang_speed_sampled'].unique())) ### Assume that the df was already prefiltered for the max lin speed
    #min_lin_speed_limmit =  dict_terrain["min_lin_speed_limmit"] #5.0  #The low-level_limit is constant min(list(df['max_linear_speed_sampled'].unique()))
    
    # Define the coordinates of the polygon
    max_body_slip  = np.array([(-min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,min_ang_speed_limmit,), 
                      (-min_lin_speed_limmit, min_ang_speed_limmit),
                      (-min_lin_speed_limmit, -min_ang_speed_limmit)]).T # A square
    
    b = robot_param["basewidth"]
    r = robot_param["wheel_radius"] 
    jacobians = np.array([[1/2,1/2],[-1/b, 1/b]]) * r

    max_body_in_wheel_constraints = (np.linalg.inv(jacobians) @ max_body_slip)
    

    
    ### Assuming that wheel reaches their Steady state velocity after 2 s
    left_wheel = column_type_extractor(df, "cmd_left")
    right_wheel = column_type_extractor(df, "cmd_right")
    
    max_wheel_speed_cmd = robot_param["maximum_wheel_speed_empty"]#max([np.max(np.abs(left_wheel)),np.max(np.abs(right_wheel))])
    
    max_wheel_coordinates = np.array([(-max_wheel_speed_cmd,-max_wheel_speed_cmd), (-max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, -max_wheel_speed_cmd)]).T  #    

    ##### 
    
    
    # Create the polygon
    
    rectangle = Polygon(zip(max_body_in_wheel_constraints[1,:],max_body_in_wheel_constraints[0,:]))

    rectangle_2 = Polygon(zip(max_wheel_coordinates[1,:],max_wheel_coordinates[0,:]))
    
    union_res = rectangle.intersection(rectangle_2)


    
    cmd_body_lin =  np.mean(column_type_extractor(df,"cmd_left_wheels"),axis=1)
    cmd_body_yaw = np.mean(column_type_extractor(df,"cmd_right_wheels"),axis=1)

    cmd = np.array([cmd_body_yaw,cmd_body_lin]).T

    filter = []
    for i in range(cmd.shape[0]):

        pt = Point(cmd[i,:])
        if filter_data:
            if shapely.within(pt, union_res) or shapely.touches(pt,union_res):
                filter.append(True)
            else:
                filter.append(False)
        else:
            filter.append(True)

    
    df["is_within_software_limits"] = filter
    
    new_df = df.loc[filter]    

    if debug:
        scatter_diamond_displacement_graph(df,[union_res],subtitle="")
        scatter_diamond_displacement_graph(new_df,[union_res],subtitle="")
        plt.show()

    print(new_df)
    return new_df,min_ang_speed_limmit,min_lin_speed_limmit


def generate_body_frame_domain_polygon(df,dict_terrain,robot_param,debug=False):
    """ATTENTION: YOU NEED TO PASS THE PREFILTERED DOMAIN. 

    Args:
        df (_type_): THE DF CONTAINING ONLY ONE TERRAIN
        debug (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: _description_
    """

    
    
    min_ang_speed_limmit = dict_terrain["min_ang_speed_limmit"] #5.0 # The low-level_limit is constant # min(list(df['max_ang_speed_sampled'].unique())) ### Assume that the df was already prefiltered for the max lin speed
    min_lin_speed_limmit =  dict_terrain["min_lin_speed_limmit"] #5.0  #The low-level_limit is constant min(list(df['max_linear_speed_sampled'].unique()))
    
    # Define the coordinates of the polygon
    max_body_slip  = np.array([(-min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,min_ang_speed_limmit,), 
                      (-min_lin_speed_limmit, min_ang_speed_limmit),
                      (-min_lin_speed_limmit, -min_ang_speed_limmit)]).T # A square
    
    
    
    
    ### Assuming that wheel reaches their Steady state velocity after 2 s
    left_wheel = column_type_extractor(df, "cmd_left")
    right_wheel = column_type_extractor(df, "cmd_right")
    
    max_wheel_speed_cmd = robot_param["maximum_wheel_speed_empty"]#max([np.max(np.abs(left_wheel)),np.max(np.abs(right_wheel))])
    
    max_wheel_coordinates = np.array([(-max_wheel_speed_cmd,-max_wheel_speed_cmd), (-max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, -max_wheel_speed_cmd)]).T  #    

    ##### 
    b = robot_param["basewidth"]
    r = robot_param["wheel_radius"] 
    jacobians = np.array([[1/2,1/2],[-1/b, 1/b]]) * r

    max_wheel_in_body_constraints = (jacobians @ max_wheel_coordinates)
    # Create the polygon
    
    rectangle = Polygon(zip(max_body_slip[1,:],max_body_slip[0,:]))

    rectangle_2 = Polygon(zip(max_wheel_in_body_constraints[1,:],max_wheel_in_body_constraints[0,:]))
    
    union_res = rectangle.intersection(rectangle_2)

    if debug:

        fig, axs = plt.subplots(1,1)

        x,y = union_res.exterior.xy
        axs.plot(x,y)
        plt.show()

    return union_res, min_ang_speed_limmit,min_lin_speed_limmit,max_wheel_speed_cmd


def generate_wheel_frame_domain_polygon(df,dict_terrain,robot_param,debug=False):
    """ATTENTION: YOU NEED TO PASS THE PREFILTERED DOMAIN. 

    Args:
        df (_type_): THE DF CONTAINING ONLY ONE TERRAIN
        debug (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: _description_
    """

    
    
    min_ang_speed_limmit = dict_terrain["min_ang_speed_limmit"] #5.0 # The low-level_limit is constant # min(list(df['max_ang_speed_sampled'].unique())) ### Assume that the df was already prefiltered for the max lin speed
    min_lin_speed_limmit =  dict_terrain["min_lin_speed_limmit"] #5.0  #The low-level_limit is constant min(list(df['max_linear_speed_sampled'].unique()))
    
    # Define the coordinates of the polygon
    max_body_slip  = np.array([(-min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,-min_ang_speed_limmit,), 
                      ( min_lin_speed_limmit,min_ang_speed_limmit,), 
                      (-min_lin_speed_limmit, min_ang_speed_limmit),
                      (-min_lin_speed_limmit, -min_ang_speed_limmit)]).T # A square
    
    b = robot_param["basewidth"]
    r = robot_param["wheel_radius"] 
    jacobians = np.array([[1/2,1/2],[-1/b, 1/b]]) * r

    max_body_in_wheel_constraints = (np.linalg.inv(jacobians) @ max_body_slip)
    

    
    ### Assuming that wheel reaches their Steady state velocity after 2 s
    left_wheel = column_type_extractor(df, "cmd_left")
    right_wheel = column_type_extractor(df, "cmd_right")
    
    max_wheel_speed_cmd = robot_param["maximum_wheel_speed_empty"] #max([np.max(np.abs(left_wheel)),np.max(np.abs(right_wheel))])
    
    max_wheel_coordinates = np.array([(-max_wheel_speed_cmd,-max_wheel_speed_cmd), (-max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, max_wheel_speed_cmd), (max_wheel_speed_cmd, -max_wheel_speed_cmd)]).T  #    

    ##### 
    
    
    # Create the polygon
    
    rectangle = Polygon(zip(max_body_in_wheel_constraints[1,:],max_body_in_wheel_constraints[0,:]))

    rectangle_2 = Polygon(zip(max_wheel_coordinates[1,:],max_wheel_coordinates[0,:]))
    
    union_res = rectangle.intersection(rectangle_2)

    if debug:

        fig, axs = plt.subplots(1,1)

        x,y = union_res.exterior.xy
        axs.plot(x,y)
        plt.show()

    return union_res
    



def filter_all_results_clearpath(path_to_df,robot,filter_data=False,debug=False):


    with open(PATH_ROBOT_CONFIG_FILE) as file:

        robot_dict = yaml.safe_load(file)["robot"]
        robot_param = robot_dict[robot]

    if isinstance(path_to_df,str):
        path_to_df = pathlib.Path(path_to_df)
    else:
        path_to_df = pathlib.Path(path_to_df)

    df = pd.read_pickle(path_to_df)
    # Extract path_to_save
    
    path_parent = path_to_df.parent
    last_name = path_to_df.parts[-1]
    path_to_save = path_parent/ (f"filtered_cleared_path_{robot}_following_robot_param_"+last_name)
    
    # Extract result of the robot
    df = df.loc[df.robot==robot]

    
    list_terrain = list(df.terrain.unique())

    list_df = []

    print(list_terrain)
    dict_terrain = {}

    for terrain in list_terrain: 
        
        new_df,min_ang_speed_limmit,min_lin_speed_limmit = reverse_engineer_clearpath_max_speed(df.loc[df.terrain== terrain],robot_param,filter_data=filter_data,debug=debug)
        
        dico_temp = {"min_ang_speed_limmit":min_ang_speed_limmit,
                    "min_lin_speed_limmit":min_lin_speed_limmit,
                    "maximum_wheel_speed_empty":robot_param["maximum_wheel_speed_empty"]}
        dict_terrain[terrain] =dico_temp         
        list_df.append(new_df)
        print(len(list_df))
    
    print(list_df)
    df_finall = pd.concat(list_df,axis=0)

    df_finall.to_pickle(path_to_save)
    dict_terrain["robot"] = robot_param
    extract_wheel_and_clearpath_limit_by_terrain(path_to_save,robot,dict_terrain)

    
    return df_finall
### Extract the maximum limits from the body. 

    
def extract_wheel_and_clearpath_limit_by_terrain(path_to_df,robot,dict_terrain):
    """THE DF MUST ALREADY BE FILTERED

    Args:
        path_to_df (_type_): _description_
    """

    if isinstance(path_to_df,str):
        path_to_df = pathlib.Path(path_to_df)

    parent= path_to_df.parent
    geom_suffix = path_to_df.parts[-1]

    path_to_save =parent/(f"{robot}_geom_limits_by_terrain_for_"+geom_suffix)

    
    df = pd.read_pickle(path_to_df)

    list_terrain = list(df.terrain.unique())

    df_geom = {"body":{},"wheel":{},"dict_terrain":dict_terrain}
    
    for terrain in list_terrain:
        df_terrain = df.loc[df.terrain==terrain]

        
        union_res, min_ang_speed_limmit,min_lin_speed_limmit,max_wheel_speed_cmd= generate_body_frame_domain_polygon(df_terrain,dict_terrain[terrain],dict_terrain["robot"])
        df_geom["body"][terrain] = union_res

        
        df_geom["wheel"][terrain] = generate_wheel_frame_domain_polygon(df_terrain,dict_terrain[terrain],dict_terrain["robot"])
        
    # Dump the dictionary into a pickle file
    with open(path_to_save, 'wb') as file:
        pickle.dump(df_geom, file)
    print("\n"*3,path_to_save,"\n"*3)


if __name__=="__main__":
    #argparser = argparse.ArgumentParser()
    #argparser.add_argument("--path",type=str,default=DATASET_PATH)
    #argparser.add_argument("--robot",type=str,default=ROBOT)
    #argparser.add_argument("--max_lin_speed",type=str,default=MAX_LIN_SPEED)
    #argparser.add_argument("--debug",type=bool,default=DEBUG)
    #
    ##args = argparser.parse_args()
    #path = args.path
    #robot = args.robot
    #max_lin_speed = args.max_lin_speed
    #debug = args.debug

    #filter_all_results_clearpath(path,robot,max_lin_speed,debug=debug)
    


    

    # Under that is to produce_the /filtered dataset for warthog
    filter_data = True
    path = "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"
    df_filtered = filter_all_results_clearpath(path,"warthog",debug=True,filter_data=filter_data)

    df_test = pd.read_pickle(path)
    
    df_sand = df_test.loc[df_test.terrain == "sand"]
    print(df_sand.shape)

    df_filtered_sand = df_filtered.loc[df_filtered.terrain=="sand"]
    print(df_filtered_sand)
    print('doing the slip dataset for results')
    path = "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"
    filter_all_results_clearpath(path,"husky",debug=False)

    
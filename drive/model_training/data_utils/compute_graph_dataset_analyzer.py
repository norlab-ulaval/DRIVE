import pandas as pd

import sys
print(sys.version)
sys.path.append('../../')
sys.path.append('../')
sys.path.append('../../../')


from drive.model_training.data_utils.extractors import * 
from drive.model_training.data_utils.animate_time_constant import * 
from first_order_model import *

from drive.util.model_func import *
from drive.util.transform_algebra import *



import matplotlib.animation as animation
from matplotlib.backend_bases import KeyEvent

import tqdm


class SteadyStateAnalysis():
    # 
    def __init__(self,path_2_acceleration_dataset,robot,terrain,traction,verbose=True,rate=0.05,n_step=40,n_window=3) -> None:
                
        
        self.df_acceleration = pd.read_pickle(path_2_acceleration_dataset)
        self.verbose = verbose
        self.rate = rate
        self.n_step = n_step
        self.n_timestamp = n_window * n_step

        self.path_to_model_training_datasets = pathlib.Path(path_2_acceleration_dataset).parent

        self.path_to_model_training_datasets_video = self.path_to_model_training_datasets/"video"

        
        if self.path_to_model_training_datasets_video.is_dir() ==False:
            
            self.path_to_model_training_datasets_video.mkdir()
        self.robot = robot
        self.traction = traction
        self.terrain = terrain

    def compute_body_vel_cmd(self, df_slip,cmd_l_name,cmd_r_name):


        cmd_left = column_type_extractor(df_slip, cmd_l_name,verbose=False)
        cmd_right = column_type_extractor(df_slip,cmd_r_name,verbose=False)
        
        cmd_left_reshape = reshape_into_6sec_windows(cmd_left, n_window=3)
        cmd_right_reshape = reshape_into_6sec_windows(cmd_right,n_window=3) 

        list_cmd_vx = []
        list_cmd_vyaw = []

        for i in range(cmd_left_reshape.shape[0]):
            cmd_left_i = cmd_left_reshape[i,:]
            cmd_right_i = cmd_right_reshape[i,:]
            cmd_vel = np.vstack((cmd_left_i,cmd_right_i))
            body_vel = compute_body_vel_IDD(cmd_vel , robot='warthog-wheel')

            list_cmd_vx.append(body_vel[0,:])
            list_cmd_vyaw.append(body_vel[1,:])

        cmd_body_cmd_x = reshape_into_2sec_windows(np.array(list_cmd_vx),n_window=3)
        cmd_body_cmd_yaw = reshape_into_2sec_windows(np.array(list_cmd_vyaw),n_window=3)

        total_name_2_add = [f"cmd_body_vel_x_{i}" for i in range(cmd_body_cmd_x.shape[1])] + [f"cmd_body_vel_yaw_{i}" for i in range(cmd_body_cmd_yaw.shape[1])]

        df = pd.DataFrame(np.hstack((cmd_body_cmd_x,cmd_body_cmd_yaw)),columns=total_name_2_add)

        new_df_slip = pd.concat((df_slip,df),axis=1)

        
        return new_df_slip 
        
    def compute_time_cst_gain_delay_step_operation_points(self, df_slip,name_column_interest, name_column_cmd,freq, n_window = 3,verbose=False,produce_video_now=False,video_saving_path=""):
        """Compute the time_constant, time_delay, gains.

        Args:
            df_slip (df): _description_
            name_column_interest (str): _description_
            name_column_cmd (str): _description_
            n_window (int, optional): _description_. Defaults to 3.
            verbose (bool, optional): _description_. Defaults to False.
        """
        column_of_interest = column_type_extractor(df_slip, name_column_interest,verbose=verbose)
        column_interest_reshape = reshape_into_6sec_windows(column_of_interest, n_window=n_window)
        
        if name_column_cmd =="":
            cmd_of_interest_reshape = np.zeros(column_interest_reshape.shape)
        else:
            cmd_of_interest = column_type_extractor(df_slip, name_column_cmd,verbose=verbose)
            cmd_of_interest_reshape = reshape_into_6sec_windows(cmd_of_interest, n_window=n_window)

        operation_point_interest,steps_cmd_interest_reshape = compute_operation_points_and_step(column_interest_reshape,cmd_of_interest_reshape)

        nb_timestamp = self.n_timestamp
        max_time_bounds = freq * nb_timestamp
        #print("max time bounds",max_time_bounds)
        #
        time_axis = create_time_axe(freq,nb_timestamp)
        first_order_model = FirstOrderModelWithDelay(1,1,freq)
        step_y_interest_centered = column_interest_reshape-operation_point_interest
        gains_computed, time_constants_computed, time_delay_computed,predictions = first_order_model.train_all_calib_state(time_axis,steps_cmd_interest_reshape,step_y_interest_centered,operation_point_interest,max_time_bounds)


        predictions_error = np.sum(np.abs(predictions-column_of_interest.reshape(predictions.shape))) 
        # Create a second dataframe 

        
        time_constant_error_flag = np.squeeze((np.abs(time_constants_computed)> nb_timestamp * freq ) | (time_constants_computed<0))
        
        time_delay_error_flag =np.squeeze((np.abs(time_delay_computed)> nb_timestamp * freq ) | (time_delay_computed<0))

        
        valid_mask = np.logical_or(time_delay_error_flag,  time_constant_error_flag)
        #print("")
        #print( valid_mask.shape )

        
        # left_wheel_vel_predictions_sum_abs shape = ()
        # right_wheel_vel_valid_mask shape = (197, 197)
        # right_wheel_vel_time_constant_problematic_computation shape = (197, 1)
        # right_wheel_vel_time_delay_problematic_computation shape = (197,)
        
        if produce_video_now:
            #produce_video(predictions,time_axis,cmd_of_interest_reshape,gt_of_interest_reshpae ,names=["cmd","model","measurement"],video_saving_path="")
            produce_video(predictions,time_axis,cmd_of_interest_reshape,column_interest_reshape ,
                        names=[name_column_cmd,name_column_interest],video_saving_path=self.path_to_model_training_datasets_video)
            
        column_2_add = {name_column_interest+"_gains":gains_computed,
                    name_column_interest+"_time_constants":time_constants_computed,
                    name_column_interest+"_time_delay":time_delay_computed,
                    #name_column_interest+"_predictions_sum_abs_tc":predictions_error,
                    name_column_interest+"_time_constant_problematic_computation":time_constant_error_flag,
                    name_column_interest+"_time_delay_problematic_computation":time_delay_error_flag,
                    name_column_interest+"_valid_mask_tc":valid_mask,
                    name_column_interest+"_operation_points": np.squeeze(operation_point_interest),
                    name_column_interest+"_steps": np.squeeze(steps_cmd_interest_reshape)
                    }
        
        return column_2_add


    def compute_time_constant_wheel_and_body(self, df_slip,rate, nb_timestamp, verbose = True,produce_video_now = False,video_saving_path=""):

        list_of_cmd_vector = ['cmd_left', "cmd_right","cmd_body_vel_x","cmd_body_vel_yaw",""]
        list_column_interest = ["left_wheel_vel","right_wheel_vel","icp_vel_x" , "icp_vel_yaw", "icp_vel_y" ] 

        df_slip_w_cmd_body = self.compute_body_vel_cmd(df_slip,"cmd_left","cmd_right")

        dico_column_2_add = {}

        #print(rate,nb_timestamp)
        
        for cmd_name, column_of_interest in tqdm.tqdm(zip(list_of_cmd_vector,list_column_interest)):

            dico_new_column = self.compute_time_cst_gain_delay_step_operation_points(df_slip_w_cmd_body,column_of_interest, cmd_name,rate, n_window = 3,verbose=verbose,produce_video_now=produce_video_now,video_saving_path=video_saving_path)
            dico_column_2_add.update(dico_new_column)

        
        if verbose:
            for key,value in dico_column_2_add.items():
                print(f'{key} shape = {value.shape}')
            
        
        
        return dico_column_2_add


    ### Create the dataframe for diamond shape

    def create_dataframe_for_diamond_shape_graph(self,produce_video_now=False):

        df_slip_dataset = self.df_acceleration
        
        # Steady state keeping
        df_steady_state = df_slip_dataset.loc[df_slip_dataset['steady_state_mask'] == 1]

        print(df_steady_state)


        # Remove the first window
        df_last_window = df_steady_state.drop_duplicates(subset=['steady_state_mask','calib_step'],keep='last')
        #print(df_last_window.shape)

        print(df_last_window)
        
        cmd_left = np.mean(column_type_extractor(df_last_window, 'cmd_left',verbose=False),axis=1)
        cmd_right = np.mean(column_type_extractor(df_last_window, 'cmd_right',verbose=False),axis=1)
        cmd_vel = np.vstack((cmd_left,cmd_right))
        body_vel = compute_body_vel_IDD(cmd_vel , robot='warthog-wheel')

        ##
        icp_vel_x = np.mean(column_type_extractor(df_last_window, 'icp_vel_x',verbose=False),axis=1)
        icp_vel_y = np.mean(column_type_extractor(df_last_window, 'icp_vel_y',verbose=False),axis=1)
        icp_vel_yaw = np.mean(column_type_extractor(df_last_window, 'icp_vel_yaw',verbose=False),axis=1)
        body_vel_icp_mean = np.vstack((icp_vel_x,icp_vel_yaw))

        
        #raw_icp_vel_x_mean = np.mean(column_type_extractor(df_last_window, 'raw_icp_vel_x',verbose=False),axis=1)
        #raw_icp_vel_yaw_mean = np.mean(column_type_extractor(df_last_window, 'raw_icp_vel_yaw',verbose=False),axis=1)


        odom_speed_l = np.mean(column_type_extractor(df_last_window, 'left_wheel_vel',verbose=False),axis=1)
        odom_speed_right = np.mean(column_type_extractor(df_last_window, 'right_wheel_vel',verbose=False),axis=1)

        body_slip_x =  body_vel[0,:] - icp_vel_x
        body_slip_yaw = body_vel[1,:] -icp_vel_yaw 

        

        size = odom_speed_l.shape[0]
        dictionnary_ = {"terrain": [self.terrain] * size,
                        "robot": [self.robot] * size,
                        "traction": [self.traction] * size,
                        #"offline_localisation":[offline_localisation]*size,
                        "cmd_left_wheels":cmd_left,
                        "cmd_right_wheels":cmd_right,
                        "cmd_body_x":body_vel[0,:],
                        "cmd_body_yaw":body_vel[1,:],
                        "icp_vel_x_smoothed":icp_vel_x,
                        "icp_vel_yaw_smoothed":icp_vel_yaw,
                        #"raw_icp_vel_x_mean":raw_icp_vel_x_mean,
                        #"raw_icp_vel_yaw_mean":raw_icp_vel_yaw_mean,
                        "odom_speed_left_wheels": odom_speed_l,
                        "odom_speed_right_wheels": odom_speed_right,
                        "slip_body_x_ss": body_slip_x,
                        "slip_body_yaw_ss": body_slip_yaw,
                        "slip_body_y_ss": icp_vel_y,
                        "slip_wheel_left_ss": cmd_left-odom_speed_l,
                        "slip_wheel_right_ss": cmd_right-odom_speed_right,
                        }

        

        df = pd.DataFrame.from_dict(dictionnary_)

        ## Compute time and body time constant
        
        dico_2_add = self.compute_time_constant_wheel_and_body(self.df_acceleration,self.rate , self.n_timestamp, verbose = False,produce_video_now = produce_video_now)
        df_temp  = pd.DataFrame.from_dict(dico_2_add)
        


        
        df = pd.concat((df,df_temp),axis=1)

        
        path = self.path_to_model_training_datasets/"steady_state_results.pkl"
        print(path)
        df.to_pickle(str(path))

        verbose=True
        if verbose ==True:
            print(f"cmd_shape {cmd_vel.shape}")
            print(f"body_vel_icp_mean {body_vel_icp_mean.shape}")
        return df
    

    def main(self,produce_video_now):
        # Extract the name of the rosbag.
        #name_divided = self.path_to_model_training_datasets.parts[-3].split("_")
        #robot = name_divided[0]
        #traction = name_divided[1]
        #terrain = name_divided[2]

        #print(robot,traction,terrain)

        self.create_dataframe_for_diamond_shape_graph(produce_video_now=produce_video_now)

        #)
        #print(df)
        #list_df.append(df)


    def compile_and_assemble_all_steady_state_video(self):
        pass

"""    def combine_terrain_dataset(param_array,path_2_save ="/home/nicolassamson/workspaces/norlab_WMRD/NicolasSamson/drive_datasets"):

        list_df = []
        
        for params in param_array:
            df_slip = pd.read_pickle(params["path_2_dataset"])

            drive_params = params["calibration_node_config_file"]["/drive/calibration_node"]["ros__parameters"]

            df = create_dataframe_for_diamond_shape_graph(df_slip,params["terrain"],params["robot"],params["traction"],drive_params
                                                        )
            print(df)
            list_df.append(df)

        df = pd.concat(list_df)

        if path_2_save != "/home/nicolassamson/workspaces/norlab_WMRD/NicolasSamson/drive_datasets":
            path_to_save  = path_2_save # 
        else:
            path_to_save  = pathlib.Path(path_2_save)/"all_terrain_diamond_shape_graph_data.pkl"
        df.to_pickle(str(path_to_save))
        return df 
"""

if __name__== "__main__":
    path_2_acceleration_dataset ="/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/gravel/warthog_wheels_gravel_ral2023/model_training_datasets/slip_dataset_all.pkl"


    robot = "warthog"
    terrain = "gravel" 
    traction = "wheels"
    
    analyzer = SteadyStateAnalysis(path_2_acceleration_dataset,robot,terrain,traction,verbose=True,rate=0.05,n_step=40,n_window=3)
    #column_list= list(analyzer.df_acceleration.columns)
    #for column in column_list:
    #    print(column)
    #analyzer.df_acceleration[["icp_vel_x","icp_vel_y","icp_vel_yaw"]].describe()
    analyzer.main(produce_video_now=True)


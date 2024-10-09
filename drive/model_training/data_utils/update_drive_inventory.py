
import pathlib 
import datetime
import yaml 
import copy 
import sys
import pandas as pd
import numpy as np 
import time 

from drive.model_training.data_utils.dataset_parser import DatasetParser
from drive.model_training.data_utils.slip_dataset_parser import SlipDatasetParser
from drive.model_training.data_utils.graph_module import   GraphicProductionDrive
from drive.model_training.model_trainers.powertrain_trainer import Powertrain_Trainer
from drive.model_training.models.powertrain.bounded_powertrain import Bounded_powertrain

def extract_components_from_names(names):
    names_components = names.split("_")
    
    robot = names_components[0]
    traction = names_components[1]
    terrain = names_components[2]
    return robot,traction,terrain

def trained_powertrain_model(df,path_to_save_training_results_with_x_steps,max_wheel_vel,min_wheel_vel,path_to_powertrain_config,rate,training_horizon):

    
    ####### Load param of the calibration node
    with open(path_to_powertrain_config, 'r') as file:
        motion_model_trainer_config = yaml.safe_load(file)
        pwrtrain_param_dict= motion_model_trainer_config["pwrtrain_motion_model"]

    
    init_param_dict = pwrtrain_param_dict["init_params"]
    init_params = [init_param_dict["time_constant"], init_param_dict["time_delay"]]
    
    bounds_dict = pwrtrain_param_dict["minimizer_bounds"]

    bounds = [(bounds_dict["time_constant"][0], bounds_dict["time_constant"][1]),
            (bounds_dict["time_delay"][0], bounds_dict["time_delay"][1])]
    
    method = pwrtrain_param_dict["optimizer_methods"]

    timesteps_per_horizon = training_horizon * rate
    dt = 1/rate
    
    ####### Train the motion model 
    bounded_powertrain = Bounded_powertrain(min_wheel_vel, max_wheel_vel, time_constant=0.5, time_delay=dt, dt=dt)
    

    powertrain_trainer = Powertrain_Trainer(powertrain_model=bounded_powertrain, init_params=init_params, dataframe=df,
                                timesteps_per_horizon=timesteps_per_horizon, dt=dt)
    
    left_training_result, right_training_result = powertrain_trainer.train_model(init_params=init_params, method=method, bounds=bounds)


    #######  Save the results

    # Create the folder needed to saved the results 
    path_to_save_pwrtrain_results = path_to_save_training_results_with_x_steps/"powertrain"
    if path_to_save_pwrtrain_results.exists() == False:
        path_to_save_pwrtrain_results.mkdir()

    left_side_saved_params = path_to_save_pwrtrain_results /'powertrain_training_left.npy'
    #os.makedirs(os.path.dirname(left_side_saved_params), exist_ok=True)
    np.save(left_side_saved_params, left_training_result)
    
    right_side_saved_params = path_to_save_pwrtrain_results /'powertrain_training_right.npy'
    
    np.save(right_side_saved_params, right_training_result)

    return left_side_saved_params, right_side_saved_params



def compute_dataframe(pathlib_to_model_training_datasets,path_to_drive_repo,length_window_time=2.0):

    
    robot,traction,terrain = extract_components_from_names(str(pathlib_to_model_training_datasets.parent.name))
    
    print("Processing the raw_dataframe")
    ### Powertrain parser
    raw_dataset_path = pathlib_to_model_training_datasets/"raw_dataframe.pkl"
    export_dataset_path = pathlib_to_model_training_datasets/"torch_ready_dataframe.pkl"
    path_to_config = pathlib_to_model_training_datasets.parent/"config_file_used"/f"_{robot}.config.yaml"
    
    
    
    if path_to_config.is_file():
        with open(path_to_config, 'r') as file:
            config_file_robot = yaml.load(file, Loader=yaml.FullLoader)["/drive/calibration_node"]["ros__parameters"]
    else:
        raise ValueError(f"You need to add the config_file_used _{robot}.config.yaml for the experiment at the folowing place: \n {path_to_config}")
    
    path_to_config_model = pathlib_to_model_training_datasets.parent/"config_file_used"/f"_{robot}_model_trainer.config.yaml"
    if path_to_config_model.is_file():
        with open(path_to_config_model, 'r') as file:
            config_file_trainer = yaml.load(file, Loader=yaml.FullLoader)["/drive/model_trainer_node"]["ros__parameters"]
    else:
        raise ValueError(f"You need to add the motion model config filed for the powertrain model at the folowing place: \n {path_to_config_model}")
    

    training_horizon = length_window_time
    rate = config_file_robot["cmd_rate"]
    calib_step_time = config_file_robot["step_len"]
    wheel_radius = config_file_robot["wheel_radius"]
    baseline = config_file_robot["wheel_baseline"]
    imu_inverted = config_file_trainer["imu_inverted"]
    
    dataParser = DatasetParser(str(raw_dataset_path), export_dataset_path, training_horizon, rate,
                calib_step_time, wheel_radius, baseline, imu_inverted)
    df = dataParser.process_data()
    ## Extracts values 

    path_to_calib =  pathlib_to_model_training_datasets/"input_space_data.pkl"
    input_space_df = pd.read_pickle(path_to_calib)
    max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
    min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]

    print("Training the powertrain model")
    ### Powertrain train
    path_to_powertrain_config =  path_to_drive_repo/"motion_model_available"/"_pwrtain_motion_model_parameters.yaml"


    path_to_training_results = pathlib_to_model_training_datasets.parent/"model_training_results"
    
    if path_to_training_results.exists() == False:
        path_to_training_results.mkdir()

    path_to_all_steps = path_to_training_results/"offline"
    
    if path_to_all_steps.exists() == False:
        path_to_all_steps.mkdir()
    
    trained_powertrain_model(df,path_to_all_steps,max_wheel_vel,min_wheel_vel,path_to_powertrain_config,rate,training_horizon)

    print("Creating the slip datasets")
    # Slip dataset creation
    path_slip_df = pathlib_to_model_training_datasets/ "slip_dataset_all.pkl" 
    path_steady_state_df = pathlib_to_model_training_datasets/ "steady_state_results.pkl" 
    
    data_parser = SlipDatasetParser(df,path_to_all_steps,wheel_radius,baseline,min_wheel_vel,max_wheel_vel,rate)
    data_slip = data_parser.append_slip_elements_to_dataset(compute_by_whole_step=True,debug=False,smooth=True)
    data_slip.to_pickle(path_slip_df)

    print("Creating the diamond_datasets")
    diamond_shape = data_parser.create_dataframe_for_diamond_shape_graph(terrain,robot,traction,produce_video_now=False)

    # Creating the analysis
    print("creating the time constant video")
    graphic_producer = GraphicProductionDrive(path_to_dataframe_diamond=path_steady_state_df,path_to_dataframe_slip=path_slip_df)
    path_video = pathlib_to_model_training_datasets/"video"
    
    if path_video.is_dir() == False:
        path_video.mkdir()
    graphic_producer.produce_video_time_constants(video_saving_path=path_video,live_observation=False)
    
    return data_slip,diamond_shape

def append_selection_column(df,metadata_dict,config_file_robot):
    # df_slip,terrain_name,robot_name,traction_name,metadata_dict))

    shape = df.shape[0]
    added_column = pd.DataFrame.from_dict({
        "id": [id]*shape,
        "roboticist": [ metadata_dict["roboticist"]]*shape,
        "max_linear_speed_sampled": [config_file_robot["max_lin_speed"]]*shape,
        "max_ang_speed_sampled": [config_file_robot["max_ang_speed"]]*shape
    })
    
    return pd.concat((df,added_column),axis=1)

def update_yaml_file(result_folder="results_multiple_terrain_dataframe",drive_inventory_names = "drive_inventory",length_window_time=2.0):

    
    path_to_drive_repo =  pathlib.Path.cwd().parent.parent.parent
    path_to_drive_datasets_fodler =path_to_drive_repo/"drive_datasets"
    #print(path_to_drive_datasets_fodler)
    path_to_data = path_to_drive_datasets_fodler/"data"

    
    path_to_resutls_folder = path_to_drive_datasets_fodler/result_folder
    path_to_drive_inventory = path_to_resutls_folder/(drive_inventory_names+".yaml")
    dico_ready_datasets = {"last_update_time": datetime.datetime.now()}


    
    if not path_to_data.is_dir():

        raise ValueError("You need to create a folder name data in the 'drive_datasets' folder and extract dataframe informations from the rosbags.")
    
    if not path_to_resutls_folder.is_dir():

        raise ValueError(f"You need to create a fodler name '{result_folder}' to get the assembled results")

    dictionnary_dataframe = {"slip_dataset":[],"steady_state_dataset":[]}
    for robot in path_to_data.iterdir():
        if robot == "to_class":
            continue
        else:
            for traction in robot.iterdir():
                if robot == traction:
                        continue # Case when the folder is empty
                else:
                    
                    for terrain in traction.iterdir():
                        if terrain == traction:
                            continue # Case when the folder is empty
                        else:
                            
                            for experiment in terrain.iterdir():
                                
                                experiments_path =  experiment / "model_training_datasets"
                                
                                
                                path_2_config_file_robot_drive = experiment/"config_file_used"/("_"+robot.parts[-1]+".config.yaml")
                                with open(path_2_config_file_robot_drive, 'r') as file:
                                    config_file_robot = yaml.load(file, Loader=yaml.FullLoader)["/drive/calibration_node"]["ros__parameters"]

                                    path_2_metadata_file = experiment/"metadata.yaml"
                                    with open(path_2_metadata_file, 'r') as metadata_file:
                                        metadata_dict = yaml.load(metadata_file, Loader=yaml.FullLoader)

                                if experiments_path.is_dir():
                                    
                                    start_time = time.time()
                                    underline_number = 25
                                    print("_"*underline_number+f"starting the computation of {experiment.name}"+"_"*underline_number)
                                    df_slip, df_steady_state = compute_dataframe(experiments_path,path_to_drive_repo,length_window_time=length_window_time)
                                    end_time = time.time()

                                    print(f"computation time {np.round(end_time-start_time,2)} s")

                                    #### Change because the column will always be computed 
                                    #### create a function to add description columns 
                                    #### Add both path in the yaml file. 
                                    
                                    path_training_dataset_slip_dataset = str(experiments_path/"slip_dataset_all.pkl")
                                    path_training_dataset_steady_stateresults = str(experiments_path/"steady_state_results.pkl")
                                    
                                    robot_name,traction_name,terrain_name = extract_components_from_names(str(experiments_path))


                                    id = str(experiment.parts[-1])
                                    
                                    dico_dataset_specific = {
                                        str(experiment.parts[-1]):{
                                        "id": id,
                                        "path_2_slip_dataset":path_training_dataset_slip_dataset,
                                        "path_2_steady_state_dataset":path_training_dataset_steady_stateresults,
                                        "terrain":terrain_name,
                                        "robot":robot_name,
                                        "traction":traction_name,
                                        "calibration_node_config_file": path_2_config_file_robot_drive,
                                        "roboticist": metadata_dict["roboticist"]
                                        }
                                    }
                                    dico_ready_datasets.update(copy.copy(dico_dataset_specific))

                                    dictionnary_dataframe["slip_dataset"].append(append_selection_column(df_slip,metadata_dict,config_file_robot))
                                    dictionnary_dataframe["steady_state_dataset"].append(append_selection_column(df_steady_state,metadata_dict,config_file_robot))
                                    
    # Write data to a YAML file
    with open(path_to_drive_inventory, 'w') as file:
        yaml.dump(dico_ready_datasets, file)

    for key,df_to_concat in dictionnary_dataframe.items():
        
        path_to_compile_terrain = path_to_resutls_folder/("all_terrain_"+key+".pkl")
        big_df = pd.concat(df_to_concat)
        big_df.to_pickle(path_to_compile_terrain)
        
    return dico_ready_datasets


if __name__=="__main__":

    name = "warthog_wheels_gravel_2024_8_6_11h37s32"

    path_to_update_config_file = pathlib.Path.cwd().parent.parent.parent/"drive_datasets"/"scripts"/"config"/"update_config.yaml"
    
    with open(path_to_update_config_file, 'r') as file:
        update_config = yaml.load(file, Loader=yaml.FullLoader)

    dataset_used_for_append=update_config["dataset_used_for_append"]
    result_folder= update_config["result_folder"]
    drive_inventory_names = update_config["drive_inventory_names"]
    
#
    dico_2_do = update_yaml_file(result_folder=result_folder,drive_inventory_names = drive_inventory_names)
    dico_2_do.pop("last_update_time")
    list_dataframe = list(dico_2_do.values())
    
    


    


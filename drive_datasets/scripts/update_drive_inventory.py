
import pathlib 
import datetime
import yaml 
import copy 
import sys
print(sys.version)
print(sys.path.append('../'))
sys.path.append('../')

from drive.model_training.data_utils.compute_graph_dataset_analyzer import *
from utils.extractors import *




def update_yaml_file(dataset_used_for_append="slip_dataset_all.pkl",result_folder="results_multiple_terrain_dataframe",drive_inventory_names = "drive_inventory"):

    

    path_to_drive_datasets_fodler = pathlib.Path.cwd().parent

    path_to_data = path_to_drive_datasets_fodler/"data"

    
    path_to_resutls_folder = path_to_drive_datasets_fodler/result_folder
    path_to_drive_inventory = path_to_resutls_folder/(drive_inventory_names+".yaml")
    dico_ready_datasets = {"last_update_time": datetime.datetime.now()}


    
    if not path_to_data.is_dir():

        raise ValueError("You need to create a folder name data in the 'drive_datasets' folder and extract dataframe informations from the rosbags.")
    
    if not path_to_resutls_folder.is_dir():

        raise ValueError(f"You need to create a fodler name '{result_folder}' to get the assembled results")

    
    for robot in path_to_data.iterdir():
        if robot == "to_class":
            continue
        else:
            for traction in robot.iterdir():
                if robot == traction:
                        continue # Case when the folder is empty
                else:
                    print(traction)
                for terrain in traction.iterdir():
                    if terrain == traction:
                        continue # Case when the folder is empty
                    else:
                        
                        for experiment in terrain.iterdir():
                        
                            experiments_path =  experiment / "model_training_datasets"
                            
                            path_2_config_file_robot_drive = experiment/"config_file_used"/("_"+robot.parts[-1]+".config.yaml")
                            with open(path_2_config_file_robot_drive, 'r') as file:
                                config_file_robot = yaml.load(file, Loader=yaml.FullLoader)


                            if experiments_path.is_dir():
                                path_training_dataset = experiments_path/dataset_used_for_append
                            
                                if path_training_dataset.is_file():
                                    dico_dataset_specific = {
                                        str(experiment.parts[-1]):{
                                        "id": str(experiment.parts[-1]),
                                        "path_2_dataset":str(path_training_dataset),
                                        "terrain":terrain.parts[-1],
                                        "robot":robot.parts[-1],
                                        "traction":traction.parts[-1],
                                        "calibration_node_config_file": config_file_robot
                                        }
                                    }
                                    dico_ready_datasets.update(copy.copy(dico_dataset_specific))

                                   
                                else:
                                    print(f"The folowing dataset does not exist : \n {path_training_dataset}")

    
    
    # Write data to a YAML file
    with open(path_to_drive_inventory, 'w') as file:
        yaml.dump(dico_ready_datasets, file)
        
    return dico_ready_datasets


if __name__=="__main__":





    path_to_update_config_file = pathlib.Path.cwd().parent/"scripts"/"config"/"update_config.yaml"
    with open(path_to_update_config_file, 'r') as file:
        update_config = yaml.load(file, Loader=yaml.FullLoader)


    dataset_used_for_append=update_config["dataset_used_for_append"]
    result_folder= update_config["result_folder"]
    drive_inventory_names = update_config["drive_inventory_names"]
    diamond_dshape_resutls = update_config["diamond_shape_results_names"]


    dico_2_do = update_yaml_file(dataset_used_for_append=dataset_used_for_append,result_folder=result_folder,drive_inventory_names = drive_inventory_names)
    
    dico_2_do.pop("last_update_time")
    list_dataframe = list(dico_2_do.values())

    path_to_save_all_combined = (pathlib.Path.cwd().parent)/result_folder/(diamond_dshape_resutls+".pkl")
    print(path_to_save_all_combined)
    combine_terrain_dataset(list_dataframe,path_2_save =str(path_to_save_all_combined))



    


update_yaml_file()

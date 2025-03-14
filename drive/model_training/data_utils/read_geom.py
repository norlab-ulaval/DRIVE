import shapely
import matplotlib.pyplot as plt
import pandas
import yaml 
import pickle
path_to_save = "drive_datasets/results_multiple_terrain_dataframe/warthog_geom_limits_by_terrain_for_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"



with open(path_to_save, 'rb') as file:
    data = pickle.load(file)
    

print(data.keys())


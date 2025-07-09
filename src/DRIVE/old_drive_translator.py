import pandas as pd
from typing import Dict, Any, List
import pathlib  
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
import numpy as np 

DatasetPath = str
"""Absolute path to a csv or pkl file containing the Dataset produced by the old DRIVE system """

NewFormatPath = str
"""Absolute path to a folder where the new format dataset will be saved """

class OldDataset():

    def __init__(self, path_to_dataset: DatasetPath, path_to_new_folder: NewFormatPath,verbose: bool = True):
        
        self.path_to_dataset = path_to_dataset
        self.verbose = verbose

        if path_to_dataset.endswith('.csv'):
            self.data = pd.read_csv(path_to_dataset)
        elif path_to_dataset.endswith('.pkl'):
            self.data = pd.read_pickle(path_to_dataset)
        else:
            raise ValueError("Unsupported file format. Please provide a .csv or .pkl file.")
        
        # Create the new folder if it does not exist
        self.path_to_new_folder = pathlib.Path(path_to_new_folder)
        if not self.path_to_new_folder.is_dir():
            self.path_to_new_folder.mkdir(parents=True, exist_ok=True)
        
        self.data.rename(columns={
            "ros_time":"timestamp",
            "icp_pos_x": "x",
            "icp_pos_y": "y",
            "icp_pos_z": "z",
            "icp_quat_x": "qx",
            "icp_quat_y": "qy",
            "icp_quat_z": "qz",
            "icp_quat_w": "qw"
        }, inplace=True)
    
    def remove_first_line_of_zero(self):

        # Convert values to number 
        col = ["timestamp", "calib_step", "cmd_vel_x", "cmd_vel_omega", "x", "y", "z", 'qx', 'qy', 'qz', 'qw']
        self.data[col] = self.data[col].apply(pd.to_numeric, errors='coerce')
        n_values = self.data.shape[0]

        mask = self.data["qw"] != 0.0
        self.data = self.data[mask]
        
        if self.verbose:
            print(f"Initial number of values: {n_values}")
            print(f"{n_values - self.data.shape[0]} values dropped due to zero quaternion w component.")
        
        #print(self.data["calib_state"].value_counts())
        # compute step -1. 
    def extract_transition(self):
        """Extract the state transition in the old format to the new format. 
        This conversion will results in the absence of the folowing states:
        1. Waiting 
        2. Geofence_creation
        3. The idle state can represent both the paused and the driving back in zone. 
        4. Driving back in zone. 
        Because these steps were not executed in the past. 
        Assumes that the stop is not log. 
        """

        dict_conversion = {
            "": "ready_state",
            "calib": "running_state",
            "idle": "back_to_geofence_state",
            "drive_finished": "stopped"
        }

        data_states = self.data[["calib_step", "calib_state", "timestamp"]].copy()
        data_states.calib_state = data_states["calib_state"].replace(dict_conversion)
        #print(data_states["calib_state"].head(5))
        mask_of_change = data_states["calib_state"].shift(1) != data_states["calib_state"]
        #print(mask_of_change)
        previous_state = data_states[mask_of_change].copy()
        previous_state.rename(columns={"calib_state": "to_state"}, inplace=True)
        current_state = data_states.shift(1)[mask_of_change].copy()
        current_state.rename(columns={"calib_state": "from_state"}, inplace=True)

        # Assuming that we start in ready_state, the first line should identify the change from 
        # ready_state to running_state.
        # use the timestamp of the current state 
        current_state.reset_index(inplace=True,drop=True)
        previous_state.reset_index(inplace=True,drop=True)
        
        
        final_format = current_state.iloc[1:].join(previous_state["to_state"])[["timestamp","calib_step","from_state", "to_state"]].copy()
        
        file_name = self.path_to_new_folder / "state_transitions.csv"
        final_format.to_csv(file_name, index=False)
        
        first_running_time = final_format[final_format["to_state"] == "running_state"].iloc[0]["timestamp"]
        self.starting_timestamp = first_running_time

        if self.verbose:
            print(final_format.head(5))
            print(f"State transitions saved to {file_name}")
            print(f"First running state timestamp: {first_running_time}")
        

       


    def extract_steps(self):
        """
        Extract the calibration step from the dataset.
        """
        timestamp = self.data[["timestamp", "calib_step", "cmd_vel_x", "cmd_vel_omega"]].copy() 

        mask_to_remove_initial_idle_time = timestamp["timestamp"] >= self.starting_timestamp
        timestamp = timestamp[mask_to_remove_initial_idle_time].copy()

        
        start_timestamp = timestamp.drop_duplicates(subset=["calib_step"], keep='first')
        end_timestamp = timestamp.drop_duplicates(subset=["calib_step"], keep='last')
        delta_timestamp = (end_timestamp["timestamp"].values - start_timestamp["timestamp"].values) * 1e-9  # Convert to seconds 
        
        mask_restarted = np.where(delta_timestamp > 6.0, 1, 0)  # verify if steps was interrupted

        dict_dataframe = {
            "start_timestamp": start_timestamp["timestamp"].values,
            "end_timestamp": end_timestamp["timestamp"].values,
            "commanded_linear_velocity": start_timestamp["cmd_vel_x"].values,
            "commanded_angular_velocity": start_timestamp["cmd_vel_omega"].values,
            "completion_status": mask_restarted,
            "id": start_timestamp["calib_step"].values,
        }
        
        df_steps = pd.DataFrame.from_dict(dict_dataframe)
        
        df_steps.id = df_steps.id.astype(int)
        df_steps.set_index("id",drop=True, inplace=True)
        df_steps.completion_status = df_steps.completion_status.replace(0, "completed")
        df_steps.completion_status =df_steps.completion_status.replace(1, "restarted")
        

        file_name = self.path_to_new_folder / "steps.csv"
        df_steps.to_csv(file_name, index=True)
        if self.verbose:
            print(f"Steps extracted and saved to {file_name}")
            print(df_steps)

    def extract_positions(self):
        """It is possible that the start was done then immediatly paused by the user.
        """
        position_dataset = self.data[["timestamp","calib_step", "calib_state","x","y", "z", 'qx', 'qy',
        'qz', 'qw']].copy()
        
        position_dataset.drop_duplicates(subset=["x","y", "z", 'qx', 'qy',
        'qz', 'qw'],inplace=True)

        
        orientation_quaternions = position_dataset[["qx", "qy", "qz", "qw"]].to_numpy()
        
        list_rpy = []
        
        for i in range(orientation_quaternions.shape[0]):
            list_rpy.append(R.from_quat(orientation_quaternions[i],scalar_first=False).as_euler('xyz', degrees=False))
        
        rpy_dataset = pd.DataFrame(list_rpy, columns=["roll", "pitch", "yaw"])
        
        position_dataset = position_dataset.join(rpy_dataset)


        # replace calib_step before by -1 
        timestamp = position_dataset["timestamp"].values
        calib_step = position_dataset["calib_step"].values

        calib_step = np.where(timestamp < self.starting_timestamp, -1, calib_step)
        position_dataset["calib_step"] = calib_step

        file_name = self.path_to_new_folder / "positions.csv"
        position_dataset.to_csv(file_name, index=False)
        if self.verbose:
            print(f"Positions extracted and saved to {file_name}")
            print(position_dataset.head(5))
            print(f"Starting timestamp for positions: {self.starting_timestamp}")
        

    def process_dataset(self):
        """
        Process the dataset to extract positions and orientations.
        """
        
        self.remove_first_line_of_zero()
        self.extract_transition()
        self.extract_steps()
        self.extract_positions()
        # Add more processing methods as needed


    
if __name__ == "__main__":
    # Example usage
    dataset_path = "/home/nicolassamson/temp/drive_datasets/data/warthog/wheels/sand/warthog_wheels_sand_2024_9_25_14h32s32/model_training_datasets/raw_dataframe.pkl"  # Replace with your actual dataset path
    path_to_new_folder = "/home/nicolassamson/temp/new_format"  # Replace with your desired new format path
    old_dataset = OldDataset(dataset_path,path_to_new_folder)
    print(old_dataset.data.columns)  # Display the path to the dataset
    print(old_dataset.data.head())  # Display the first few rows of the dataset 
    old_dataset.process_dataset()  # Process the dataset to extract positions and orientations
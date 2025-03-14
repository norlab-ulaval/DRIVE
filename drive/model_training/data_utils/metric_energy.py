import numpy as np 
import pandas as pd 
import pathlib 
import yaml
from drive.model_training.data_utils.extractors import *
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive
import pickle 
import matplotlib.pyplot as plt 
global PATH_TO_METRIC 
global PATH_TO_SAVE_FOLDER
from datetime import datetime
import sys
import os
project_root = os.path.abspath("/home/william/workspaces/drive_ws/src/DRIVE/")
if project_root not in sys.path:
    sys.path.append(project_root)
    

PATH_TO_METRIC = pathlib.Path('drive/model_training/data_utils/metric_config.yaml')
PATH_TO_SAVE_FOLDER = pathlib.Path('drive_datasets/results_multiple_terrain_dataframe/metric')
RESULTS_FILE_NAME = "metric_results.pkl"

PATH_TO_RESULT_FILE = PATH_TO_SAVE_FOLDER/RESULTS_FILE_NAME

if not PATH_TO_SAVE_FOLDER.is_dir():
    PATH_TO_SAVE_FOLDER.mkdir()




class Dataset2Evaluate():

    def __init__(self,dataset_name) -> None:
        
        with open(PATH_TO_METRIC, 'r') as file:
            config_file = yaml.safe_load(file)
        
        

        for datasets,datasets_info in config_file["datasets"].items():

            if datasets == dataset_name:

                self.datasets_info = datasets_info
        
        
        path = self.datasets_info["path_dataset"]

        # Extract the dataframe and filter
        self.df = pd.read_pickle(path)
        print(self.df.terrain.unique())
        for column_filter, value in self.datasets_info["filter_columns_and_values"].items():
            self.df = self.df.loc[self.df[column_filter]==value]

        self.id = dataset_name + "_"+ self.datasets_info["filter_columns_and_values"]["robot"] +"_"+ self.datasets_info["filter_columns_and_values"]["traction"] 
        
        self.terrains = list(self.df.terrain.unique())
        
        print(self.terrains)
        self.rate = self.datasets_info["rate"]

    def get_sub_sample(self, terrain,lim_yaw,lim_vel_x):
        # Extract the columns of the dataframe
        dict_2_update = {"format":self.datasets_info["format"]}

        df_by_terrain = self.df.loc[self.df.terrain == terrain]
        
        cmbd_vel_x = df_by_terrain['cmd_body_x_lwmean'].to_numpy()
        cmbd_vel_yaw = df_by_terrain['cmd_body_yaw_lwmean'].to_numpy()

        mask = (cmbd_vel_x <= lim_vel_x) & (cmbd_vel_yaw <= lim_yaw)

        for attribute_name, column_name in self.datasets_info["columns_names"].items():
            dict_2_update[attribute_name] = column_type_extractor(df_by_terrain,column_name)[mask]
        

        col_2_add = {"dataset_id":self.id}
        col_2_add.update(self.datasets_info["filter_columns_and_values"])
        dict_2_update["col_2_multiply_and_add"] = col_2_add 

        return dict_2_update

    def __getitem__(self, terrain):
        # Extract the columns of the dataframe
        dict_2_update = {"format":self.datasets_info["format"]}

        df_by_terrain = self.df.loc[self.df.terrain == terrain]
        
        for attribute_name, column_name in self.datasets_info["columns_names"].items():
            dict_2_update[attribute_name] = column_type_extractor(df_by_terrain,column_name)
        
        #print(self.terrains.unique())
        col_2_add = {"dataset_id":self.id}
        col_2_add.update(self.datasets_info["filter_columns_and_values"])
        dict_2_update["col_2_multiply_and_add"] = col_2_add 

        return dict_2_update

class DifficultyMetric():


    def __init__(self,metric_name) -> None:
        
        

        with open(PATH_TO_METRIC, 'r') as file:
            metric_param_config = yaml.safe_load(file)
        
        

        
        for metric,params_metric in metric_param_config["metric"].items():

            if metric == metric_name:
                
                self.metric_parameters = params_metric
        

        path_2_robot = self.metric_parameters["robot_rel_path"]
        with open(path_2_robot, 'r') as file:
            robot_param = yaml.safe_load(file)
            

        self.metric_parameters["robot"] = robot_param["robot"]


        self.metric_name = metric_name
        

        # Read results_file 
        list_possible_metric = ["kinetic_energy","kinetic_energy_wheel_encoder",
                                "kinetic_energy_wheel_encoder_ratio","DiffSpeedProprioExteroEnergy",
                                "KineticEnergyWheelOnly","KineticEnergyICPOnly","KineticEnergyMetricOnly",
                                "SlopeMetric"]
        
        if not PATH_TO_RESULT_FILE.is_file():
            empty_dict  = {}
            for metric in list_possible_metric:
                empty_dict[metric] = {}
    
            with open(PATH_TO_RESULT_FILE, 'wb') as file:
                results_file = pickle.dump(empty_dict,file)
        

        with open(PATH_TO_RESULT_FILE, 'rb') as file:
            self.results_file = pickle.load(file)
        

class KineticEnergyMetric(DifficultyMetric):

    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name)

        self.robot_name = robot_name
        # Extract robot params
        for robot,robot_param in self.metric_parameters["robot"].items():
            if robot == self.robot_name:
                self.width = robot_param["width"]
                self.length = robot_param["length"]
                self.basewidth = robot_param["basewidth"]
                self.wheel_radius = robot_param["wheel_radius"]
                self.masse = robot_param["masse"]
                self.compute_intertia()
        self.metric_name = "kinetic_energy"
        print(self.inertia_constraints )

    def compute_intertia(self):
        self.inertia_constraints = (self.width**2 + self.length**2)/12
    def compute_energy(self,vx,vy,omega_body):
        """_summary_

        Args:
            vx (array): assuming that the vector is N by 1
            vy (_type_): assuming that the vector is N by 1
            omega_body (_type_): assuming that the vector is N by 1
        """

        translation_energy = 1/2 * self.masse * (vx**2+vy**2) 
        rotationnal_energy = 1/2 * self.masse * (self.inertia_constraints * omega_body**2)
        state_kin_energy =  translation_energy + rotationnal_energy

        
        return state_kin_energy,rotationnal_energy, translation_energy

    def compute_kinetic_energy_metric(self,dataset):

        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Assuming instant acceleration
        y_cmd =np.zeros(dataset["gt_body_lin_vel"].shape)
        idd_energies = self.compute_energy(dataset["cmd_body_lin_vel"],
                                                y_cmd,
                                                dataset["cmd_body_yaw_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, gt_energy, idd_energy in zip(energy_order,gt_energies,idd_energies):

                kinetic_metric = (idd_energy[:,:-1] - gt_energy[:,1:])  / idd_energy[:,:-1] 
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy
    
    def compute_all_terrain(self,dataset,multiple_terrain=False):

        
        list_df = []
        for terrain in dataset.terrains:
            
            dico_data = dataset[terrain]
            shape = dico_data["cmd_left_wheel"].shape
            result_terrain = self.compute_kinetic_energy_metric(dico_data)
        
            result_terrain["cmd_left_wheel"] = dico_data["cmd_left_wheel"]
            result_terrain["cmd_right_wheel"] = dico_data["cmd_right_wheel"]
            # Create all names
            
            new_dict = {}
            list_col = list(result_terrain.keys()) 
            results_dict_1_name_by_column = create_columns_names_from_dict_with_names(list_col,result_terrain,new_dict)
            # Add identifying columns 
            for key,value in dico_data["col_2_multiply_and_add"].items():
                results_dict_1_name_by_column[key] = [value] * shape[0]
            
            results_dict_1_name_by_column["terrain"] = [terrain] * shape[0]
            list_df.append(pd.DataFrame.from_dict(results_dict_1_name_by_column))

        df_all_terrain = pd.concat(list_df)

        # Get the current date and time
        self.results_file[self.metric_name] = {dataset.id:df_all_terrain}

        with open(PATH_TO_RESULT_FILE, 'wb') as file:
            results_file = pickle.dump(self.results_file,file)
        

        
        #self.saving_path = path_to_dataset_folder
        

class KineticEnergyMetricWheelEncoder(KineticEnergyMetric):

    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        
        
        self.metric_name = "kinetic_energy_wheel_encoder"
        self.jacobian = self.wheel_radius *np.array([[1/2, 1/2], [-1/self.basewidth,1/self.basewidth]])
        # Initiate IDD
        # Initiate IDD
        
    def compute_energy_from_wheel_encoder(self,left_wheel_encoder,right_wheel_encoder,vy_array):
        """Compute the energy of equivalent from the wheel encoder motion predicted by the IDD
        Args:
            left_wheel_encoder (_type_): _description_
            right_wheel_encoder (_type_): _description_

        Returns:
            _type_: _description_
        """

        # Compute the vx, vy equivalent speed encoder 

        list_state_kin_energy = [] 
        list_rotationnal_energy = []
        list_translationnal_energy = []
        for horizon in range(left_wheel_encoder.shape[0]):

            u_wheel = np.array([left_wheel_encoder[horizon,:],right_wheel_encoder[horizon,:]])
            body_cmd = self.jacobian @ u_wheel

            vx = body_cmd[0,:]
            vomega = body_cmd[1,:]
            vy = np.zeros(vx.shape) # vy_array[horizon,:]  

            state_kin_energy,rotationnal_energy, translation_energy = self.compute_energy(vx,vy,vomega)

            list_state_kin_energy.append(state_kin_energy)
            list_rotationnal_energy.append(rotationnal_energy)
            list_translationnal_energy.append(translation_energy)

        return np.array(list_state_kin_energy),np.array(list_rotationnal_energy), np.array(list_translationnal_energy)

    
    def compute_kinetic_energy_metric(self,dataset):

        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Computing energy from proprioceptive sensors (wheel encoder)
        
        wheel_encoder_energies = self.compute_energy_from_wheel_encoder(dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, gt_energy, wheel_encoder_energy in zip(energy_order,gt_energies,wheel_encoder_energies):

                kinetic_metric = (wheel_encoder_energy - gt_energy)  / wheel_encoder_energy
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy
    
    def compute_all_terrain(self,dataset,multiple_terrain=False):

        new_file =False
        list_df = []
        for terrain in dataset.terrains:
            
            dico_data = dataset[terrain]
            shape = dico_data["cmd_left_wheel"].shape
            result_terrain = self.compute_kinetic_energy_metric(dico_data)
        
            result_terrain["cmd_left_wheel"] = dico_data["cmd_left_wheel"]
            result_terrain["cmd_right_wheel"] = dico_data["cmd_right_wheel"]
            # Create all names
            
            list_col = list(result_terrain.keys()) 
            new_dico = {}
            results_dict_1_name_by_column = create_columns_names_from_dict_with_names(list_col,result_terrain,new_dico)
            # Add identifying columns 
            for key,value in dico_data["col_2_multiply_and_add"].items():
                results_dict_1_name_by_column[key] = [value] * shape[0]
            
            results_dict_1_name_by_column["terrain"] = [terrain] * shape[0]
            list_df.append(pd.DataFrame.from_dict(results_dict_1_name_by_column))

        df_all_terrain = pd.concat(list_df)

        # Get the current date and time
        self.results_file[self.metric_name] = {dataset.id:df_all_terrain}

        with open(PATH_TO_RESULT_FILE, 'wb') as file:
            results_file = pickle.dump(self.results_file,file)

        #self.saving_path = path_to_dataset_folder
        

class KineticEnergyMetricWheelEncoderRatio(KineticEnergyMetricWheelEncoder):

    
    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        
        
        self.metric_name = "kinetic_energy_wheel_encoder_ratio"
        self.jacobian = self.wheel_radius *np.array([[1/2, 1/2], [-1/self.basewidth,1/self.basewidth]])
        # Initiate IDD
        # Initiate IDD
    def compute_kinetic_energy_metric(self,dataset):
        
        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Computing energy from proprioceptive sensors (wheel encoder)
        
        wheel_encoder_energies = self.compute_energy_from_wheel_encoder(dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, gt_energy, wheel_encoder_energy in zip(energy_order,gt_energies,wheel_encoder_energies):

                kinetic_metric = gt_energy/wheel_encoder_energy
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy
    
class KineticEnergyWheelOnly(KineticEnergyMetricWheelEncoder):

    
    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        
        
        self.metric_name = "KineticEnergyWheelOnly"
        self.jacobian = self.wheel_radius *np.array([[1/2, 1/2], [-1/self.basewidth,1/self.basewidth]])
        # Initiate IDD
        # Initiate IDD
    def compute_kinetic_energy_metric(self,dataset):
        
        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Computing energy from proprioceptive sensors (wheel encoder)
        
        wheel_encoder_energies = self.compute_energy_from_wheel_encoder(dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, wheel_encoder_energy in zip(energy_order,wheel_encoder_energies):

                kinetic_metric = wheel_encoder_energy
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy

class KineticEnergyICPOnly(KineticEnergyMetricWheelEncoder):

    
    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        
        
        self.metric_name = "KineticEnergyICPOnly"
        self.jacobian = self.wheel_radius *np.array([[1/2, 1/2], [-1/self.basewidth,1/self.basewidth]])
        # Initiate IDD
        # Initiate IDD
    def compute_kinetic_energy_metric(self,dataset):
        
        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        
        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, gt_energy in zip(energy_order,gt_energies):

                kinetic_metric = gt_energy
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy

class DiffSpeedProprioExteroEnergy(KineticEnergyMetricWheelEncoder):

    
    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        
        
        self.metric_name = "DiffSpeedProprioExteroEnergy"
        self.jacobian = self.wheel_radius *np.array([[1/2, 1/2], [-1/self.basewidth,1/self.basewidth]])
        # Initiate IDD
        # Initiate IDD
    
    def compute_diff_proprio_extero_energy(self,left_wheel_encoder,right_wheel_encoder,vy_array,gt_vx,gt_vy,gt_vyaw):
        """Compute the energy of equivalent from the wheel encoder motion predicted by the IDD
        Args:
            left_wheel_encoder (_type_): _description_
            right_wheel_encoder (_type_): _description_

        Returns:
            _type_: _description_
        """

        # Compute the vx, vy equivalent speed encoder 

        list_state_kin_energy = [] 
        list_rotationnal_energy = []
        list_translationnal_energy = []
        for horizon in range(left_wheel_encoder.shape[0]):

            u_wheel = np.array([left_wheel_encoder[horizon,:],right_wheel_encoder[horizon,:]])
            body_cmd = self.jacobian @ u_wheel

            vx = body_cmd[0,:]
            vomega = body_cmd[1,:]
            vy = np.zeros(vx.shape) # vy_array[horizon,:]  

            horizon_gt_vx = gt_vx[horizon,:]
            horizon_gt_vy = gt_vy[horizon,:]
            horizon_gt_vyaw = gt_vyaw[horizon,:]

            delta_vx = vx - horizon_gt_vx
            delta_vy = vy - horizon_gt_vy
            delta_vyaw = vomega - horizon_gt_vyaw
            
            state_kin_energy,rotationnal_energy, translation_energy = self.compute_energy(delta_vx,delta_vy,delta_vyaw)

            list_state_kin_energy.append(state_kin_energy)
            list_rotationnal_energy.append(rotationnal_energy)
            list_translationnal_energy.append(translation_energy)

        return np.array(list_state_kin_energy),np.array(list_rotationnal_energy), np.array(list_translationnal_energy)

    
    def compute_kinetic_energy_metric(self,dataset):

        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Computing energy from proprioceptive sensors (wheel encoder)
        wheel_encoder_energies = self.compute_energy_from_wheel_encoder(dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"])

        diff_energies = self.compute_diff_proprio_extero_energy(dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"],
                                                dataset["gt_body_lin_vel"],dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, diff_energy, wheel_encoder_energy,gt_energy in zip(energy_order,diff_energies,wheel_encoder_energies,gt_energies):

                kinetic_metric = diff_energy/wheel_encoder_energy
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy
    
    def compute_all_terrain(self,dataset,multiple_terrain=False):

        new_file =False
        list_df = []
        for terrain in dataset.terrains:
            
            dico_data = dataset[terrain]
            shape = dico_data["cmd_left_wheel"].shape
            result_terrain = self.compute_kinetic_energy_metric(dico_data)
        
            result_terrain["cmd_left_wheel"] = dico_data["cmd_left_wheel"]
            result_terrain["cmd_right_wheel"] = dico_data["cmd_right_wheel"]
            # Create all names
            new_dico = {}
            list_col = list(result_terrain.keys()) 
            results_dict_1_name_by_column = create_columns_names_from_dict_with_names(list_col,result_terrain,new_dico)
            # Add identifying columns 
            for key,value in dico_data["col_2_multiply_and_add"].items():
                results_dict_1_name_by_column[key] = [value] * shape[0]
            
            results_dict_1_name_by_column["terrain"] = [terrain] * shape[0]
            list_df.append(pd.DataFrame.from_dict(results_dict_1_name_by_column))

        df_all_terrain = pd.concat(list_df)

        # Get the current date and time
        self.results_file[self.metric_name] = {dataset.id:df_all_terrain}

        with open(PATH_TO_RESULT_FILE, 'wb') as file:
            results_file = pickle.dump(self.results_file,file)
        

        
        #self.saving_path = path_to_dataset_folder


class GraphMetric():

    def __init__(self, dataset,metric) -> None:
        
        self.dataset = dataset
        self.path_to_results =  PATH_TO_RESULT_FILE
        self.metric = metric
        
        with open(self.path_to_results, 'rb') as file:  # Open the file in binary read mode
            self.metric_results = pickle.load(file)

    def graph_metric_boxplot_by_terrain(self,percentage=True):



        ## Ravel results
        data_all = self.metric_results[self.metric.metric_name][dataset.id]
        #print_column_unique_column(data_all)
        data_axs = []
        list_terrain = list(data_all.terrain.unique())
        
        list_metric = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
        nb_submetric =3

        for metric in list_metric: 

            dico_metric = {}
            for terrain in list_terrain:

                df_local = data_all.loc[data_all.terrain == terrain]

                dico_metric[terrain] = column_type_extractor(df_local,metric)
            data_axs.append(dico_metric)

        
        
        i = 0 
        fig, axs = plt.subplots(nb_submetric,1)
        fig.suptitle(f"{self.metric.metric_name}")
        fig.subplots_adjust(hspace=0.4,wspace=0.4)
        fig.set_figheight(6)
        fig.set_figwidth(6)
        for data in data_axs:
        
            ###### PLot one ax
            ax= axs[i]
            
            # Prepare data for boxplot
            
            medians = {terrain: np.median(results) for terrain, results in data.items()}

            sorted_terrains = sorted(medians.items(), key=lambda item: item[1])

            
            # Prepare data for boxplot in sorted order
            sorted_labels = [terrain for terrain, _ in sorted_terrains]
            if percentage:
                sorted_results = [np.ravel(data[terrain])*100 for terrain in sorted_labels]
            else:
                sorted_results = [np.ravel(data[terrain]) for terrain in sorted_labels]
            # Define colors for each box
            
            color_dict = {"asphalt":"lightgrey", "ice":"aliceblue","gravel":"papayawhip","grass":"honeydew","sand":"darkgoldenrod"}
            
            colors = [color_dict[terrain] for terrain in sorted_labels]

            

            # Create box plot
            box = ax.boxplot(sorted_results, labels=sorted_labels, patch_artist=True,showfliers=False)

            # Set colors for each box
            for patch, color in zip(box['boxes'], colors):
                patch.set_facecolor(color)

            # Set title and labels
            ax.set_title('Box Plot of Results by Terrain (Sorted by Median)')
            ax.set_ylabel(f'{list_metric[i]} [%]')


            
            # Show the plot

            i+=1
        plt.show()

    def graph_correlation(self,prediction_model_error):


        ## Ravel results
        data_all = self.metric_results[self.metric.metric_name][dataset.id]
        #print_column_unique_column(data_all)
        data_axs = []
        list_terrain = list(data_all.terrain.unique())
        
        list_metric = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
        nb_submetric =3


        for metric in list_metric: 

            dico_metric = {}
            for terrain in list_terrain:

                df_local = data_all.loc[data_all.terrain == terrain]

                dico_metric[terrain] = column_type_extractor(df_local,metric)
            data_axs.append(dico_metric)

        ## 
        fig, axs = plt.subplots(2,1)



class KineticEnergyMetricOnly(KineticEnergyMetricWheelEncoder):

    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)

        self.metric_name = "KineticEnergyMetricOnly"

        

    def compute_kinetic_energy_metric(self,dataset):

        gt_energies = self.compute_energy(dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"])
        
        # Assuming instant acceleration
        y_cmd =np.zeros(dataset["gt_body_lin_vel"].shape)
        idd_energies = self.compute_energy(dataset["cmd_body_lin_vel"],
                                                y_cmd,
                                                dataset["cmd_body_yaw_vel"])

        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name, idd_energy in zip(energy_order,idd_energies):

                kinetic_metric = idd_energy    
                
                
                resulting_energy[energy_name] = kinetic_metric

        return resulting_energy
    




class SlopeMetric(KineticEnergyMetricWheelEncoder):

    def __init__(self,metric_name,robot_name) -> None:
        super().__init__(metric_name,robot_name)
        self.robot_name = robot_name
        self.metric_name = "SlopeMetric"
        self.joule_treshold = self.metric_parameters['joule_treshold']
        self.steady_state_only = self.metric_parameters['steady_state_only']
        self.mean_the_steady_state = self.metric_parameters['mean_the_steady_state']

    def compute_average_slope(self,x,y,joules_treshold=500.0, n_steady_state=40,compensation_on=False):
        
        
        #mask_x = (x > np.percentile(x,2.5)) & (x < np.percentile(x,97.5))
        #mask_y = (y > np.percentile(y,2.5)) & (y < np.percentile(y,97.5))
        
        compensation_to_use = self.translationnal_compensation_array
        if self.steady_state_only:
            x = x[:,-n_steady_state:]
            y = y[:,-(n_steady_state):]

            
        if self.mean_the_steady_state:
            x = np.mean(x,axis=1)
            y = np.mean(y,axis=1)
            
        mask = np.abs(x)>=joules_treshold

        #mask = mask_x| mask_y
        #x_masked = x[mask]
        #y_masked = y[mask]
        if compensation_on:
            y = y
            y_masked = y

        else:
            y_masked = y[:,1:]
        x_masked = x[:,:-1] 
        
        m_slope = y_masked/x_masked
        
        m_slope_masked = m_slope[m_slope<np.percentile(m_slope,95)]
        mean_slope = np.median(m_slope_masked)
        std_slope = np.std(m_slope_masked)
        print("start")
        if compensation_on:
            metric_raw = 4/np.pi * np.power(np.abs(np.arctan2(y,x[:,:-1]) - np.pi/4 ),1)
        else:    
            metric_raw = 4/np.pi * np.power(np.abs(np.arctan2(y[:,1:],x[:,:-1]) - np.pi/4 ),1)
        print("fini")
        metric_mean = np.median(metric_raw) # 
        std_metric = np.std(metric_raw)
        x_95 = np.percentile(x_masked,95)

        
        return m_slope,mean_slope,std_slope,metric_mean,std_metric, metric_raw,x_95,y_masked,x_masked

    def filter_contamination(self,df_energy_cmd,terrain,treshold):

        df = df_energy_cmd.loc[df_energy_cmd.terrain==terrain]

        rotationnal_cmd = np.mean(extract_ss_based_on_ratio(df,"rotationnal_energy_metric"),axis=1)
        translationnal_cmd = np.mean(extract_ss_based_on_ratio(df,"translationnal_energy_metric"),axis=1)
        total_energy_cmd = np.mean(extract_ss_based_on_ratio(df,"total_energy_metric"),axis=1)
        mask_rotationnal = (translationnal_cmd/total_energy_cmd) < treshold
        mask_translationnal =  (rotationnal_cmd/total_energy_cmd)  < treshold

        return mask_rotationnal, mask_translationnal

    def n_rows_filter(self,list_col,n_rows):
        
        filtered_cols = []
        for col in list_col:
            if n_rows != -1:
                col.reset_index(inplace=True)
                col[:n_rows]

            filtered_cols.append(col)
        return filtered_cols

    def compute_slope_metric(self,dataset, gt_energies, idd_energies,debug=False,n_steady_state=40,x_energy_type="cmd" ):
        
        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            resulting_energy['steady_state_only'] = self.steady_state_only
            resulting_energy['mean_the_steady_state'] = self.mean_the_steady_state
            
            metric_energy_raw = {}
            metric_scatter = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name,gt_energy, idd_energy in zip(energy_order,gt_energies,idd_energies):


                if energy_name == "total_energy_metric":
                    translation_energy = gt_energies[2]
                    rotationnal_energy = gt_energies[1]

                    total_energy_compensated =  translation_energy[:,1:] * self.translationnal_compensation_array + rotationnal_energy[:,1:] * self.rotationnal_compensation_array
                    
                    m_slope,mean_slope,std_slope,metric,std_metric, metric_raw,x_95,y_maksed,x_masked = self.compute_average_slope(idd_energy,total_energy_compensated ,
                                                                                                joules_treshold=self.joule_treshold,
                                                                                                n_steady_state = n_steady_state,
                                                                                                compensation_on=True)
                
                    if self.steady_state_only:
                        y_maksed = gt_energy[:,-(n_steady_state-1):]
                    else:
                        y_maksed = gt_energy[:,1:]
                    # makes sure tha the total energy saved is the real energy metric and not the affected one.
                    
                    
                else:
                    m_slope,mean_slope,std_slope,metric,std_metric, metric_raw,x_95,y_maksed,x_masked = self.compute_average_slope(idd_energy,gt_energy ,
                                                                                            joules_treshold=self.joule_treshold,
                                                                                            n_steady_state = n_steady_state)
                
                resulting_energy["std_slope_" +energy_name] = std_slope
                resulting_energy["mean_slope_" +energy_name] = mean_slope
                resulting_energy["cmd_95_"+energy_name] = x_95
                resulting_energy["maximum_cmd_energy_"+energy_name] = np.max(idd_energy)
                resulting_energy["metric_"+energy_name] = metric
                resulting_energy["std_metric_"+energy_name] = std_metric
                #resulting_energy["metric_raw"+energy_name] = metric_raw

                metric_energy_raw[energy_name] = np.ravel(metric_raw) #np.mean(metric_raw,axis=1)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name] = np.ravel(metric_raw)
                metric_scatter["y_coordinates_"+energy_name] = np.ravel(y_maksed)
                metric_scatter[f"{x_energy_type}_"+energy_name] = np.ravel(x_masked)
                metric_scatter[f"{x_energy_type}_diff_icp_"+energy_name] = np.ravel(x_masked) - np.ravel(y_maksed) 

                if debug and energy_name=="total_energy_metric":
                    fig, ax = plt.subplots(1,1)
                    ax.hist(metric_raw,range=(0,1),bins=60,density=True)
                    y_lim = ax.get_ylim()
                    ax.vlines(np.median(metric_raw),ymin=y_lim[0],ymax=y_lim[1],label="median", color="red")
                    ax.vlines(np.mean(metric_raw),ymin=y_lim[0],ymax=y_lim[1],label="mean", color="green" )
                    ax.legend()
                    #plt.boxplot(metric_raw,showfliers=False)
                    print(x_energy_type)
                    print("________")
                    print("median",np.median(metric_raw))
                    print("mean",np.mean(metric_raw))
                    print("std",np.std(metric_raw))
                    #plt.title()
                    plt.show()
            resulting_energy["joule_treshold"] = self.joule_treshold
        
        return resulting_energy,metric_energy_raw,metric_scatter
            
    def compute_compensation_param(self, gt_speed, cmd_speed):
        """Compute the compensation array based on the gt_speed already_prefiltered with the correct amount of row. 

        Args:
            gt_speed (_type_): _description_
            cmd_speed (_type_): _description_
        """
        original_shape = gt_speed[0].shape
        original_shape = (original_shape[0],original_shape[1]-1)
        cmd_vector = np.array([np.ravel(cmd_speed[0][:,:-1]),np.ravel(cmd_speed[1][:,:-1])]).T
        gt_vector = np.array([np.ravel(gt_speed[0][:,1:]),np.ravel(gt_speed[1][:,1:])]).T

        factor_list = []
        for cmd_i_trans_speed, gt_i_trans_speed in zip(cmd_vector,gt_vector):

            dot_product = cmd_i_trans_speed @ gt_i_trans_speed.T

            cos_theta =  dot_product / (np.linalg.norm(cmd_i_trans_speed) * np.linalg.norm(gt_i_trans_speed))

            compensation_factor = (cos_theta + 1)/2

            if np.isnan(compensation_factor):
                compensation_factor = 1.0
            
            factor_list.append(compensation_factor)

        translationnal_compensation_array = np.array(factor_list).reshape(original_shape)


        cmd_rot = cmd_speed[2][:,:-1]
        gt_rot = gt_speed[2][:,1:]

        sign_to_classify = np.sign(cmd_rot * gt_rot)
        sign_to_classify = np.where(sign_to_classify <=0, np.zeros_like(sign_to_classify), sign_to_classify)
        sign_to_classify = np.where(np.isnan(sign_to_classify)==True, np.zeros_like(sign_to_classify),sign_to_classify)
        self.rotationnal_compensation_array = sign_to_classify
        
        
        self.translationnal_compensation_array = translationnal_compensation_array

    
    def compute_energy(self,vx,vy,omega_body):
        """_summary_

        Args:
            vx (array): assuming that the vector is N by 1
            vy (_type_): assuming that the vector is N by 1
            omega_body (_type_): assuming that the vector is N by 1
        """

        translation_energy = 1/2 * self.masse * (vx**2+vy**2) 
        rotationnal_energy = 1/2 * self.masse * (self.inertia_constraints * omega_body**2)
        state_kin_energy =  translation_energy + rotationnal_energy

        
        return state_kin_energy,rotationnal_energy, translation_energy
    
    def compute_kinetic_energy_metric(self,dataset,n_steady_state, n_rows=-1,debug=True):
        """Compute the kinetic energy metric of the terrain

        Args:
            dataset (_type_): Dataset containing all results of one vehicle on one terrain. 

        Returns:
            _type_: _description_
        """
        
        ### Extract kinematic gt
        gt_speed = self.n_rows_filter([dataset["gt_body_lin_vel"],
                                                dataset["gt_body_y_vel"],
                                                dataset["gt_body_yaw_vel"]],n_rows)
        ## Extract IDD
        y_cmd =np.zeros(dataset["cmd_body_lin_vel"].shape)
        cmd_speed = self.n_rows_filter([dataset["cmd_body_lin_vel"],
                                                y_cmd,
                                                dataset["cmd_body_yaw_vel"]],n_rows)


        self.compute_compensation_param(cmd_speed,gt_speed)
        
        gt_energies = self.compute_energy(gt_speed[0],gt_speed[1],gt_speed[2])   
        idd_energies = self.compute_energy(cmd_speed[0],cmd_speed[1],cmd_speed[2])      
        
        
        ## Extract Wheel
        columns3 = self.n_rows_filter([dataset["gt_left_wheel"],
                                                dataset["gt_right_wheel"],dataset["gt_body_y_vel"]],n_rows)
        wheel_encoder_energies = self.compute_energy_from_wheel_encoder(columns3[0],columns3[1],columns3[2])

        resulting_energy_cmd,metric_energy_raw_cmd,metric_scatter_cmd = self.compute_slope_metric(dataset,gt_energies, idd_energies,debug=False,n_steady_state=n_steady_state)
        
        resulting_energy_wheels, metric_energy_raw_wheels,metric_scatter_wheels = self.compute_slope_metric(dataset,gt_energies, wheel_encoder_energies,debug=False,n_steady_state=n_steady_state,x_energy_type="wheels")
        
        metric_scatter_cmd.update(metric_scatter_wheels)
        return resulting_energy_cmd,resulting_energy_wheels, metric_energy_raw_wheels,metric_energy_raw_cmd,metric_scatter_cmd
    
    def compute_all_terrain(self,dataset,multiple_terrain=False,n_rows=-1,list_lim_vel_x = [5.0],list_lim_vel_yaw=[5.0],save_video=True):

        new_file =False
        list_row = []
        list_row_encoder = []

        list_df_cmd = []
        list_df_wheels = [] 
        
        list_df_cmd_scatter = []
        i = 0
        for lim_vel_yaw, lim_vel_x in zip(list_lim_vel_yaw, list_lim_vel_x):
            print(i/(11*11))
            i+=1
            for terrain in dataset.terrains:
                
                dico_data = dataset.get_sub_sample(terrain,lim_vel_yaw,lim_vel_x)
                shape = dico_data["cmd_left_wheel"].shape
                result_terrain_cmd, result_terrain_encoder, metric_energy_raw_wheels,metric_energy_raw_cmd,metric_scatter_cmd= self.compute_kinetic_energy_metric(dico_data,dataset.datasets_info["n_steady_state"],n_rows=n_rows)

                
                result_terrain_cmd["lim_vel_yaw"] = lim_vel_yaw 
                result_terrain_cmd["lim_vel_x"] = lim_vel_x
                result_terrain_encoder["lim_vel_yaw"] = lim_vel_yaw
                result_terrain_encoder["lim_vel_x"] = lim_vel_x
                
                shape = metric_energy_raw_wheels["total_energy_metric"].shape[0]
                metric_energy_raw_wheels["terrain"] = [terrain] * shape
                metric_energy_raw_cmd["terrain"] = [terrain] * shape 
                
                if self.steady_state_only:
                    cmd_body_lin = np.ravel(dico_data["cmd_body_lin_vel"][:,-39:])
                    cmd_body_yaw = np.ravel(dico_data["cmd_body_yaw_vel"][:,-39:])

                else: 
                    cmd_body_lin = np.ravel(dico_data["cmd_body_lin_vel"][:,:-1])
                    cmd_body_yaw = np.ravel(dico_data["cmd_body_yaw_vel"][:,:-1])
                metric_energy_raw_cmd["cmd_body_lin_vel"] =  cmd_body_lin #np.ravel(dico_data["cmd_body_lin_vel"])
                metric_energy_raw_cmd["cmd_body_yaw_vel"] =  cmd_body_yaw #np.ravel(dico_data["cmd_body_yaw_vel"])
                
                metric_energy_raw_wheels["cmd_body_lin_vel"] = cmd_body_lin #np.ravel(dico_data["cmd_body_lin_vel"])#np.mean(dico_data["cmd_body_lin_vel"],axis=1)
                metric_energy_raw_wheels["cmd_body_yaw_vel"] = cmd_body_yaw #np.ravel(dico_data["cmd_body_yaw_vel"])#np.mean(dico_data["cmd_body_yaw_vel"],axis=1)
                
                metric_energy_raw_cmd["lim_vel_yaw"] = [lim_vel_yaw] * shape
                metric_energy_raw_cmd["lim_vel_x"] = [lim_vel_x] * shape
                metric_energy_raw_wheels["lim_vel_yaw"] = [lim_vel_yaw] * shape
                metric_energy_raw_wheels["lim_vel_x"] = [lim_vel_x] * shape

                df_cmd = pd.DataFrame.from_dict(metric_energy_raw_cmd)
                df_wheel = pd.DataFrame.from_dict(metric_energy_raw_wheels)

                shape2 = cmd_body_yaw.shape[0]
                metric_scatter_cmd["terrain"] = [terrain] * (shape2)

                metric_scatter_cmd["lim_vel_yaw"] = [lim_vel_yaw] * (shape2)
                metric_scatter_cmd["lim_vel_x"] = [lim_vel_x] * (shape2)
                
                list_df_cmd_scatter.append(pd.DataFrame.from_dict(metric_scatter_cmd))
                list_df_cmd.append(df_cmd)
                list_df_wheels.append(df_wheel)

                

                # Create all names
                dico_temp = {"terrain":terrain}
                dico_temp.update(result_terrain_cmd)
                list_row.append(dico_temp) 

                print(np.unique(terrain))
                dico_temp2 = {"terrain":terrain}
                dico_temp2.update(result_terrain_encoder)
                list_row_encoder.append(dico_temp2) 
                

        df_all_terrain = pd.DataFrame.from_records(list_row)
        df_all_terrain["nstep"] =[n_rows]*df_all_terrain.shape[0]
        print("lsit_row")
        df_all_terrain_2 = pd.DataFrame.from_records(list_row_encoder)
        df_all_terrain_2["robot"] = df_all_terrain_2.shape[0] * [self.robot_name]
        

        print("lsit_wheels")
        df_all_wheel = pd.concat(list_df_wheels,axis=0)
        df_all_cmd = pd.concat(list_df_cmd,axis=0)

        df_all_wheel["robot"] = df_all_wheel.shape[0] * [self.robot_name]
        df_all_cmd["robot"] = df_all_cmd.shape[0] * [self.robot_name]

        #self.saving_path = path_to_dataset_folder

        df_all_scatter_cmd = pd.concat(list_df_cmd_scatter,axis=0)
        df_all_scatter_cmd["robot"] = df_all_scatter_cmd.shape[0] * [self.robot_name]
        
        if save_video:
            df_all_terrain.to_csv(self.metric_parameters['path_to_save'][:-4]+f"_{self.robot_name}"+".csv")
            df_all_terrain_2.to_csv(self.metric_parameters['path_to_save'][:-4]+ f"_{self.robot_name}"+"_wheel_encoder.csv")
            df_all_wheel.to_csv(f"drive_datasets/results_multiple_terrain_dataframe/metric/{self.robot_name}_metric_wheels_raw_slope_metric.csv")
            df_all_cmd.to_csv(f"drive_datasets/results_multiple_terrain_dataframe/metric/{self.robot_name}_metric_cmd_raw_slope_metric.csv")
            df_all_scatter_cmd.to_csv(f"drive_datasets/results_multiple_terrain_dataframe/metric/{self.robot_name}_metric_cmd_raw_slope_metric_scatter.csv")


        
        return df_all_terrain

    def combpute_for_var_basewidth(self,width,length, dataset,terrain,n_rows=-1,lim_vel_yaw = 5.0,lim_vel_x=5.0,save_video=False):

        
        self.length = length
        self.width = width

        self.compute_intertia()
        dico_data = dataset.get_sub_sample(terrain,lim_vel_yaw,lim_vel_x)
        shape = dico_data["cmd_left_wheel"].shape                                                                   
        result_terrain_cmd, result_terrain_encoder, metric_energy_raw_wheels,metric_energy_raw_cmd,metric_scatter_cmd= self.compute_kinetic_energy_metric(dico_data,dataset.datasets_info["n_steady_state"],n_rows=n_rows)

        return metric_energy_raw_cmd

    def compute_all_terrain_variable_steps(self, dataset,multiple_terrain=False,list_lim_vel_x = [5.0],list_lim_vel_yaw=[5.0],n_division=51):
    


        list_df = [] 
        
        list_terrain = dataset.df.terrain.value_counts()


        nb_rows = np.linspace(0,100,n_division)

        for nrow in nb_rows:

            # Lire marsupial robotics : transporter :coordinator, leader, facilitate communication, transporter, supporter. 
            #
            list_df.append(self.compute_all_terrain(dataset,n_rows=nrow,list_lim_vel_x = list_lim_vel_x,list_lim_vel_yaw=list_lim_vel_yaw,save_video=False))

        df_combine = pd.concat(list_df,axis=0)

        df_combine.to_csv("drive_datasets/results_multiple_terrain_dataframe/metric/{self.robot_name}_steps_convergence.csv")


def recompute_results_for_watermelon_metric(df,robot,windo_to_use = "first_window"):

    column_to_keep = ["cmd_body_lin_vel", "cmd_body_yaw_vel","cmd_metric_total_energy_metric",
                      "cmd_total_energy_metric","cmd_rotationnal_energy_metric","cmd_translationnal_energy_metric"]
    column_to_keep_texte = ["terrain"]
    dico_translate = {"cmd_body_lin_vel":"cmd_body_x_mean",
                      "cmd_body_yaw_vel":"cmd_body_yaw_mean",
                      "cmd_metric_total_energy_metric":f"{windo_to_use}_metric",
                      "cmd_total_energy_metric":f"{windo_to_use}_cmd_total_energy_metric",
                      "cmd_rotationnal_energy_metric":f"{windo_to_use}_cmd_rotationnal_energy_metric",
                      "cmd_translationnal_energy_metric":f"{windo_to_use}_cmd_translationnal_energy_metric"}
    
    new_dico = {}
    for col in column_to_keep:

        extracted_col = df[col].to_numpy()
        length = extracted_col.shape[0]
        size1 = 119
        size0 = length//size1
        reshaped_extracted_col = extracted_col.reshape((size0,size1))

        if windo_to_use == "first_window":
            lw = np.mean(reshaped_extracted_col[:,:40],axis=1)
        elif windo_to_use == "last_window":
            lw = np.mean(reshaped_extracted_col[:,-39:],axis=1)
        new_dico[dico_translate[col]] = lw
    
    for col in column_to_keep_texte:

        extracted_col = df[col].to_numpy()
        length = extracted_col.shape[0]
        size1 = 119
        size0 = length//size1
        reshaped_extracted_col = extracted_col.reshape((size0,size1))

        lw = reshaped_extracted_col[:,60]
        new_dico[col] = lw
    

    df= pd.DataFrame.from_dict(new_dico)
    return df
    #df_warthog.to_csv("drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_to_watermelon.csv")
    #df.to_csv(f"drive_datasets/results_multiple_terrain_dataframe/metric/{robot}_{windo_to_use}_metric_to_watermelon.csv")
    

if __name__ == "__main__":

    

    dataset = Dataset2Evaluate("drive_dataset_warthog")
    
    
    dm0 = SlopeMetric("SlopeMetric","warthog")

    
    try_linspace = np.linspace(0.5,5.0,11)
    X,Y = np.meshgrid(try_linspace,try_linspace)
    list_lim_vel_x = np.ravel(X)
    list_lim_vel_yaw = np.ravel(Y)

    list_lim_vel_x = [5.0]
    list_lim_vel_yaw = [5.0]
    path_to_result = dm0.compute_all_terrain(dataset,list_lim_vel_x = list_lim_vel_x,list_lim_vel_yaw=list_lim_vel_yaw)

    dataset2 = Dataset2Evaluate("drive_dataset_husky")
    dm2 = SlopeMetric("SlopeMetric","husky")


    path_to_result = dm2.compute_all_terrain(dataset2,list_lim_vel_x = list_lim_vel_x,list_lim_vel_yaw=list_lim_vel_yaw)

    #dm = KineticEnergyMetric("kinetic_energy","warthog")
    #path_to_result = dm.compute_all_terrain(dataset)

    #graph_metric = GraphMetric(dataset,dm)
    #graph_metric.graph_metric_boxplot_by_terrain()
#
    #
    #dm2 = KineticEnergyMetricWheelEncoder("kinetic_energy_wheel_encoder","warthog")
    #path_to_result2 = dm2.compute_all_terrain(dataset)
    #graph_metric2 = GraphMetric(dataset,dm2)
    #graph_metric2.graph_metric_boxplot_by_terrain()
#
    #dm3 = KineticEnergyMetricWheelEncoderRatio("kinetic_energy_wheel_encoder_ratio","warthog")
    #path_to_result2 = dm3.compute_all_terrain(dataset)
    #graph_metric3 = GraphMetric(dataset,dm3)
    #graph_metric3.graph_metric_boxplot_by_terrain(percentage=True)
#
#
    #dm4 = DiffSpeedProprioExteroEnergy("DiffSpeedProprioExteroEnergy","warthog")
    #path_to_result2 = dm4.compute_all_terrain(dataset)
    #graph_metric3 = GraphMetric(dataset,dm4)
    #graph_metric3.graph_metric_boxplot_by_terrain(percentage=True)
    #
    #
    #dm5 = KineticEnergyICPOnly("KineticEnergyICPOnly","warthog")
    #path_to_result2 = dm5.compute_all_terrain(dataset)
    #graph_metric3 = GraphMetric(dataset,dm5)
    #graph_metric3.graph_metric_boxplot_by_terrain(percentage=True)
    #
    #dm6 = KineticEnergyWheelOnly("KineticEnergyWheelOnly","warthog")
    #path_to_result2 = dm6.compute_all_terrain(dataset)
    #graph_metric3 = GraphMetric(dataset,dm6)
    #graph_metric3.graph_metric_boxplot_by_terrain(percentage=True)
    #
#
    #print("start")
    #dm7 = KineticEnergyMetricOnly("KineticEnergyMetricOnly","warthog")
    #path_to_result2 = dm7.compute_all_terrain(dataset)
    #print("start")
    #graph_metric3 = GraphMetric(dataset,dm7)
    #graph_metric3.graph_metric_boxplot_by_terrain(percentage=True)
    
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric_scatter.csv"
    df_warthog_metric = pd.read_csv(path_to_raw_result)
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric_scatter.csv"
    df_husky_metric = pd.read_csv(path_to_raw_result)
    
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric.csv"
    df_warthog_vels = pd.read_csv(path_to_raw_result)
    path_to_raw_result = "drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_cmd_raw_slope_metric.csv"
    df_husky_vels = pd.read_csv(path_to_raw_result)
    
    df_warthog = pd.concat([df_warthog_metric, df_warthog_vels["cmd_body_lin_vel"], df_warthog_vels["cmd_body_yaw_vel"]], axis=1)
    df_husky = pd.concat([df_husky_metric, df_husky_vels["cmd_body_lin_vel"], df_husky_vels["cmd_body_yaw_vel"]], axis=1)

    fw_warthog = recompute_results_for_watermelon_metric(df_warthog,"warthog",windo_to_use = "first_window")
    lw_warthog = recompute_results_for_watermelon_metric(df_warthog,"warthog",windo_to_use = "last_window")
    
    fw_husky = recompute_results_for_watermelon_metric(df_husky,"husky",windo_to_use = "first_window")
    lw_husky = recompute_results_for_watermelon_metric(df_husky,"husky",windo_to_use = "last_window")
    
    lw_warthog["first_window_metric"] = fw_warthog[f"first_window_metric"]
    lw_husky["first_window_metric"] = fw_husky[f"first_window_metric"]
    
    lw_warthog.to_csv("drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_to_watermelon.csv")
    lw_husky.to_csv("drive_datasets/results_multiple_terrain_dataframe/metric/husky_metric_to_watermelon.csv")
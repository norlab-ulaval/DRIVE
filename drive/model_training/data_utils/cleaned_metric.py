import numpy as np 
import pandas as pd 
import pathlib 
import yaml
from drive.model_training.data_utils.extractors import *
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive
from drive.model_training.models.kinematic.ideal_diff_drive_pwrtrain import Ideal_diff_drive_bounded

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
                                "SlopeMetric","SlopePWRtrainMetric"]
        
        if not PATH_TO_RESULT_FILE.is_file():
            empty_dict  = {}
            for metric in list_possible_metric:
                empty_dict[metric] = {}
            with open(PATH_TO_RESULT_FILE, 'wb') as file:
                results_file = pickle.dump(empty_dict,file)
        with open(PATH_TO_RESULT_FILE, 'rb') as file:
            self.results_file = pickle.load(file)



class robot():
    def __init__(self, robot_name) -> None:
        path_2_robot = "drive/model_training/data_utils/robot_param.yaml"
        with open(path_2_robot, 'r') as file:
            robot_param = yaml.safe_load(file)
        self.__dict__ = robot_param[robot_name]


class SlopeMetric():

    def __init__(self,robot,motion_model,joule_treshold=0.001,steady_state_only=False,mean_the_steady_state=False) -> None:
        
        self.motion_model = motion_model
        self.joule_treshold = joule_treshold
        self.steady_state_only = steady_state_only
        self.mean_the_steady_state = mean_the_steady_state
        
        self.metric_name = "SlopeMetric"
        
        self.robot = robot
        self.compute_intertia()

    def compute_intertia(self):
        self.inertia_constraints = (self.width**2 + self.length**2)/12    

    def compute_energy_from_wheel_encoder(self,left_wheel_encoder,right_wheel_encoder,vy_array):
        """Compute the energy of equivalent from the wheel encoder motion predicted by the IDD
        Args:
            left_wheel_encoder (_type_): _description_
            right_wheel_encoder (_type_): _description_

        Returns:
            _type_: _description_
        """
        # Compute the vx, vy equivalent speed encoder 
        u_wheel = np.array([left_wheel_encoder,right_wheel_encoder])
        body_cmd = self.robot.jacobian @ u_wheel

        state_kin_energy,rotationnal_energy, translation_energy = self.compute_energy(body_cmd[0,1],0.0,body_cmd[0,1])

        return state_kin_energy,rotationnal_energy, translation_energy

    def compute_energy(self,vx,vy,omega_body):
        """_summary_

        Args:
            vx (array): assuming that the vector is N by 1
            vy (_type_): assuming that the vector is N by 1
            omega_body (_type_): assuming that the vector is N by 1
        """

        translation_energy = 1/2 * self.robot.masse * (vx**2+vy**2) 
        rotationnal_energy = 1/2 * self.robot.masse * (self.inertia_constraints * omega_body**2)
        state_kin_energy =  translation_energy + rotationnal_energy

        return state_kin_energy,rotationnal_energy, translation_energy
    
    def compute_kinetic_energy_metric(self,gt_body_vels,cmd_body_vels,debug=True):
        """Compute the kinetic energy metric of the terrain

        Args:
            dataset (_type_): Dataset containing all results of one vehicle on one terrain. 

        Returns:
            _type_: _description_
        """
        
        
        ### réécrire cette partie pour qu'elle soit plus claire enlève toute partie vectorielle. 
        
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
   
    def compute_metrix_simplified():

        total_energy_compensated =  translation_energy[:,1:] * self.translationnal_compensation_array + rotationnal_energy[:,1:] * self.rotationnal_compensation_array
                    
        if dataset["format"] == "n_cmd x horizon":
            resulting_energy = {}
            resulting_energy['steady_state_only'] = self.steady_state_only
            resulting_energy['mean_the_steady_state'] = self.mean_the_steady_state
            
            metric_energy_raw = {}
            metric_scatter = {}
            energy_order = ["total_energy_metric","rotationnal_energy_metric","translationnal_energy_metric"]
            
            for energy_name,gt_energy, idd_energy in zip(energy_order,gt_energies,idd_energies):

                # state_kin_energy,rotationnal_energy, translation_energy
                if energy_name == "total_energy_metric":
                    translation_energy = gt_energies[2]
                    rotationnal_energy = gt_energies[1]

                    total_energy_compensated =  translation_energy[:,1:] * self.translationnal_compensation_array + rotationnal_energy[:,1:] * self.rotationnal_compensation_array
                    
                    m_slope,mean_slope,std_slope,metric,std_metric, metric_raw,x_95,y_maksed,x_masked = self.compute_average_slope(idd_energy,total_energy_compensated ,
                                                                                                joules_treshold=self.joule_treshold,
                                                                                                n_steady_state = n_steady_state,
                                                                                                compensation_on=True)
                    
                    
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_translationnal_j_components"] = np.ravel(translation_energy[:,1:])
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_translationnal_weights"] = np.ravel(self.translationnal_compensation_array)
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_j_components"] = np.ravel(rotationnal_energy[:,1:])
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_weights"] = np.ravel(self.rotationnal_compensation_array)
                    
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
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_translationnal_j_components"] = np.ravel(translation_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_translationnal_weights"] = np.ravel(self.translationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_j_components"] = np.ravel(rotationnal_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_weights"] = np.ravel(self.rotationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_total_weighted"] = np.ravel(translation_energy[:,1:]) *np.ravel(self.translationnal_compensation_array)+ \
                                                                                    np.ravel(rotationnal_energy[:,1:])*np.ravel(self.rotationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_total"] = np.ravel(translation_energy[:,1:]) +  np.ravel(rotationnal_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_idd_rotationnal_j"] = np.ravel(idd_energies[1][:,:-1])
                metric_scatter[f"{x_energy_type}_metric_idd_translationnal_j"] = np.ravel(idd_energies[2][:,:-1])
                metric_scatter[f"{x_energy_type}_metric_idd_total_j"] = np.ravel(idd_energies[0][:,:-1])
            
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
            
            metric_energy_raw[f"{x_energy_type}_metric_idd_rotationnal_j"] = np.ravel(idd_energies[1][:,:-1])
            metric_energy_raw[f"{x_energy_type}_metric_idd_translationnal_j"] = np.ravel(idd_energies[2][:,:-1])
            metric_energy_raw[f"{x_energy_type}_metric_idd_total_j"] = np.ravel(idd_energies[0][:,:-1])
            
            metric_energy_raw[f"gt_body_lin_vel"] = np.ravel(dataset["gt_body_lin_vel"][:,:-1])
            metric_energy_raw[f"gt_body_yaw_vel"] = np.ravel(dataset["gt_body_yaw_vel"][:,:-1])
            metric_energy_raw[f"gt_body_y_vel"] = np.ravel(dataset['gt_body_y_vel'][:,:-1])
             
        return resulting_energy,metric_energy_raw,metric_scatter


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

                # state_kin_energy,rotationnal_energy, translation_energy
                if energy_name == "total_energy_metric":
                    translation_energy = gt_energies[2]
                    rotationnal_energy = gt_energies[1]

                    total_energy_compensated =  translation_energy[:,1:] * self.translationnal_compensation_array + rotationnal_energy[:,1:] * self.rotationnal_compensation_array
                    
                    m_slope,mean_slope,std_slope,metric,std_metric, metric_raw,x_95,y_maksed,x_masked = self.compute_average_slope(idd_energy,total_energy_compensated ,
                                                                                                joules_treshold=self.joule_treshold,
                                                                                                n_steady_state = n_steady_state,
                                                                                                compensation_on=True)
                    
                    
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_translationnal_j_components"] = np.ravel(translation_energy[:,1:])
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_translationnal_weights"] = np.ravel(self.translationnal_compensation_array)
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_j_components"] = np.ravel(rotationnal_energy[:,1:])
                    metric_energy_raw[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_weights"] = np.ravel(self.rotationnal_compensation_array)
                    
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
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_translationnal_j_components"] = np.ravel(translation_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_translationnal_weights"] = np.ravel(self.translationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_j_components"] = np.ravel(rotationnal_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_rotationnal_weights"] = np.ravel(self.rotationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_total_weighted"] = np.ravel(translation_energy[:,1:]) *np.ravel(self.translationnal_compensation_array)+ \
                                                                                    np.ravel(rotationnal_energy[:,1:])*np.ravel(self.rotationnal_compensation_array)
                metric_scatter[f"{x_energy_type}_metric_"+energy_name+"_total"] = np.ravel(translation_energy[:,1:]) +  np.ravel(rotationnal_energy[:,1:])
                metric_scatter[f"{x_energy_type}_metric_idd_rotationnal_j"] = np.ravel(idd_energies[1][:,:-1])
                metric_scatter[f"{x_energy_type}_metric_idd_translationnal_j"] = np.ravel(idd_energies[2][:,:-1])
                metric_scatter[f"{x_energy_type}_metric_idd_total_j"] = np.ravel(idd_energies[0][:,:-1])
            
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
            
            metric_energy_raw[f"{x_energy_type}_metric_idd_rotationnal_j"] = np.ravel(idd_energies[1][:,:-1])
            metric_energy_raw[f"{x_energy_type}_metric_idd_translationnal_j"] = np.ravel(idd_energies[2][:,:-1])
            metric_energy_raw[f"{x_energy_type}_metric_idd_total_j"] = np.ravel(idd_energies[0][:,:-1])
            
            metric_energy_raw[f"gt_body_lin_vel"] = np.ravel(dataset["gt_body_lin_vel"][:,:-1])
            metric_energy_raw[f"gt_body_yaw_vel"] = np.ravel(dataset["gt_body_yaw_vel"][:,:-1])
            metric_energy_raw[f"gt_body_y_vel"] = np.ravel(dataset['gt_body_y_vel'][:,:-1])
             
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

            if np.isnan(compensation_factor): # The way we deal our exception might not be good
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


from drive_custom_srv.srv import TrainMotionModel
import rclpy
from rclpy.node import Node
### Import message type 
from std_msgs.msg import String


### Import for the motion model trianing
import pandas as pd
import numpy as np

import argparse
import os
from time import gmtime, strftime,time

from drive.model_training.data_utils.dataset_parser import DatasetParser
from drive.model_training.data_utils.slip_dataset_parser import SlipDatasetParser
# from data_utils.acceleration_dataset_parser import AccelerationDatasetParser

from drive.model_training.model_trainers.powertrain_trainer import Powertrain_Trainer
from drive.model_training.models.powertrain.bounded_powertrain import Bounded_powertrain

from drive.model_training.model_trainers.blr_slip_trainer import SlipBLRTrainer

from rclpy.qos import qos_profile_action_status_default
from drive_custom_srv.msg import PathTree
import time
import pathlib
import yaml
from shutil import copy
from ament_index_python.packages import get_package_share_directory

class MotionModelTrainerNode(Node):

    def __init__(self):
        super().__init__('model_trainer_node')
        # Load the gui message 
        self.path_to_share_directory = pathlib.Path(get_package_share_directory('drive'))
        path_to_gui_message = self.path_to_share_directory.parent.parent.parent.parent/'src'/'DRIVE'/'drive'/'gui_message.yaml'
        with open(str(path_to_gui_message),'r') as f:
            self.gui_message = yaml.safe_load(f)["gui_message"]
        
        
        # Default value
        self.declare_parameters(
            namespace='',
            parameters=[
                ('run_by_maestro',False),
                ('path_to_input_space_calib',"Not defined"),
                ('path_to_calibration_node_config_file',"Not defined"),
                ("path_to_calibration_data_raw","Not defined"),
                ('imu_inverted',False),
                ('training_horizon', 2.0),
                ('pwrtrain_model_config_path','not_defined'),
                ('model_results_path',"Not defined"),
                ('slip_BLR_model_config_path',"Not_defined"),
                ('path_to_slip_dataset_all',"Not defined")
            ]
        )
        self.run_by_maestro = self.get_parameter('run_by_maestro').get_parameter_value().bool_value
        self.path_to_input_space_calib = self.get_parameter('path_to_input_space_calib').get_parameter_value().string_value
        self.path_to_calibration_node_config_file = self.get_parameter('path_to_calibration_node_config_file').get_parameter_value().string_value
        self.path_to_calibration_data_raw = self.get_parameter('path_to_calibration_data_raw').get_parameter_value().string_value
        self.path_to_torch_ready_df = str(pathlib.Path(self.path_to_calibration_data_raw).parent/'torch_ready_dataframe.pkl')
        self.get_logger().info(str(self.path_to_torch_ready_df))
        self.imu_inverted = self.get_parameter('imu_inverted').get_parameter_value().bool_value
        self.training_horizon =  self.get_parameter('training_horizon').get_parameter_value().double_value
        #self.pwrtrain_model_config_path = self.get_parameter('pwrtrain_model_config_path').get_parameter_value().string_value
        self.path_model_training_results = pathlib.Path(self.get_parameter('model_results_path').get_parameter_value().string_value)
        #self.slip_BLR_model_config_path = self.get_parameter('slip_BLR_model_config_path').get_parameter_value().string_value
        self.path_to_share_directory = pathlib.Path(get_package_share_directory('drive'))
        path_to_motion_model_training_params_folder = self.path_to_share_directory.parent.parent.parent.parent/'src'/'DRIVE'/'motion_model_available'
        self.pwrtrain_model_config_path = path_to_motion_model_training_params_folder/ '_pwrtain_motion_model_parameters.yaml'
        self.slip_BLR_model_config_path = path_to_motion_model_training_params_folder/ '_slip-BLR_motion_model_parameters.yaml'
        self.path_to_slip_dataset_all = self.get_parameter('path_to_slip_dataset_all').get_parameter_value().string_value


        self.drive_maestro_status = "no_status_received_yet"
        self.srv = self.create_service(TrainMotionModel, 'train_motion_model', self.train_motion_model)
        
        # self.motion_model_parameters
        self.training_horizon =  self.get_parameter('training_horizon').get_parameter_value().double_value
        
        # Debug
        #self.run_by_maestro = False
        if self.run_by_maestro:
                ## Subscription
            self.exp_path_sub = self.create_subscription(
                PathTree,
                'experiment_data_paths',
                self.experiment_path_callback,
                qos_profile_action_status_default)

            self.drive_maestro_status_sub = self.create_subscription(
                String,
                'maestro_status',
                self.drive_maestro_status_callback,
                10)

        else:
            self.get_logger().info(str(self.path_to_input_space_calib))
            input_space_df = pd.read_pickle(self.path_to_input_space_calib)
            self.wheel_radius = input_space_df['calibrated_radius [m]'][0]
            self.baseline = input_space_df['calibrated_baseline [m]'][0] 
            self.max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
            self.min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]

            # Load param of the calibration node
            with open(self.path_to_calibration_node_config_file, 'r') as file:
                prime_service = yaml.safe_load(file)
                param_dict = prime_service["/drive/calibration_node"]["ros__parameters"]
                self.rate = param_dict["cmd_rate"]
                self.calib_step_time = param_dict["step_len"]
                #param_dict[""] #TODO: modifié drive pour que training horizon et step_time_len peut être différent de 6. 
                
            
        self.is_torch_data_all_processed = False
        self.is_slip_blr_processed = False
        self.is_pwrtrain_processed = False

        self.model_trainer_node_status_msg =String() 
        self.model_trainer_node_status_msg.data = "No_model_trained"
        self.model_trainer_node_status_pub = self.create_publisher(String, 'model_trainer_node_status', 10)
        self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))

        self.timer_dict = {}
    def experiment_path_callback(self,experiment_path_msg):

        self.path_to_input_space_calib = str(pathlib.Path(experiment_path_msg.path_model_training_datasets)/"input_space_data.pkl")
        self.path_to_calibration_data_raw = str(pathlib.Path(experiment_path_msg.path_model_training_datasets)/"raw_dataframe.pkl")
        self.path_to_calibration_node_config_file = experiment_path_msg.path_to_calibration_node_config
        self.path_to_torch_ready_df = str(pathlib.Path(experiment_path_msg.path_model_training_datasets)/'torch_ready_dataframe.pkl')
        self.path_to_slip_dataset_all = str(pathlib.Path(experiment_path_msg.path_model_training_datasets)/'slip_dataset_all.pkl')
        self.path_config_folder = pathlib.Path(experiment_path_msg.path_config_folder)
        self.path_model_training_results = pathlib.Path(experiment_path_msg.path_model_training_results)
        

    def drive_maestro_status_callback(self,drive_maestro_status_msg):
        self.drive_maestro_status = drive_maestro_status_msg.data
    
    def post_process_dataframe(self):
        """Take the DF from the drive logger node and create the torch_ready_dataframe.
        """
        start = time.time()
        dataset_parser = DatasetParser(self.path_to_calibration_data_raw, self.path_to_torch_ready_df, self.training_horizon,
                                    self.rate, self.calib_step_time, self.wheel_radius, self.baseline, self.imu_inverted)
        self.parsed_dataframe = dataset_parser.process_data()
        self.is_torch_data_all_processed = True

        

        end = time.time()
        self.timer_dict["processing_raw_data_time"] = end-start

        # Processing the slip_blr_datasets
        self.get_logger().info(str(self.path_model_training_results))
        slip_dataset_parser = SlipDatasetParser(self.parsed_dataframe, self.path_model_training_results, self.wheel_radius, self.baseline, self.mean_min_vel, self.mean_max_vel, self.rate)
        self.slip_dataset_df = slip_dataset_parser.append_slip_elements_to_dataset()
        self.slip_dataset_df.to_pickle(self.path_to_slip_dataset_all)

        self.timer_dict["processing_slip_blr_data"] = time.time()-end


    def train_pwrtrain_motion_model(self):
        """Train the powertrain_motion_model 
        """
        start = time.time()
        self.model_trainer_node_status_msg.data = "training_pwrtrain_model"
        self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))
        # Create the folder needed to saved the results 
        #### Todo better
        path_to_save_results = self.path_model_training_results/"powertrain"
        if path_to_save_results.exists() == False:
            path_to_save_results.mkdir()
        path_to_save_results = path_to_save_results/f"{self.dataframe_stop_index}_steps_of_{self.calib_step_time}"
        if path_to_save_results.exists() == False:
            path_to_save_results.mkdir()
        #### To do better
        # Load param of the calibration node
        with open(self.pwrtrain_model_config_path, 'r') as file:
            motion_model_trainer_config = yaml.safe_load(file)
            self.pwrtrain_param_dict= motion_model_trainer_config["pwrtrain_motion_model"]
    
        init_param_dict = self.pwrtrain_param_dict["init_params"]
        init_params = [init_param_dict["time_constant"], init_param_dict["time_delay"]]
        
        bounds_dict = self.pwrtrain_param_dict["minimizer_bounds"]

        bounds = [(bounds_dict["time_constant"][0], bounds_dict["time_constant"][1]),
                (bounds_dict["time_delay"][0], bounds_dict["time_delay"][1])]
        
        method = self.pwrtrain_param_dict["optimizer_methods"]

        timesteps_per_horizon = self.training_horizon * self.rate
        dt = 1/self.rate
        
        # Train the motion model 
        bounded_powertrain = Bounded_powertrain(self.min_wheel_vel, self.max_wheel_vel, time_constant=0.5, time_delay=dt, dt=dt)
        
        self.parsed_dataframe_reduced_size = self.parsed_dataframe.iloc[:self.dataframe_stop_index-1]

        powertrain_trainer = Powertrain_Trainer(powertrain_model=bounded_powertrain, init_params=init_params, dataframe=self.parsed_dataframe_reduced_size,
                                    timesteps_per_horizon=timesteps_per_horizon, dt=dt)
        
        left_training_result, right_training_result = powertrain_trainer.train_model(init_params=init_params, method=method, bounds=bounds)


        left_min_vel, left_max_vel, right_min_vel, right_max_vel = powertrain_trainer.find_max_wheel_vels()

        self.mean_min_vel = (left_min_vel + right_min_vel) / 2
        self.mean_max_vel = (left_max_vel + right_max_vel) / 2

        # Save the results
        left_side_saved_params = path_to_save_results /'powertrain_training_left.npy'
        #os.makedirs(os.path.dirname(left_side_saved_params), exist_ok=True)
        np.save(left_side_saved_params, left_training_result)
        
        right_side_saved_params = path_to_save_results /'powertrain_training_right.npy'
        
        np.save(right_side_saved_params, right_training_result)
        
        self.model_trainer_node_status_msg.data = "finished_training_pwrtrain_model"
        self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))

        end = time.time()
        self.timer_dict["powertrain_model_training_time"] = end-start
        return str(path_to_save_results)

    def train_slip_blr_model_motion_model(self):
        """Train the powertrain_motion_model 
        """
        start = time.time()
        self.model_trainer_node_status_msg.data = "training_slip_blr_model"
        self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))
        # Create the folder needed to saved the results 
        #### Todo better
        path_to_save_results = self.path_model_training_results/"slip_blr"
        if path_to_save_results.exists() == False:
            path_to_save_results.mkdir()
        path_to_save_results = path_to_save_results/f"{self.dataframe_stop_index}_steps_of_{self.calib_step_time}"
        if path_to_save_results.exists() == False:
            path_to_save_results.mkdir()
        #### To do better

        slip_dataset_df_reduced_size = self.slip_dataset_df.iloc[:self.dataframe_stop_index]
        
        
        
        ### Saved dataset
        blr_slip_trainer = SlipBLRTrainer(slip_dataset_df_reduced_size, self.wheel_radius, self.baseline, self.rate)
        trained_blr_slip_model = blr_slip_trainer.train_blr_model()
        self.get_logger().info(str(path_to_save_results))
        #self.get_logger().info('weights_x : '+ str(trained_blr_slip_model.body_x_slip_blr.weights))
        #self.get_logger().info('weights_y : '+ str(trained_blr_slip_model.body_y_slip_blr.weights))
        #self.get_logger().info('weights_yaw : '+ str(trained_blr_slip_model.body_yaw_slip_blr.weights))

        trained_blr_slip_model.save_params(path_to_save_results)

        self.model_trainer_node_status_msg.data = "finished_training_slip_blr_model"
        self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))
        
        end = time.time()
        self.timer_dict["slip_blr_model_training_time"]= end-start
        return str(path_to_save_results)
    
    def copy_config_file(self):

        if self.run_by_maestro:
            path_pwrt_train_config_parts = pathlib.Path(self.pwrtrain_model_config_path).parts
            path_slip_blr_config_parts = pathlib.Path(self.slip_BLR_model_config_path).parts
            copy(str(self.pwrtrain_model_config_path),str(self.path_config_folder/path_slip_blr_config_parts[-1]))
            copy(str(self.slip_BLR_model_config_path),str(self.path_config_folder/path_slip_blr_config_parts[-1]))

    def save_training_time(self):

        df = pd.DataFrame.from_dict(self.timer_dict,orient="index",columns=["time_s"])
        df.to_pickle(str(self.path_model_training_results/"training_time.pkl"))

    def train_motion_model(self, request, response):
        
        
        if self.run_by_maestro:
            # Load param of the calibration node
            with open(self.path_to_calibration_node_config_file, 'r') as file:
                prime_service = yaml.safe_load(file)
                param_dict = prime_service["/drive/calibration_node"]["ros__parameters"]
                self.rate = param_dict["cmd_rate"]
                self.calib_step_time = param_dict["step_len"]
                #param_dict[""] #TODO: modifié drive pour que training horizon et step_time_len peut être différent de 6. 
            input_space_df = pd.read_pickle(self.path_to_input_space_calib)
            self.wheel_radius = input_space_df['calibrated_radius [m]'][0]
            self.baseline = input_space_df['calibrated_baseline [m]'][0] 
            self.max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
            self.min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]
        
        self.dataframe_stop_index = int(request.number_of_seconds_2_train_on//self.calib_step_time)
        self.get_logger().info(f"Total time used for training {self.dataframe_stop_index * self.calib_step_time}")
        self.get_logger().info(f"Nbr of step used for training {self.dataframe_stop_index}")
        #self.get_logger().info(str(self.run_by_maestro == False))

        if (self.drive_maestro_status == self.gui_message["model_training"]["status_message"]) or (self.run_by_maestro == False):
            
            self.copy_config_file()

            if self.is_torch_data_all_processed == False:
                self.model_trainer_node_status_msg.data = "processing_raw_data"
                self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))
                self.post_process_dataframe()
                self.get_logger().info("The torch_read_dataframe.pkl has been produced at"+
                f"the folowing path {self.path_to_torch_ready_df}")
                self.model_trainer_node_status_msg.data = "finish_processing_raw_data"
                self.model_trainer_node_status_pub.publish((self.model_trainer_node_status_msg))

                
        
        
            
            motion_model_name = request.motion_model

            if motion_model_name == "power_train_model" and self.is_torch_data_all_processed == True:
                

                path_of_results = self.train_pwrtrain_motion_model()
                self.is_pwrtrain_processed = True
                response.training_results = "The powertrain has been trained and the results are at the following path: "+path_of_results


            elif motion_model_name == "slip_blr"  and self.is_pwrtrain_processed == True:
                test = 3

                path_of_results = self.train_slip_blr_model_motion_model()
                self.is_slip_blr_processed = True


                response.training_results = "The slip_blr has been trained"
            elif motion_model_name == "all":
                
                path_of_results_1 = self.train_pwrtrain_motion_model()
                self.is_pwrtrain_processed = True
                path_of_results_2 = self.train_slip_blr_model_motion_model()
                self.is_slip_blr_processed = True

                response.training_results = "Motion model trained : path" + "1)powertrain : "+ path_of_results_1 + "2)slip_blr : "+path_of_results_2

                self.save_training_time()

                
        else:
            response.training_results = f"The maestro status ='{self.drive_maestro_status}' which does not equal 'model_training'. Verify that drive_maestro_node is launchedself.copy_config_file() and that you are to the model_training_phase"
        return response

    
def main():
    rclpy.init()

    minimal_service = MotionModelTrainerNode()
    
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
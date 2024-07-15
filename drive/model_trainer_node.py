from example_interfaces.srv import AddTwoInts
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
from time import gmtime, strftime

from drive.model_training.data_utils.dataset_parser import DatasetParser
from drive.model_training.data_utils.slip_dataset_parser import SlipDatasetParser
# from data_utils.acceleration_dataset_parser import AccelerationDatasetParser

from drive.model_training.model_trainers.powertrain_trainer import Powertrain_Trainer
from drive.model_training.models.powertrain.bounded_powertrain import Bounded_powertrain

from drive.model_training.model_trainers.blr_slip_trainer import SlipBLRTrainer



class MotionModelTrainerNode(Node):

    def __init__(self):
        super().__init__('motion_model_trainer_node')
        self.srv = self.create_service(TrainMotionModel, 'train_motion_model', self.train_motion_model)
        # Default value
        self.drive_maestro_status = "no_status_received_yet"
        ## Subscription
        self.exp_path_sub = self.create_subscription(
            String,
            'experiment_data_paths',
            self.experiment_path_callback,
            10)

        self.drive_maestro_status_sub = self.create_subscription(
            String,
            'maestro_status',
            self.drive_maestro_status_callback,
            10)
        
        # To load the same config param used by drive

        # input_space_df = pd.read_pickle(experiment_data_path + 'input_space_data.pkl')
        # wheel_radius = input_space_df['calibrated_radius [m]'][0]
        # baseline = input_space_df['calibrated baseline [m]'][0] 
        # max_wheel_vel = input_space_df['maximum_wheel_vel_positive [rad/s]'][0]
        # min_wheel_vel = input_space_df['maximum_wheel_vel_negative [rad/s]'][0]
    def experiment_path_callback(self,experiment_path_msg):
        self.experiment_data_path = experiment_path_msg.data

    def drive_maestro_status_callback(self,drive_maestro_status_msg):
        self.drive_maestro_status = drive_maestro_status_msg.data
    
    def train_motion_model(self, request, response):
        
        if self.drive_maestro_status == "model_training":
            motion_model_name = request.motion_model

            if motion_model_name == "power_train_model":
                test = 2 

            if motion_model_name == "slip_blr":
                test = 3

        
        else:
            response.training_results = f"The maestro status ='{self.drive_maestro_status}' which does not equal 'model_training'. Verify that drive_maestro_node is launched"
        return response


def main():
    rclpy.init()

    minimal_service = MotionModelTrainerNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
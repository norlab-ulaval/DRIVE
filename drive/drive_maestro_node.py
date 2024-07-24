import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Int32
from drive_custom_srv.srv import BashToPath,DriveMetaData
from drive_custom_srv.msg import PathTree
from std_srvs.srv import Empty,Trigger

import pathlib
from ament_index_python.packages import get_package_share_directory
import pathlib
import datetime
import shutil
import os 
from rclpy.qos import qos_profile_action_status_default
import yaml


class DriveMaestroNode(Node):
    """
    Class that sends out commands to the nodes for a full step by step drive experiment
    """
    # def __init__(self, max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
    #              step_len, dead_man_button, dead_man_index, dead_man_threshold, ramp_trigger_button, ramp_trigger_index,
    #              calib_trigger_button, calib_trigger_index, response_model_window, steady_state_std_dev_threshold,
    #              cmd_rate_param, encoder_rate_param):
    def __init__(self):
        super().__init__('drive_maestro_node')

        # Inform the user to open the GUI aka foxglove interface.
        self.get_logger().info("Please open foxglove, connect to the robot and import the foxglove template located at ../DRIVE/gui/drive_gui")

        # Initialize the defautl value of msg to publish
        self.drive_maestro_operator_action_msg = String() #Create Topic for operator action
        self.drive_maestro_operator_action_msg.data = "Fill the form in the tab named 'Metadata_form' and click on the button 'send_metadata_form'"

        self.drive_maestro_status_msg = String()
        self.drive_maestro_status_msg.data = "Metadata Form" #init at fill the form 

        self.path_to_drive_experiment_folder_msg = PathTree()
        self.path_to_drive_experiment_folder_msg.path_to_experiment_folder = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_config_folder = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_model_training_datasets = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_model_training_results = "Not define yet" 
        

        # Create publisher 
        
        self.path_to_drive_experiment_folder_pub = self.create_publisher(PathTree, 'experiment_data_paths', qos_profile_action_status_default) # Makes durability transient_local
        self.drive_maestro_operator_action_pub = self.create_publisher(String, 'operator_action', 10)
        self.drive_maestro_status_pub = self.create_publisher(String, 'maestro_status', 10)
        self.operator_action_listener = self.create_subscription(String,'operator_action_calibration', self.drive_node_operator_action_callback,1000)

        timer_period = 0.5  # seconds #TIMER
        self.timer = self.create_timer(timer_period, self.timer_callback) #TIMER execute callback

        # Services creations
        self.srv = self.create_service(DriveMetaData, 'send_metadata_form', self.send_metadata_form_callback) #service for starting drive
        self.srv = self.create_service(BashToPath, 'start_random_sampling', self.start_random_sampling_callback) #service for starting drive
       
        self.srv = self.create_service(BashToPath, 'path_to_folder', self.save_logger_dataset_service_client) # service wrapper to call the service saving the calibration dataset
        
        # Self variable initialization 
        self.path_to_share_directory = pathlib.Path(get_package_share_directory('drive'))


        # Publish the run by master topic
        #self.path_to_drive_experiment_folder_pub = self.create_publisher(Bool, 'run_by_maestro', qos_profile_action_status_default) # Makes durability transient_local
        #self.run_by_master_msg = Bool()
        #self.run_by_master_msg.data = True
        #self.path_to_drive_experiment_folder_pub.publish(self.run_by_master_msg)
        

    def timer_callback(self):
        self.publish_drive_maestro_operator_action()
        self.publish_drive_maestro_status()
        
    
    #TOPIC SUBSCRIBED
    
    
    def drive_node_operator_action_callback(self, msg): #operator action FROM drive node
        self.drive_maestro_operator_action_msg = msg

    
    #TOPIC PUBLISH
    
    def publish_drive_maestro_operator_action(self): # Operator action
        self.drive_maestro_operator_action_pub.publish(self.drive_maestro_operator_action_msg)

    def publish_drive_maestro_status(self): # Status
        self.drive_maestro_status_pub.publish(self.drive_maestro_status_msg)

    def publish_drive_maestro_path_to_drive_folder(self): # Path
        self.path_to_drive_experiment_folder_pub.publish(self.path_to_drive_experiment_folder_msg)

    # Usefull fonction 
    def create_folder(self,robot_arg,traction_arg,terrain_arg):
        """Create all the folder-tree that is used to save the data during a drive-to TNR experiements. 

        fill the self.path_dict with the path to save the info. 

        """
        path_to_directory_results = self.path_to_share_directory.parent.parent.parent.parent/'src'/'DRIVE'/'calib_data'
        
        basic_name_of_folder = robot_arg+"_"+traction_arg+"_"+terrain_arg
        path_to_experiment_folder = path_to_directory_results/basic_name_of_folder
        print(path_to_experiment_folder,2)

        if os.path.isdir(path_to_experiment_folder):
            now = datetime.datetime.now()
            basic_name_of_folder = basic_name_of_folder + f"_{now.year}_{now.month}_{now.day}_{now.hour}h{now.minute}s{now.second}"
            path_to_experiment_folder = path_to_directory_results/basic_name_of_folder
            print(path_to_experiment_folder,3)

        path_to_experiment_folder = path_to_experiment_folder
        path_config_folder = path_to_experiment_folder/"config_file_used"
        path_model_training_datasets = path_to_experiment_folder/"model_training_datasets"
        path_model_training_results = path_to_experiment_folder/"model_training_results"

        self.path_dict = {"path_experiment_folder":path_to_experiment_folder,
                    "path_config_folder":path_config_folder,
                    "path_model_training_datasets":path_model_training_datasets,
                    "path_model_training_results":path_model_training_results}
        
        
        # Create directory  
        for keyys, _path in self.path_dict.items():
            _path.mkdir()
            self.path_dict[keyys] = str(_path)
            
        
    def copy_config_files_used_in_this_experiments(self,robot_arg):
        """Copy the config files used in this experiments to be able to validate if everything is alright.
        Eventually add a validation on the config file before continuing. 

        Args:
            robot_arg (_type_): _description_
        """
        driver_node_config_specific = f"_{robot_arg}.config.yaml"
        logger_node_config_specific = f"_{robot_arg}_logger.config.yaml"
        
        path_driver_node_config = self.path_to_share_directory / driver_node_config_specific
        path_logger_config = self.path_to_share_directory /logger_node_config_specific
        
        config_file_driver_node = str(path_driver_node_config)
        config_file_logger = str(path_logger_config)

        shutil.copy(path_driver_node_config,str(pathlib.Path(self.path_dict["path_config_folder"])/driver_node_config_specific))
        shutil.copy(path_logger_config,str(pathlib.Path(self.path_dict["path_config_folder"])/logger_node_config_specific))

    #SEVICES callback and action client
    
    def send_metadata_form_callback(self, request, response):
        """
        
        1. TODO: log the information provided in a yaml file and save 
            it metadata.yaml in path_experiment_folder/metadata.yaml. 
        2. Create the path to save the data. 
        3. Copy the config file used in the protocole
        4. TODO: Compare the config file with a validation yaml file 
            to be sure that important parameters have not changed.
        5. Publish the path  
        6. 
        
        """  
        robot_arg = request.robot
        traction_arg = request.traction_geometry
        terrain_arg = request.terrain
        weather_arg = request.weather

        
        

        #2. Create the path to save the data. 
        self.create_folder(robot_arg,traction_arg,terrain_arg)
        
        ## 1.0 TODO: log the information provided in a yaml file and save 
        #    it metadata.yaml in path_experiment_folder/metadata.yaml.

        metadata = {"metadata":{"robot":robot_arg,
                    "traction":traction_arg,
                    "terrain":terrain_arg,
                    "weather":weather_arg}}
        pathlib_to_object = pathlib.Path(self.path_dict["path_experiment_folder"])/"metadata.yaml"
        pathlib_to_object.touch() # Create dump

        with open(str(pathlib_to_object),"w") as f:
                metadata_file = yaml.dump(metadata,f, sort_keys=False, default_flow_style=False)
        

        #3. Copy the config file used in the protocole
        self.copy_config_files_used_in_this_experiments(robot_arg)

        #4. TODO: Compare the config file with a validation yaml file 
            #to be sure that important parameters have not changed.

        #5 Publish the path. ----> Transient local so should be always accessible evn if published once.  
        self.path_to_drive_experiment_folder_msg.path_to_experiment_folder = self.path_dict["path_experiment_folder"]
        self.path_to_drive_experiment_folder_msg.path_config_folder = self.path_dict["path_config_folder"]
        self.path_to_drive_experiment_folder_msg.path_model_training_datasets = self.path_dict["path_model_training_datasets"]
        self.path_to_drive_experiment_folder_msg.path_model_training_results = self.path_dict["path_model_training_results"]

        self.publish_drive_maestro_path_to_drive_folder() 
        
        response.status_messages = "All path have been created and published, back of file has been done too. and meta data has been saved."
        #6 Changing the drive status to start mapping 
        self.drive_maestro_status_msg.data = "Mapping" 
        self.drive_maestro_operator_action_msg.data = "1. Drive the robot along the perimeter of the space available to realise the Protocole drive. \n 2. Drive the robot in the center of the space available. \n 3. click on the 'Mapping finished' button."

        response.status_messages = f"The results of the experiments are log in {self.path_to_drive_experiment_folder_msg.path_to_experiment_folder}"   
        return response

    def start_random_sampling_callback(self, request, response):
        """Change the maestro_status to allow drive to start working. 

        Args:
            request (_type_): _description_
            response (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.drive_maestro_operator_action_msg.data = request.input
        self.drive_maestro_status_msg.data = 'drive_ready'
        self.drive_maestro_operator_action_msg.data = "Press characterization trigger to start drive"
        return response

    def save_logger_dataset_service_client(self,request,response):
        """
        
        1. Call the service  "save_data_callback(self, req, res)" in the logger node 
        2. Adjust the operator message and action. 

        Args:
            request (_type_): _description_
            response (_type_): _description_
        """
        test =1 
        return response

    def model_training_service_client(self, request, response):
        """Call the service training  motion model in the motion model training dataset. 
        2. Adjust the operator message and action. 
        Args:
            request (_type_): _description_
            response (_type_): _description_

        Returns:
            _type_: _description_
        """

        test = 1
        return response
        


def main():
    rclpy.init()
    drive_maestro = DriveMaestroNode()
    rclpy.spin(drive_maestro)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
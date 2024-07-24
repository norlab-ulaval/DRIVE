import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Int32
from drive_custom_srv.srv import BashToPath,DriveMetaData,TrainMotionModel
from drive_custom_srv.msg import PathTree
from std_srvs.srv import Empty,Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
import pathlib
from ament_index_python.packages import get_package_share_directory
import pathlib
import datetime
import shutil
import os 
from rclpy.qos import qos_profile_action_status_default
import yaml
from rclpy.executors import MultiThreadedExecutor
from threading import Event
import time
from norlab_controllers_msgs.srv import ExportData

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

        
        # Load the gui message 
        self.path_to_share_directory = pathlib.Path(get_package_share_directory('drive'))
        path_to_gui_message = self.path_to_share_directory.parent.parent.parent.parent/'src'/'DRIVE'/'drive'/'gui_message.yaml'
        with open(str(path_to_gui_message),'r') as f:
            self.gui_message = yaml.safe_load(f)["gui_message"]
        
        self.get_logger().info(str(self.gui_message))
        # Add on 
        self.service_done_event = Event()

        self.callback_group = ReentrantCallbackGroup()
        #######
        self.sub_node = rclpy.create_node('sub_node')

        # Inform the user to open the GUI aka foxglove interface.
        self.get_logger().info("Please open foxglove, connect to the robot and import the foxglove template located at ../DRIVE/gui/drive_gui")

        # Initialize the defautl value of msg to publish
        self.drive_maestro_operator_action_msg = String() #Create Topic for operator action
        self.drive_maestro_operator_action_msg.data = self.gui_message["metadata_form"]["operator_action_message"]

        self.drive_maestro_status_msg = String()
        self.drive_maestro_status_msg.data = self.gui_message["metadata_form"]["status_message"] #init at fill the form 
        # Path initialization
        self.path_to_drive_experiment_folder_msg = PathTree()
        self.path_to_drive_experiment_folder_msg.path_to_experiment_folder = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_config_folder = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_model_training_datasets = "Not define yet" 
        self.path_to_drive_experiment_folder_msg.path_model_training_results = "Not define yet" 
        

        # Create publisher 
        
        self.path_to_drive_experiment_folder_pub = self.create_publisher(PathTree, 'experiment_data_paths', qos_profile_action_status_default) # Makes durability transient_local
        self.drive_maestro_operator_action_pub = self.create_publisher(String, 'operator_action', 10)
        self.drive_maestro_status_pub = self.create_publisher(String, 'maestro_status', 10)

        #Create subscriber

        self.calib_state_listener = self.create_subscription(String,'calib_state', self.calib_state_callback, 10)
        self.calib_state_listener_msg = 'not yet defined'

        self.drive_finish_published = False

        timer_period = 0.5  # seconds #TIMER
        self.timer_callgroup = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(timer_period, self.timer_callback,callback_group=self.timer_callgroup) #TIMER execute callback

        # Services creations
        self.srv_call_group = MutuallyExclusiveCallbackGroup() 
        self.srv_send_metadata = self.create_service(DriveMetaData, 'send_metadata_form', self.send_metadata_form_callback) #service for starting drive

        self.srv_save_logger_dataset = self.create_service(Trigger, 'maestro_export_data', self.save_logger_dataset_service_client,callback_group=MutuallyExclusiveCallbackGroup() ) # service wrapper to call the service saving the calibration dataset
        self.srv_stop_mapping = self.create_service(Trigger, 'done_mapping', self.stop_mapping_service,callback_group=MutuallyExclusiveCallbackGroup() ) # service wrapper to call the service saving the calibration dataset
        
        self.srv_train_motion_model = self.create_service(TrainMotionModel, 'maestro_train_motion_model', self.model_training_service_client,callback_group=MutuallyExclusiveCallbackGroup() ) # service wrapper to call the service saving the calibration dataset
        
    
        
        # Creation of client
        
        self.stop_mapping_client = self.sub_node.create_client(Empty, '/mapping/disable_mapping', callback_group=MutuallyExclusiveCallbackGroup() )
        self.save_calibration_dataset_client = self.sub_node.create_client(ExportData, '/drive/export_data', callback_group=MutuallyExclusiveCallbackGroup() )
        self.train_motion_model_client = self.sub_node.create_client(TrainMotionModel, '/drive/train_motion_model', callback_group=MutuallyExclusiveCallbackGroup() )
        
        
        # Self variable initialization 
        
        
        # Publish the run by master topic
        #self.path_to_drive_experiment_folder_pub = self.create_publisher(Bool, 'run_by_maestro', qos_profile_action_status_default) # Makes durability transient_local
        #self.run_by_master_msg = Bool()
        #self.run_by_master_msg.data = True
        #self.path_to_drive_experiment_folder_pub.publish(self.run_by_master_msg)
        

    def timer_callback(self):
        self.publish_drive_maestro_operator_action()
        self.publish_drive_maestro_status()
        
    
    #TOPIC SUBSCRIBED
    
    
    def calib_state_callback(self, msg): #operator action FROM drive node
        self.calib_state_listener_msg = msg.data
        
        if self.calib_state_listener_msg == 'drive_finished' and self.drive_finish_published == False:
            self.drive_maestro_status_msg.data = self.gui_message["drive_save"]["status_message"]
            self.drive_maestro_operator_action_msg.data = self.gui_message["drive_save"]["operator_action_message"]
            self.publish_drive_maestro_status()
            self.publish_drive_maestro_operator_action()
            self.drive_finish_published =True
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
        
        self.path_dict["path_to_calibration_node_config"] = str(path_driver_node_config)
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
        self.path_to_drive_experiment_folder_msg.path_to_calibration_node_config = self.path_dict["path_to_calibration_node_config"]

        self.publish_drive_maestro_path_to_drive_folder() 
        
        response.status_messages = "All path have been created and published, back of file has been done too. and meta data has been saved."
        #6 Changing the drive status to start mapping 
        self.drive_maestro_status_msg.data = self.gui_message["mapping"]["status_message"] 
        self.drive_maestro_operator_action_msg.data = self.gui_message["mapping"]["operator_action_message"]

        response.status_messages = f"The results of the experiments are log in {self.path_to_drive_experiment_folder_msg.path_to_experiment_folder}"   
        return response
    
    def stop_mapping_service(self, request, response):
        """Call the service training  motion model in the motion model training dataset. 
        2. Adjust the operator message and action. Shout out  ce og:
          https://answers.ros.org/question/373169/mistakes-using-service-and-client-in-same-node-ros2-python/.
        Args:
            request (_type_): _description_
            response (_type_): _description_

        Returns:
            _type_: _description_
        """

        if self.drive_maestro_status_msg.data == self.gui_message["mapping"]["status_message"]:
            
            self.stop_mapping_client.wait_for_service()
            self.stop_mapping_req = Empty.Request() 
            self.future = self.stop_mapping_client.call_async(self.stop_mapping_req)
            rclpy.spin_until_future_complete(self.sub_node, self.future)
            answer = self.future.result()
            self.get_logger().info("\n"*4+str(answer)+"\n"*4)

            if self.future.result() is not None:
                self.drive_maestro_status_msg.data = self.gui_message["drive_ready"]["status_message"]
                self.drive_maestro_operator_action_msg.data = self.gui_message["drive_ready"]["operator_action_message"]
            
                response.success = True
                response.message = "The mapping has been stopped and drive is ready to start."
            else:
                response.success = False
                response.message = "The map has not been stopped."
        else:
            response.success = False
            response.message = f"Not in the correct status, you should be in the {self.gui_message['mapping']['status_message']},but you are in the {self.drive_maestro_status_msg.data}"
        

        

        return response


    def save_logger_dataset_service_client(self,request,response):
        """
        
        1. Call the service  "save_data_callback(self, req, res)" in the logger node 
        2. Adjust the operator message and action. 

        Args:
            request (_type_): _description_
            response (_type_): _description_
        """

        
        if self.drive_maestro_status_msg.data == self.gui_message["drive_save"]["status_message"]:
            
            self.save_calibration_dataset_client.wait_for_service()
            req = ExportData.Request()

            path_to_dataset = pathlib.Path(self.path_to_drive_experiment_folder_msg.path_model_training_datasets) /"data_raw.pkl"

            path = String()
            path.data = str(path_to_dataset)
            req.export_path = path

            future = self.save_calibration_dataset_client.call_async(req)
            rclpy.spin_until_future_complete(self.sub_node, future)
            answer = future.result()
            self.get_logger().info("\n"*4+str(answer)+"\n"*4)

            if future.result() is not None:
                self.drive_maestro_status_msg.data = self.gui_message["model_training"]["status_message"]
                self.drive_maestro_operator_action_msg.data = self.gui_message["model_training"]["operator_action_message"]
            
                response.success = True
                response.message = f"The dataset has been saved to the following path {path_to_dataset}."
            else:
                response.success = False
                response.message = "The dataset was not saved has not been stopped."
        else:
            response.success = False
            response.message = f"Not in the correct status, you should be in the {self.gui_message['drive_save']['status_message']},but you are in the {self.drive_maestro_status_msg.data}"
        

            


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

        if self.drive_maestro_status_msg.data == self.gui_message["model_training"]["status_message"]:
            

            self.train_motion_model_client.wait_for_service()
            req = TrainMotionModel.Request()
            req.motion_model = request.motion_model

            future = self.train_motion_model_client.call_async(req)
            rclpy.spin_until_future_complete(self.sub_node, future)
            answer = future.result()
            self.get_logger().info("\n"*4+str(answer)+"\n"*4)

            if future.result() is not None:
                self.drive_maestro_status_msg.data = self.gui_message["model_training"]["status_message"]
                self.drive_maestro_operator_action_msg.data = self.gui_message["model_training"]["operator_action_message"]
                response.training_results = answer.training_results
            else:
                
                response.training_results = "The training did not work"
        else:
            response.training_results = f"Not in the correct status '{self.drive_maestro_status_msg.data}', you should be in the {self.gui_message['model_training']['status_message']},but you are in the {self.drive_maestro_status_msg.data}"

        return response
        
        
        

def main():
    rclpy.init()
    drive_maestro = DriveMaestroNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(drive_maestro)
    try:
        drive_maestro.get_logger().info('Beginning drive_maestro_node')
        executor.spin()
    except KeyboardInterrupt:
        drive_maestro.get_logger().info('Keyboard interrupt, shutting down.\n')
    drive_maestro.destroy_node()
    rclpy.shutdown()

    #service_from_service = DriveMaestroNode()

    #executor = MultiThreadedExecutor()
    #rclpy.spin(service_from_service, executor)

    #rclpy.shutdown()

if __name__ == '__main__':
    main()
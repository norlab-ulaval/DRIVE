import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, UInt32,Float32
from drive_custom_srv.srv import BashToPath,DriveMetaData,TrainMotionModel,ExecuteAllTrajectories,LoadTraj
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
from nav_msgs.msg import Odometry
from  drive.trajectory_creator.eight_trajectory import EightTrajectoryGenerator,RectangleTrajectoryGenerator

import numpy as np 
import matplotlib.pyplot as plt
from norlab_controllers_msgs.action import FollowPath
from norlab_controllers_msgs.msg import PathSequence,DirectionalPath,FollowerOptions
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Pose,Quaternion,Point 
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Path

from rclpy.action import ActionClient

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
        self.visualise_path_topic_name = "visualize_path_topic"
        self.path_to_drive_experiment_folder_pub = self.create_publisher(PathTree, 'experiment_data_paths', qos_profile_action_status_default) # Makes durability transient_local
        self.drive_maestro_operator_action_pub = self.create_publisher(String, 'operator_action', 10)
        self.drive_maestro_status_pub = self.create_publisher(String, 'maestro_status', 10)
        self.path_loaded_pub = self.create_publisher(Path,"path_to_reapeat",qos_profile_action_status_default)
        
        #Create subscriber
        self.calib_state_listener = self.create_subscription(String,'calib_state', self.calib_state_callback, 10)
        self.calib_state_listener_msg = 'not yet defined'
        self.odom_in = self.create_subscription(Odometry,'odom_in', self.odom_in_callback, 10)
        self.pose = Odometry()


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
        
        self.srv_create_and_execute_trajectories = self.create_service(ExecuteAllTrajectories, 'execute_all_trajectories', self.execute_all_trajectories_call_back,callback_group=MutuallyExclusiveCallbackGroup() ) # service wrapper to call the service saving the calibration dataset
        self.srv_load_a_trajectorie = self.create_service(LoadTraj, 'load_trajectory', self.load_trajectory_callback,callback_group=MutuallyExclusiveCallbackGroup() ) # service wrapper to call the service saving the calibration dataset
        
        
        # Creation of service client
        
        self.stop_mapping_client = self.sub_node.create_client(Empty, '/mapping/disable_mapping', callback_group=MutuallyExclusiveCallbackGroup() )
        self.save_calibration_dataset_client = self.sub_node.create_client(ExportData, '/drive/export_data', callback_group=MutuallyExclusiveCallbackGroup() )
        self.train_motion_model_client = self.sub_node.create_client(TrainMotionModel, '/drive/train_motion_model', callback_group=MutuallyExclusiveCallbackGroup() )
        
        # Creation of action client in the subnode so that the server of the maestro can call the action client and react in cnsequence of the feedback
        #  

        self._action_client = ActionClient(self, FollowPath, '/controller/follow_path')

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
    
    def odom_in_callback(self,msg):
        #self.get_logger().info(str(msg))
        self.pose = msg
        
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
        self.get_logger().info("\n"*3+yaml.__version__+"\n"*3)
        with open(str(pathlib_to_object),'w') as f:
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
                response.training_results = answer.training_results

                if req.motion_model == "all":
                    self.drive_maestro_status_msg.data = self.gui_message["load_trajectory"]["status_message"]
                    self.drive_maestro_operator_action_msg.data = self.gui_message["load_trajectory"]["operator_action_message"]
                
            else:
                
                response.training_results = "The training did not work"
        else:
            response.training_results = f"Not in the correct status '{self.drive_maestro_status_msg.data}', you should be in the {self.gui_message['model_training']['status_message']},but you are in the {self.drive_maestro_status_msg.data}"

        return response
        
    def load_trajectory_callback(self,request,response):


        # Extract position 
        pose_extracted =self.pose.pose.pose
        orientation = pose_extracted.orientation
        translation_2d = np.array([pose_extracted.position.x, pose_extracted.position.y,1])
        rotation = Rotation.from_quat(np.array([orientation.x,orientation.y,orientation.z,orientation.w]))
        yaw_rotation = rotation.as_euler("zyx")[0]

        self.get_logger().info(str(translation_2d))

        transform_2d = np.zeros((3,3))
        transform_2d[:,2] = translation_2d
        transform_2d[:2,:2] = np.array([[np.cos(yaw_rotation),-np.sin(yaw_rotation)],[np.sin(yaw_rotation),np.cos(yaw_rotation)]])

        # Pass the transform 

        
        self.get_logger().info("\n"*3 +str(transform_2d)+"\n"*3 )
        #self.get_logger().info()

        # Type of controller posible []
        list_possible_trajectory = ["eight","rectangle"]

        trajectory_type = request.trajectory_type
        trajectory_args = request.trajectory_args
        frame_id = request.frame_id

        if trajectory_type in list_possible_trajectory:
            
            if trajectory_type == "eight":
                radius, entre_axe,horizon = trajectory_args
                if entre_axe >= 2*radius:
                    trajectory_generator = EightTrajectoryGenerator(radius,entre_axe,horizon)
                    trajectory_generator.compute_trajectory()
                    time_stamp = self.get_clock().now().to_msg()
                    traj_in_path_sequence,visualize_path_ros = trajectory_generator.export_2_norlab_controller(time_stamp,
                                                                                            frame_id,transform_2d)
                    self.path_loaded_pub.publish(visualize_path_ros)
                    response.success = True
                    response.message = f"The trajectory has been load. To visualize it, open the topic {self.visualise_path_topic_name} "
                else:
                    response.success = False
                    response.message = f"The trajectory has not been load the 'entreaxe' needs to be at least two times bigger than the radius."

                
            elif trajectory_type =="rectangle":
                
                width, lenght,horizon = trajectory_args
                trajectory_generator = RectangleTrajectoryGenerator(width,lenght,horizon)
                trajectory_generator.compute_trajectory()
                time_stamp = self.get_clock().now().to_msg()
                self._path_to_execute ,visualize_path_ros = trajectory_generator.export_2_norlab_controller(time_stamp,
                                                                                        frame_id,transform_2d)
                self.get_logger().info(str(visualize_path_ros))
                self.path_loaded_pub.publish(visualize_path_ros)
                

                response.success = True
                response.message = f"The trajectory has been load. To visualize it, open the topic {self.visualise_path_topic_name} "

        else:
            response.success = False
            response.message = f"WRONG TRAJECTORY TYPE: please write one of the folowing trajectory type {list_possible_trajectory}"
        
        return response
    def send_follow_path_goal(self):
        goal_msg = FollowPath.Goal()
        goal_msg.follower_options = self.follower_option
        goal_msg.path = self._path_to_execute
        self._action_client.wait_for_server()
        
        self._send_follow_path_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_follow_path_goal_future.add_done_callback(self.goal_response_followback_callback)

    def goal_response_followback_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('"Goal was rejected by server"')
            return

        self.get_logger().info('"Goal accepted by server, waiting for result"')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_followback_callback)

    def get_result_followback_callback(self, future):
        result = future.result().result
        if result.result_status.data == 1:
            self.get_logger().info(f'The trajectory has been completed, going back to load_traj_status')

        self.drive_maestro_status_msg.data = self.gui_message["load_trajectory"]["status_message"]
        self.drive_maestro_operator_action_msg.data = self.gui_message["load_trajectory"]["operator_action_message"]


    def execute_all_trajectories_call_back(self,request,response):
        # Type of controller posible []
        
        controller_name = request.controller_name
        list_of_max_speed = request.list_of_max_speed
        nb_of_repetition_of_each_speed =  request.nbr_repetition_of_each_speed

        if False: #self.drive_maestro_status_msg.data == self.gui_message["play_traj"]["status_message"]:
            response.success = False
            response.message = "A trajectory is already being played based on the status"
        else:
            ## Send goal 
            list_possible_controller = ["ideal-diff-drive-mpc","test2","test3"]

            if controller_name in list_possible_controller:
                init_mode_ = UInt32()
                init_mode_.data = 1
                max_speed = Float32()
                max_speed.data = list_of_max_speed[0]
                self.follower_option = FollowerOptions()
                self.follower_option.init_mode = init_mode_
                self.follower_option.velocity = max_speed

                future = self.send_follow_path_goal()

                # Used the precedent_traj and call the action client 
                self.drive_maestro_status_msg.data = self.gui_message["play_traj"]["status_message"]
                self.drive_maestro_operator_action_msg.data = self.gui_message["play_traj"]["operator_action_message"]

            else:
                response.success = False
                response.message = f"The controller name is bad, it should be one of the following: {list_possible_controller}"

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
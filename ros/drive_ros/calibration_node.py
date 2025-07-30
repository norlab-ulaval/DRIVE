#!/usr/bin/env python3

from dataclasses import dataclass
import datetime
from typing import Literal
import pathlib
import pandas as pd 
import numpy as np
import rclpy
from drive_ros.node_utils import (
    redirect_logging_to_ros2,
    declare_parameter_from_dataclass,
    update_parameter_from_dataclass,
    
)
from scipy.spatial.transform import Rotation as R
from drive_ros.calibration_node_utils import (
    compute_sampling_space, 
    forward_kin, 
    inverse_kin, 
    save_sampling_space,
    load_sampling_spaces)
from drive_ros.drive_ros_bridge import DriveRosBridgeParams
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty, SetBool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Point32, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Path, Odometry

from DRIVE.common import Pose, Command
from DRIVE.drive import (
    BackToGeofenceState,
    Drive,
    GeofenceCreationState,
    PausedState,
    ReadyState,
    RunningState,
    WaitingState,
)
from DRIVE.robot import Robot
from DRIVE.sampling import CommandSamplingFactory


@dataclass
class CalibData:
    wheel_speed_encoder = np.array([0.0, 0.0, 0.0])  # timestamp_sec, wheel speed l, wheel speed r
    left_motor_command = np.array([0.0, 0.0])
    right_motor_command = np.array([0.0, 0.0])
    left_motor_encoder = np.array([0.0, 0.0])
    right_motor_encoder = np.array([0.0, 0.0])
    odom_encoder = np.array([[0.0, 0.0, 0.0]])




PossibleState = Literal[
    "do_this_calibration",
    "trajectory_vizualization",
    "calibration_finished",
    "executing_command",
    "computing",
    "cmd_motor_accessible",
    "linear_command_validation",
    "angular_command_validation",
    "computing_linear_command",
    "computing_angular_command",
]


class SampleSpaceIdentifier:

    def __init__(self, params: DriveRosBridgeParams, robot: Robot):
        self.calibration_name = "sample_space_identification"
        self.state: PossibleState = "do_this_calibration"
        self.screen_msg = "Do you want to do the sample space identification?"

        self.sample_space = np.array([0.0, 0.0])
        self.robot = robot
        self.params = params
        self.starting_time = 10**10
        self.step_duration_s = 6.0
        self.command_to_send = np.array([0, 0])
        self.left_wheel_encorder_buffer = np.array([0.0, 0.0])
        self.right_wheel_encorder_buffer = np.array([0.0, 0.0])
        self.time_elapsed = 0.0
        self.nb_second_to_computed_top_speed = 2
        self.max_wheel_speed = 0
        self.sampling_space_wheel_constraints = np.array([0.0, 0.0])
        self.sampling_space_body_constraints = np.array([0.0, 0.0])
        self.resulting_sampling_space = np.array([0.0, 0.0])
        self.debug = ""
        self.precedent_state = self.state

        self.calibration_folder = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name / "calibration"
        self.path_config_file = self.calibration_folder / "sample_space.yaml"
        self.mean_encoder_odom = np.array([0.0, 0.0])
        
    def get_screen_msg(self):
        if self.state == "do_this_calibration":
            self.screen_msg = f"Do you want to do the {self.calibration_name} calibration ?"
        elif (
            self.state == "trajectory_vizualization"
            or self.state == "linear_command_validation"
            or self.state == "angular_command_validation"
        ):
            self.screen_msg = "Are you ready to execute the projected trajectory?"
        elif self.state == "computing":
            self.screen_msg = f"The resulting sampling space is in the folder {self.params.datasets_directory}. \n Do you want to change the parameters ?"
        elif self.state == "executing_command":
            self.screen_msg = "Executing command watch the robot move..."
        elif self.state == "calibration_finished":
            self.screen_msg = (
                "You have finished the sampling space calibration. Click yes to go to the next calibration"
            )

    def update_step(self, update_step: bool, timestamp_s: float):
        self.precedent_state = self.state
        if self.state == "do_this_calibration":
            print(update_step)
            if update_step:
                self.state = "trajectory_vizualization"
                self.command_to_send = np.array([self.params.max_linear_speed, 0.0])
            else:
                self.path_config_file = pathlib.Path(self.params.abs_path_to_calib)
                self.state = "calibration_finished"

        elif self.state == "trajectory_vizualization":
            ### Send the command to the robot and wait for its execution
            if update_step:
                self.state = "executing_command"
                self.starting_time = timestamp_s

        elif self.state == "computing":

            if update_step:
                self.state = "trajectory_vizualization"
                self.command_to_send = np.array([self.params.max_linear_speed, 0.0])
                
            else:
                self.state = "calibration_finished"

        return self.state, self.screen_msg

    def execution_logic(self, timestamp_s: float, data: CalibData):

        if self.state == "executing_command":

            self.left_wheel_encorder_buffer = np.vstack((self.left_wheel_encorder_buffer, data.left_motor_encoder))
            self.right_wheel_encorder_buffer = np.vstack((self.right_wheel_encorder_buffer, data.right_motor_encoder))
            self.robot.send_command(self.command_to_send)
            self.time_elapsed = timestamp_s - self.starting_time
            print(self.time_elapsed)
            if self.time_elapsed > self.step_duration_s:
                self.state = "computing"
                # End of the recording
                self.robot.send_command(np.array([0.0, 0.0]))

                # Compute maximum wheel speed
                nb_indices = int(self.params.protocol_frequency * self.nb_second_to_computed_top_speed)
                left_wheel_max = np.mean(self.left_wheel_encorder_buffer[-nb_indices:, 1])
                right_wheel_max = np.mean(self.right_wheel_encorder_buffer[-nb_indices:, 1])
                #self.debug = f" {self.right_wheel_encorder_buffer[-nb_indices:,1]} {nb_indices} Left wheel max speed: {left_wheel_max}, Right wheel max speed: {right_wheel_max}"

                self.max_wheel_speed = np.max(np.array([left_wheel_max, right_wheel_max]))

                # Compute sampling_space

                sampling_spaces = compute_sampling_space(float(self.max_wheel_speed), self.params)

                self.calibration_folder = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name / "calibration"
                self.calibration_folder.mkdir(parents=True, exist_ok=True)
                self.path_config_file = self.calibration_folder / "sample_space.yaml"
                
                save_sampling_space(sampling_spaces, self.path_config_file)
                # dico_polygons = {"wheel_polygons": pol_wheel_bf_constraints,
                #     "body_polygons": polygon_bf_constraints,
                #     "sampling_space": sampling_space}  
                self.sampling_space_wheel_constraints = sampling_spaces["wheel_polygons"]
                self.sampling_space_body_constraints = sampling_spaces["body_polygons"]
                self.resulting_sampling_space = sampling_spaces["sampling_space"]
        self.get_screen_msg()


class SamplingSpaceValidation:

    def __init__(self, params: DriveRosBridgeParams, robot: Robot, path_config_file: pathlib.Path = pathlib.Path("None")):
        self.calibration_name = "sample_space_validation"
        self.state: PossibleState = "do_this_calibration"
        self.screen_msg = "Do you want to do the sample space identification?"
        
        if not path_config_file.is_file():
            raise ValueError(f"path to sampling space is not a file :  path ={path_config_file}")

        self.path_config_file = path_config_file

        
        self.sample_spaces = load_sampling_spaces(path_config_file)
        #self.debug = str(self.sample_spaces)
        self.robot = robot
        self.params = params
        self.starting_time = 10**10
        self.step_duration_s = 6.0
        self.command_to_send = np.array([0, 0])
        self.left_wheel_encorder_buffer = np.array([0.0, 0.0])
        self.right_wheel_encorder_buffer = np.array([0.0, 0.0])

        self.left_wheel_cmd_buffer = np.array([0.0, 0.0])
        self.right_wheel_cmd_buffer = np.array([0.0, 0.0])
        self.using_motor_cmd_topic = False
        self.time_elapsed = 0.0
        self.nb_second_to_computed_top_speed = 2
        self.max_wheel_speed = 0
        self.sampling_space_wheel_constraints = self.sample_spaces["wheel_polygons"]
        self.sampling_space_body_constraints = self.sample_spaces["body_polygons"]
        self.resulting_sampling_space = self.sample_spaces["sampling_space"]
        self.debug = ""
        self.precedent_state = self.state
 
        self.encoder_cmd_treshold = 0.05
        self.cmd_sampling_error = 0.01
        self.left_wheel_cmd_encoder_msg = "NA"
        self.right_wheel_cmd_encoder_msg = "NA"
        self.left_wheel_cmd_diff_msg = "Left CMD difference: NA"
        self.right_wheel_cmd_diff_msg = "Right CMD difference: NA"    

        self.sampling_space_max_linear_speed = np.max(self.resulting_sampling_space.exterior.xy[0])
        self.sampling_space_max_angular_speed = np.max(self.resulting_sampling_space.exterior.xy[1])
        #self.debug = f"Max linear speed {self.sampling_space_max_linear_speed}, Max angular speed {self.sampling_space_max_angular_speed}"
        self.calibration_folder = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name / "calibration"
        self.calibration_folder.mkdir(parents=True, exist_ok=True)
        save_sampling_space(self.sample_spaces, self.calibration_folder/ "sample_space.yaml")
        self.mean_encoder_odom = np.array([0.0, 0.0])

    def get_screen_msg(self):
        if self.state == "do_this_calibration":
            self.screen_msg = f"Do you want to do the {self.calibration_name} calibration ?"
        elif (
            self.state == "trajectory_vizualization"
            or self.state == "linear_command_validation"
            or self.state == "angular_command_validation"
        ):
            self.screen_msg = "Are you ready to execute the projected trajectory?"
        elif self.state == "computing":
            self.screen_msg = f"The resulting sampling space is in the folder {self.params.datasets_directory}. \n Do you want to change the parameters ?"
        elif self.state == "executing_command":
            self.screen_msg = "Executing command watch the robot move..."
        elif self.state == "calibration_finished":
            self.screen_msg = (
                "You have finished the sampling space calibration. Click yes to go to the next calibration"
            )
        elif self.state == "cmd_motor_accessible":

            self.screen_msg = "Is the topic that sends command to the motor accessible and wired ?"
        elif self.state == "computing_linear_command":
            self.screen_msg = "Here are the results " +"\n"+ self.left_wheel_cmd_encoder_msg + "\n" + self.right_wheel_cmd_encoder_msg + "\n" + \
                self.left_wheel_cmd_diff_msg + "\n" + self.right_wheel_cmd_diff_msg + "\n" + "Click yes to test the angular speed axes"
                 

        elif self.state == "computing_angular_command":
            self.screen_msg = self.left_wheel_cmd_encoder_msg + "\n" + self.right_wheel_cmd_encoder_msg + "\n" + \
                self.left_wheel_cmd_diff_msg + "\n" + self.right_wheel_cmd_diff_msg  
        else:
            self.screen_msg = "Unknown state"   
    def update_step(self, update_step: bool, timestamp_s: float):
        self.precedent_state = self.state
        if self.state == "do_this_calibration":
            print(update_step)
            if update_step:
                self.state = "cmd_motor_accessible"

            else:
                self.state = "calibration_finished"

        elif self.state == "cmd_motor_accessible":

            if update_step:
                self.using_motor_cmd_topic = True
                
            else:
                 self.using_motor_cmd_topic = False

            self.state = "linear_command_validation"
            
            self.command_to_send = np.array([self.sampling_space_max_linear_speed, 0.0])
            self.expected_wheel_speed = np.abs(inverse_kin(self.command_to_send, self.params)[0])
        elif self.state == "linear_command_validation" or self.state == "angular_command_validation":
            ### Send the command to the robot and wait for its execution
            if update_step:
                self.state = "executing_command"
                self.starting_time = timestamp_s
        

        elif self.state == "computing_linear_command":

            if update_step:
                self.state = "angular_command_validation"
                self.command_to_send = np.array([0.0, self.sampling_space_max_angular_speed])
                self.debug = f"Command {self.command_to_send}"
                self.expected_wheel_speed = np.abs(inverse_kin(self.command_to_send, self.params)[0])
            else:
                self.state = "linear_command_validation"
                self.command_to_send = np.array([self.sampling_space_max_linear_speed, 0.0])
                self.expected_wheel_speed = np.abs(inverse_kin(self.command_to_send, self.params)[0])

        elif self.state == "computing_angular_command":

            if update_step:
                self.state = "calibration_finished"

            else:
                self.state = "angular_command_validation"
                self.command_to_send = np.array([0.0, self.sampling_space_max_angular_speed])
                self.expected_wheel_speed = np.abs(inverse_kin(self.command_to_send, self.params)[0])

        return self.state, self.screen_msg

    def execution_logic(self, timestamp_s: float, data: CalibData):

        if self.state == "executing_command":
            
            
            self.left_wheel_encorder_buffer = np.vstack((self.left_wheel_encorder_buffer, data.left_motor_encoder))
            self.right_wheel_encorder_buffer = np.vstack((self.right_wheel_encorder_buffer, data.right_motor_encoder))

            if self.using_motor_cmd_topic:
                self.left_wheel_cmd_buffer = np.vstack((self.left_wheel_cmd_buffer, data.left_motor_command))
                self.right_wheel_cmd_buffer = np.vstack((self.right_wheel_cmd_buffer, data.right_motor_command))

            self.robot.send_command(self.command_to_send)
            self.time_elapsed = timestamp_s - self.starting_time
            self.debug = f"Elapsed time {self.left_wheel_encorder_buffer} s"
            if self.time_elapsed > self.step_duration_s:

                # End of the recording
                self.robot.send_command(np.array([0.0, 0.0]))

                if self.precedent_state == "linear_command_validation":
                    self.state = "computing_linear_command"
                elif self.precedent_state == "angular_command_validation":
                    self.state = "computing_angular_command"
                else:
                    raise ValueError("Invalid state transition")

                # Compute the difference between the encoder and the command
                nb_indices = int(self.params.protocol_frequency * self.nb_second_to_computed_top_speed)
                left_wheel_value = np.mean(self.left_wheel_encorder_buffer[-nb_indices:, 1])
                right_wheel_value = np.mean(self.right_wheel_encorder_buffer[-nb_indices:, 1])

                self.mean_encoder_speed = np.array([left_wheel_value, right_wheel_value])
                self.mean_encoder_odom = forward_kin(self.mean_encoder_speed, self.params)

                left_wheel_error = np.abs((np.abs(left_wheel_value) - self.expected_wheel_speed)/self.expected_wheel_speed) * 100
                right_wheel_error = np.abs((np.abs(right_wheel_value) - self.expected_wheel_speed)/self.expected_wheel_speed) * 100

                self.debug = f"Nb indicies {nb_indices}, Left wheel error {np.round(left_wheel_error,2)} % ,  Right wheel error {right_wheel_value} %"

                
                self.left_wheel_cmd_encoder_msg = f"Left wheel encoder has an error of {np.round(left_wheel_error,2)} %"
                    
                

                self.right_wheel_cmd_encoder_msg = (
                        f"Right wheel encoder has an error of {np.round(right_wheel_error,2)} %"
                    )
                
                dico = { "expected_wheel_speed": np.ones_like(self.left_wheel_encorder_buffer[:,0]) * self.expected_wheel_speed,
                            
                    "left_wheel_encoder": self.left_wheel_encorder_buffer[:, 1],
                            "left_wheel_encoder_timestamp": self.left_wheel_encorder_buffer[:, 0],
                            "right_wheel_encoder": self.right_wheel_encorder_buffer[:, 1],
                            "right_wheel_encoder_timestamp": self.right_wheel_encorder_buffer[:, 0],
                            
                            
                            }
                if self.using_motor_cmd_topic:
                    
                    dico.update({"left_wheel_cmd": self.left_wheel_cmd_buffer[:, 1],
                                 "left_wheel_cmd_timestamp": self.left_wheel_cmd_buffer[:, 0],
                            "right_wheel_cmd": self.right_wheel_cmd_buffer[:,1],
                                    "right_wheel_cmd_timestamp": self.right_wheel_cmd_buffer[:, 0]
                            })
                    left_wheel_cmd = np.mean(self.left_wheel_cmd_buffer[-nb_indices:, 1])
                    right_wheel_cmd = np.mean(self.right_wheel_cmd_buffer[-nb_indices:, 1])

                    left_wheel_cmd_error = np.abs((np.abs(left_wheel_cmd) - self.expected_wheel_speed)/self.expected_wheel_speed) * 100
                    right_wheel_cmd_error = np.abs((np.abs(right_wheel_cmd) - self.expected_wheel_speed)/self.expected_wheel_speed) * 100

                    self.left_wheel_cmd_diff_msg = (
                            f"Right wheel sampled and sent command have an error of {left_wheel_cmd_error} % "
                        )
                    self.right_wheel_cmd_diff_msg = (
                            f"Right wheel sampled and sent command have an error of  {right_wheel_cmd_error} %"
                        )
                        
                df = pd.DataFrame(dico)
                df.to_csv(self.calibration_folder / f"{self.state}.csv", index=False)
                # Compute sampling_space

        self.get_screen_msg()


class DriveCalibration:

    def __init__(self, param: DriveRosBridgeParams, robot: Robot):
        
        self.current_calbiration = SampleSpaceIdentifier(param, robot)
        self.params = param
        self.state = self.current_calbiration.state
        self.screen_msg = self.current_calbiration.screen_msg
        self.robot = robot
        self.saved_path = ""
        self.path_config_file = self.current_calbiration.path_config_file
    def update_step(self, update_step, timestamp_s: float):

        if self.current_calbiration.state == "calibration_finished":

            if self.current_calbiration.calibration_name == "sample_space_identification":
                # Save the sample space to a file or database
                print("Sample space calibration completed and saved.")
                path_to_calibration = self.current_calbiration.path_config_file
                self.current_calbiration = SamplingSpaceValidation(self.params, self.robot, 
                                                                   path_config_file=path_to_calibration)
                # self.state = "All calibration are finished"
                # self.screen_msg = "You have finished the calibration node. Enjoy your DRIVE"
            elif self.current_calbiration.calibration_name == "sampling_space_validation":
                self.state = "All calibration are finished"
                self.screen_msg = "You have finished the calibration node. Enjoy your DRIVE"

        else:
            self.state, self.screen_msg = self.current_calbiration.update_step(update_step, timestamp_s)

    def run(self, data: CalibData, timestamp_s: float):


        
        self.current_calbiration.execution_logic(timestamp_s, data)

        self.state, self.screen_msg = self.current_calbiration.state, self.current_calbiration.screen_msg


class DriveRosCalibration(Node):
    def __init__(self):
        super().__init__("drive_calibration", parameter_overrides=[])

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.params = DriveRosBridgeParams()

        
        declare_parameter_from_dataclass(self, self.params)
        self.create_timer(1.0, lambda: update_parameter_from_dataclass(self, self.params))

        self.dataset_directory = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name
        self.current_goal: Pose | None = None
        # Find config file path
        
        
        
        #self.get_logger().info(f"Dataset directory: {self.params.wheel_radius}")
        # Drive calib core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.calib = DriveCalibration(self.params, self.robot)
        self.data = CalibData()
        # ROS setup
        delay = 1.0 / self.params.protocol_frequency
        self.get_logger().info(f"Control loop frequency: {self.params.protocol_frequency} Hz (delay: {delay:.3f} s)")
        self.timer = self.create_timer(delay, self.control_loop)

        # Pubs
        self.cmd_pub = self.create_publisher(Twist, "cmd_drive", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)
        self.viz_current_calibration_pub = self.create_publisher(String, "drive/viz/current_calibration", 10)

        self.wheel_constraints_pub = self.create_publisher(PolygonStamped, "drive/viz/wheel_constraints", 10)
        self.body_constraints_pub = self.create_publisher(PolygonStamped, "drive/viz/body_constraints", 10)
        self.sampling_space_pub = self.create_publisher(PolygonStamped, "drive/viz/sampling_space", 10)
        self.viz_current_commend_pub = self.create_publisher(Marker, "drive/viz/current_command", 10)
        self.viz_encoder_odom_pub = self.create_publisher(Marker, "drive/viz/encoder_odom", 10)
        # Subs
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "pause_drive", self.deadman_callback, 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)
        self.encoder_odom_sub = self.create_subscription(Odometry, "/encoder_odom", self.encoder_odom_callback, 10)

        # Assume that cmd _motor  = Float64
        self.left_cmd_motor_sub = self.create_subscription(Float64, "/left_motor_cmd", self.left_cmd_motor_callback, 10)
        self.right_cmd_motor_sub = self.create_subscription(
            Float64, "/right_motor_cmd", self.right_cmd_motor_callback, 10
        )

        self.left_encoder_motor_sub = self.create_subscription(
            Float64, "/left_motor_encoder", self.left_encoder_motor_callback, 10
        )
        self.right_encoder_motor_sub = self.create_subscription(
            Float64, "/right_motor_encoder", self.right_encoder_motor_callback, 10
        )

        # self.encoder_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)
        # ROS visualization
        self.viz_geofence_pub = self.create_publisher(PolygonStamped, "drive/viz/geofence", 10)
        self.viz_path_pub = self.create_publisher(Path, "drive/viz/predicted_path", 10)
        self.viz_goal_pub = self.create_publisher(PoseStamped, "drive/viz/goal", 10)
        self.viz_current_state = self.create_publisher(String, "drive/viz/current_state", 10)
        self.viz_nb_steps_completed = self.create_publisher(String, "drive/viz/nb_steps_completed", 10)
        self.viz_help_msg_pub = self.create_publisher(String, "drive/viz/help_msg", 10)
        
        # Srv
        self.create_service(SetBool, "drive/yes_no", self.yes_no)
        self.get_logger().info("Drive ROS bridge started")

    def yes_no(self, req: Bool, resp):

        #self.get_logger().info(f"{ self.get_timestamp_s()}")

        self.calib.update_step(req.data, self.get_timestamp_s())

        return resp

    def control_loop(self):
        """Execute the loop"""
        current_time_s = self.get_timestamp_s()

        # Drive core loop
        # self.get_logger().info(f"Starting time: {self.calib.current_calbiration.starting_time}")
        # self.get_logger().info(f"elapsed_time_since_command {self.calib.current_calbiration.time_elapsed:.2f} s")
        self.calib.run(self.data, current_time_s)
        #self.get_logger().info(f"Current state: {self.data.right_motor_command}")
        #self.get_logger().info(f"Current state: {self.data.right_motor_encoder}")
        update_parameter_from_dataclass(self, self.calib.current_calbiration.params)
        update_parameter_from_dataclass(self, self.calib.params)
        # self.get_logger().info(f"path dataset {self.calib.current_calbiration.path} s")
        # ROS visualization
        self.publish_vizualisations()

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def send_goal(self, goal_pose: Pose):
        quat = quat = R.from_euler("xyz", goal_pose[3:6]).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = goal_pose[0]
        pose_msg.pose.position.y = goal_pose[1]
        pose_msg.pose.position.z = goal_pose[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.current_goal = goal_pose

        self.goal_pub.publish(pose_msg)

    def goal_reached_callback(self, pose_msg: PoseStamped):
        self.current_goal = None
        self.robot.goal_reached_callback()

    def loc_callback(self, pose_msg: PoseStamped):
        quaternion = [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
        roll, pitch, yaw = R.from_quat(quaternion).as_euler("xyz")
        pose = np.array(
            [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, roll, pitch, yaw]
        )

        current_time_ns = self.get_clock().now().nanoseconds

        self.robot.pose_callback(pose, current_time_ns)

    def deadman_callback(self, msg: Bool):
        self.robot.deadman_switch_callback(not msg.data)

    def get_timestamp_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def get_timestamp_s(self) -> float:
        timestamp = self.get_clock().now().seconds_nanoseconds()
        return timestamp[0] + timestamp[1] * 10 ** (-9)

    def encoder_odom_callback(self, msg: Odometry):
        timestamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel_x = msg.twist.twist.linear.x
        vel_yaw = msg.twist.twist.angular.z
        self.data.odom_encoder = np.array([timestamp_s, vel_x, vel_yaw])

    def left_cmd_motor_callback(self, msg: Float64):

        timestamp_s = self.get_timestamp_s()
        self.data.left_motor_command = np.array([timestamp_s, msg.data])

    def right_cmd_motor_callback(self, msg: Float64):

        timestamp_s = self.get_timestamp_s()
        self.data.right_motor_command = np.array([timestamp_s, msg.data])

    def left_encoder_motor_callback(self, msg: Float64):

        timestamp_s = self.get_timestamp_s()
        self.data.left_motor_encoder = np.array([timestamp_s, msg.data])

    def right_encoder_motor_callback(self, msg: Float64):

        timestamp_s = self.get_timestamp_s()
        self.data.right_motor_encoder = np.array([timestamp_s, msg.data])

    def shapely_to_ros_geom(self, polygon, global_frame):

        wheel_constraints = PolygonStamped()
        wheel_constraints.header.frame_id = global_frame
        wheel_constraints.header.stamp = self.get_clock().now().to_msg()
        points = []
        for i in range(len(polygon.exterior.xy[0])):
            x = polygon.exterior.xy[0][i]
            y = polygon.exterior.xy[1][i]
            points.append(Point32(x=x, y=y, z=0.0))

        wheel_constraints.polygon.points = points
        return wheel_constraints

    def publish_vizualisations(self):
        current_state = self.calib.state

        global_frame = "map"

        # Current state
        current_state_msg = String()
        current_state_msg.data = f"substate = {current_state}"
        self.viz_current_state.publish(current_state_msg)
        self.viz_current_calibration_pub.publish(
            String(data=f"Calibration : {self.calib.current_calbiration.calibration_name}")
        )
        # Help msg
        help_msg = String()
        help_msg.data = self.calib.screen_msg
        self.viz_help_msg_pub.publish(help_msg)

        self.get_logger().info(self.calib.current_calbiration.debug)

        # sampling_space visualization
        if self.calib.current_calbiration.state == "computing":
            # Wheel constraints
            ros_geom_msg = self.shapely_to_ros_geom(
                self.calib.current_calbiration.sampling_space_wheel_constraints, global_frame
            )
            self.wheel_constraints_pub.publish(ros_geom_msg)

            # Body constraints
            ros_geom_msg = self.shapely_to_ros_geom(
                self.calib.current_calbiration.sampling_space_body_constraints, global_frame
            )
            self.body_constraints_pub.publish(ros_geom_msg)

            # Resulting sampling space
            ros_geom_msg = self.shapely_to_ros_geom(
                self.calib.current_calbiration.resulting_sampling_space, global_frame
            )
            self.sampling_space_pub.publish(ros_geom_msg)
        # Odom 
        ros_marker_msg = Marker()
        ros_marker_msg.header.frame_id = global_frame
        ros_marker_msg.header.stamp = self.get_clock().now().to_msg()
        ros_marker_msg.type = Marker.SPHERE
        ros_marker_msg.action = Marker.ADD
        self.get_logger().info(f"Encoder odom {self.calib.current_calbiration.mean_encoder_odom}")
        ros_marker_msg.pose.position.x = self.calib.current_calbiration.mean_encoder_odom[0]
        ros_marker_msg.pose.position.y = self.calib.current_calbiration.mean_encoder_odom[1]
        ros_marker_msg.pose.position.z = 0.0
        ros_marker_msg.pose.orientation.x = 0.0
        ros_marker_msg.pose.orientation.y = 0.0
        ros_marker_msg.pose.orientation.z = 0.0
        ros_marker_msg.pose.orientation.w = 1.0
        ros_marker_msg.scale.x = 0.2
        ros_marker_msg.scale.y = 0.2
        ros_marker_msg.scale.z = 0.2
        self.viz_encoder_odom_pub.publish(ros_marker_msg)
            
        # Predicted path
        poses = []
        if self.calib.state == "trajectory_vizualization" or self.calib.state == "computing" \
            or self.calib.state == "linear_command_validation" or self.calib.state == "angular_command_validation":
            
            ros_marker_msg = Marker()
            ros_marker_msg.header.frame_id = global_frame
            ros_marker_msg.header.stamp = self.get_clock().now().to_msg()
            ros_marker_msg.type = Marker.SPHERE
            ros_marker_msg.action = Marker.ADD
            ros_marker_msg.pose.position.x = self.calib.current_calbiration.command_to_send[0]
            ros_marker_msg.pose.position.y = self.calib.current_calbiration.command_to_send[1]
            ros_marker_msg.pose.position.z = 0.0
            ros_marker_msg.pose.orientation.x = 0.0
            ros_marker_msg.pose.orientation.y = 0.0
            ros_marker_msg.pose.orientation.z = 0.0
            ros_marker_msg.pose.orientation.w = 1.0
            ros_marker_msg.scale.x = 0.2
            ros_marker_msg.scale.y = 0.2
            ros_marker_msg.scale.z = 0.2
            
            
            self.viz_current_commend_pub.publish(ros_marker_msg)

            

            v_x, omega_z = self.calib.current_calbiration.command_to_send

            #self.get_logger().info(f"Command to send {self.calib.current_calbiration.command_to_send}") 

            x, y, z, roll, pitch, yaw = self.robot.pose
            t = 0.0
            dt = 1.0 / self.params.protocol_frequency  # s

            current_tf= np.array([[np.cos(yaw), -np.sin(yaw), 0.0, x],
                               [np.sin(yaw), np.cos(yaw), 0.0, y],
                               [0.0, 0.0, 1.0, z],
                               [0.0, 0.0, 0.0, 1.0]])
            
            while t <= self.calib.current_calbiration.step_duration_s:

                
                x = current_tf[0, 3]
                y = current_tf[1, 3]
                angles = R.from_matrix(current_tf[:3, :3]).as_quat(scalar_first= False)
                

                pose = PoseStamped()
                pose.header.frame_id = global_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = angles[0]
                pose.pose.orientation.y = angles[1]
                pose.pose.orientation.z = angles[2]
                pose.pose.orientation.w = angles[3]
                poses.append(pose)

                t += dt
                delta_yaw = omega_z * dt

                delta_tf = np.array([[np.cos(delta_yaw), -np.sin(delta_yaw), 0.0, v_x * dt ],
                                     [np.sin(delta_yaw), np.cos(delta_yaw), 0.0, 0.0],
                                     [0.0, 0.0, 1.0, 0.0],
                                     [0.0, 0.0, 0.0, 1.0]])
                
                current_tf = np.dot(current_tf, delta_tf)
                

            path_msg = Path()
            path_msg.header.frame_id = global_frame
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = poses

            self.viz_path_pub.publish(path_msg)

        # Goal
        if self.current_goal is not None:
            quat = R.from_euler("xyz", self.current_goal[3:6]).as_quat()


            goal_msg = PoseStamped()
            goal_msg.header.frame_id = global_frame
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = self.current_goal[0]
            goal_msg.pose.position.y = self.current_goal[1]
            goal_msg.pose.position.z = self.current_goal[2]
            goal_msg.pose.orientation.x = quat[0]
            goal_msg.pose.orientation.y = quat[1]
            goal_msg.pose.orientation.z = quat[2]
            goal_msg.pose.orientation.w = quat[3]

            self.viz_goal_pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosCalibration()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

from dataclasses import dataclass
import datetime
from typing import Literal
import pathlib

import numpy as np
import rclpy
from drive_ros.node_utils import (
    redirect_logging_to_ros2,
    declare_parameter_from_dataclass,
    update_parameter_from_dataclass,
)

from drive_ros.calibration_node_utils import compute_sampling_space
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty, SetBool
import tf_transformations
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
    wheel_speed_encoder = np.array([0.0, 0.0, 0.0]) # timestamp_sec, wheel speed l, wheel speed r
    left_motor_command = np.array([0.0, 0.0])
    right_motor_command = np.array([0.0,0.0])
    left_motor_encoder = np.array([0.0,0.0])
    right_motor_encoder = np.array([0.0,0.0])
    odom_encoder = np.array([[0.0,0.0,0.0]])

@dataclass
class DriveRosBridgeParams:
    nb_steps: int = 10
    step_duration_s: float = 6.0

    datasets_directory: str = f"{pathlib.Path.home()}/drive_datasets"
    dataset_name: str = datetime.datetime.now().strftime(f"calibration_%Y-%m-%d_%H-%M-%S")
    protocol_frequency: float = 10.0

    command_sampling_strategy: str = "diff_drive"

    # Random sampling strategy parameters
    min_linear_speed: float = -1.0
    max_linear_speed: float = 1.0
    min_angular_speed: float = -2.0
    max_angular_speed: float = 2.0

    # Diff drive sampling strategy parameters
    wheel_radius: float = 1.0
    base_width: float = 1.0
    min_wheel_speed: float = -1.0
    max_wheel_speed: float = 1.0

PossibleState = Literal["do_this_calibration", "trajectory_vizualization",
                         "calibration_finished", "executing_command", "validation_of_sampling_space"]
class SampleSpaceIdentifier:

    def __init__(self, params: DriveRosBridgeParams, robot: Robot):
        self.calibration_name = "sample_space_identification"
        self.state: PossibleState = "do_this_calibration"
        self.screen_msg = "Do you want to do the sample space identification?"
        self.sample_space = np.array([0.0,0.0])
        self.robot = robot
        self.params = params
        self.starting_time = 10**10
        self.step_duration_s = 6.0
        self.command_to_send = np.array([0,0])
        self.left_wheel_encorder_buffer = np.array([0.0, 0.0])
        self.right_wheel_encorder_buffer = np.array([0.0, 0.0])
        
        self.nb_second_to_computed_top_speed = 2
        self.max_wheel_speed = 0
    def get_screen_msg(self):
        if self.state == "do_this_calibration":
            self.screen_msg = f"Do you want to do the {self.calibration_name} calibration ?"
        elif self.state == "trajectory_vizualization":
            self.screen_msg = "Are you ready to execute the projected trajectory?"
        elif self.state == "validation_of_sampling_space":
            self.screen_msg = f"The resulting sampling space is in the folder {self.params.datasets_directory}. \n Do you want to change parameters ?"
        elif self.state == "executing_command":
            self.screen_msg = "Executing command watch the robot move..."
        elif self.state == "calibration_finished":
            self.screen_msg = "You have finished the sampling space calibration."

    def update_step(self, update_step: bool, timestamp_s: float):
        
        if self.state == "do_this_calibration":
            if update_step:
                self.state = "trajectory_vizualization"
                self.command_to_send = Command(np.array([self.params.max_linear_speed,0.0]))
            else:
                self.state = "calibration_finished"
                
        elif self.state == "trajectory_vizualization":
            ### Send the command to the robot and wait for its execution
            if update_step:
                self.state = "executing_command"
                self.starting_time = timestamp_s
                self.robot.send_command(self.command_to_send)

        elif self.state == "validation_of_sampling_space":

            if update_step:
                self.state = "calibration_finished"
            else:
                self.state = "trajectory_vizualization"
                self.command_to_send = Command(np.array([self.params.max_linear_speed,0.0]))
        
        return self.state, self.screen_msg

    def execution_logic(self, timestamp_s: float, data: CalibData):

        if self.state == "executing_command":
            
            self.left_wheel_encorder_buffer = np.vstack((self.left_wheel_encorder_buffer, data.left_motor_encoder))
            self.right_wheel_encorder_buffer = np.vstack((self.right_wheel_encorder_buffer, data.right_motor_encoder))

            if (timestamp_s - self.starting_time) < self.step_duration_s:
                self.state = "validation_of_sampling_space"
                # End of the recording 
                self.robot.send_command(np.array([0.0,0.0]))

                
                # Compute maximum wheel speed 
                nb_indices = int(1 / self.params.protocol_frequency * self.nb_second_to_computed_top_speed)
                left_wheel_max = np.mean(self.left_wheel_encorder_buffer[-nb_indices:,1])
                right_wheel_max = np.mean(self.right_wheel_encorder_buffer[-nb_indices:,1])

                self.max_wheel_speed = np.max(np.array([left_wheel_max,right_wheel_max]))

                # Compute sampling_space
                self.sample_space = compute_sampling_space(float(self.max_wheel_speed),self.params)

        self.get_screen_msg()
        

    


class SamplingSpaceValidation:

    def __init__(self, params: DriveRosBridgeParams,robot: Robot):
        self.calibration_name = "sample_space_identification"
        self.state: PossibleState = "do_this_calibration"
        self.screen_msg = "Do you want to do the sample space identification?"
        self.sample_space = np.array([0.0,0.0])
        self.robot = robot
        self.params = params
        self.starting_time = 10**10
        self.step_duration_s = 6.0
        self.command_to_send = np.array([0,0])
        self.wheel_encorder_buffer = np.array([0.0, 0.0,0.0])
        self.nb_second_to_computed_top_speed = 2
        self.max_wheel_speed = 0

    def update_step(self, update_step: bool = True):
        self.change_of_state = True
        if self.state == "do_this_calibration":
            if update_step:
                self.state = "trajectory_vizualization"
                self.screen_msg = "Are you ready to execute the projected trajectory?"
            else:
                self.state = "calibration_finished"
                self.screen_msg = "You have skipped the sampling space calibration."
        elif self.state == "trajectory_vizualization":

            ### Send the command to the robot and wait for its execution
            if update_step:
                self.state = "executing_command"
                self.screen_msg = "Executing command watch the robot move..."

        elif self.state == "validation_of_sampling_space":

            if update_step:
                self.state = "calibration_finished"
                self.screen_msg = "You have successfully identified the sample space."
            else:
                self.state = "trajectory_vizualization"
                self.screen_msg = "Are you ready to execute the projected trajectory?"

        return self.state, self.screen_msg

    
    def execution_logic(self, timestamp_s: float, data: CalibData):

        if self.state == "executing_command":
            
            self.wheel_encorder_buffer = np.vstack((self.wheel_encorder_buffer, data.wheel_speed_encoder))

        #    if (timestamp_s - self.starting_time) < self.step_duration_s:
        #        self.state = "validation_of_sampling_space"
        #        # End of the recording 
        #        self.robot.send_command(np.array([0.0,0.0]))

        #        
        #        # Compute maximum wheel speed 
        #        nb_indices = int(1 / self.params.protocol_frequency * self.nb_second_to_computed_top_speed)
        #        left_wheel_max = np.mean(self.wheel_encorder_buffer[-nb_indices:,1])
        #        right_wheel_max = np.mean(self.wheel_encorder_buffer[-nb_indices:,2])

        #        self.max_wheel_speed = np.max(np.array([left_wheel_max,right_wheel_max]))

        #        # Compute sampling_space
        #        self.sample_space = compute_sampling_space(float(self.max_wheel_speed),self.params)

        #self.get_screen_msg()
        #self.compute_projected_trajectory()

    

class DriveCalibration:

    def __init__(self, param: DriveRosBridgeParams,robot:Robot):

        self.current_calbiration = SampleSpaceIdentifier(param,robot)
        self.params = param
        self.state = self.current_calbiration.state
        self.screen_msg = self.current_calbiration.screen_msg
        self.robot = robot
        

    def update_step(self, update_step: bool = True):

        
        if self.current_calbiration.state == "calibration_finished":

            if self.current_calbiration.calibration_name == "sample_space_identification":
                # Save the sample space to a file or database
                print("Sample space calibration completed and saved.")
                #self.current_calbiration = SamplingSpaceValidation(self.params, self.robot)
                self.state = "All calibration are finished"
                self.screen_msg = "You have finished the calibration node. Enjoy your DRIVE"

            elif self.current_calbiration.calibration_name == "sampling_space_validation":
                self.state = "All calibration are finished"
                self.screen_msg = "You have finished the calibration node. Enjoy your DRIVE"

    def run(self,data: CalibData, timestamp_s : float):

        self.current_calbiration.execution_logic(timestamp_s,data)


class DriveRosCalibration(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.params = DriveRosBridgeParams()
        declare_parameter_from_dataclass(self, self.params)
        self.create_timer(1.0, lambda: update_parameter_from_dataclass(self, self.params))

        self.dataset_directory = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name
        self.current_goal: Pose | None = None

        # Drive calib core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.calib = DriveCalibration(self.params,self.robot)
        self.data = CalibData()
        # ROS setup
        delay = 1.0 / self.params.protocol_frequency
        self.get_logger().info(f"Control loop frequency: {self.params.protocol_frequency} Hz (delay: {delay:.3f} s)")
        self.timer = self.create_timer(delay, self.control_loop)

        # Pubs
        self.cmd_pub = self.create_publisher(Twist, "cmd_drive", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)

        # Subs
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "pause_drive", self.deadman_callback, 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)
        self.encoder_odom_sub  = self.create_subscription(Odometry, "/encoder_odom", self.encoder_odom_callback,10)
        
        # Assume that cmd _motor  = Float64 
        self.left_cmd_motor_sub  = self.create_subscription(Float64, "/left_motor_cmd", self.left_cmd_motor_callback,10)
        self.right_cmd_motor_sub  = self.create_subscription(Float64, "/right_motor_cmd", self.right_cmd_motor_callback,10)
        
        self.left_encoder_motor_sub  = self.create_subscription(Float64, "/left_motor_encoder", self.left_encoder_motor_callback,10)
        self.right_encoder_motor_sub  = self.create_subscription(Float64, "/right_motor_encoder", self.right_encoder_motor_callback,10)
        
        #self.encoder_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)
        # ROS visualization
        self.viz_geofence_pub = self.create_publisher(PolygonStamped, "drive/viz/geofence", 10)
        self.viz_path_pub = self.create_publisher(Path, "drive/viz/predicted_path", 10)
        self.viz_goal_pub = self.create_publisher(PoseStamped, "drive/viz/goal", 10)
        self.viz_current_state = self.create_publisher(String, "drive/viz/current_state", 10)
        self.viz_nb_steps_completed = self.create_publisher(String, "drive/viz/nb_steps_completed", 10)
        self.viz_help_msg_pub = self.create_publisher(String, "drive/viz/help_msg", 10)

        self.get_logger().info("Drive ROS bridge started")

    def control_loop(self):
        """Execute the loop"""
        current_time_ns = self.get_timestamp_ns()

        # Drive core loop

        self.calib.run(self.data, current_time_ns)

        # ROS visualization
        self.publish_vizualisations()

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def send_goal(self, goal_pose: Pose):
        quat = tf_transformations.quaternion_from_euler(goal_pose[3], goal_pose[4], goal_pose[5])
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
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
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
        return timestamp[0] + timestamp[1] * 10**(-9)

    def encoder_odom_callback(self, msg: Odometry):
        timestamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel_x = msg.twist.twist.linear.x
        vel_yaw = msg.twist.twist.angular.z 
        self.data.odom_encoder = np.array([timestamp_s,vel_x,vel_yaw])

    def left_cmd_motor_callback(self, msg: Float64):

    
        timestamp_s = self.get_timestamp_s()
        self.data.left_motor_command =  np.array([timestamp_s,msg.data])

    def right_cmd_motor_callback(self, msg: Float64):

    
        timestamp_s = self.get_timestamp_s()
        self.data.right_motor_command = np.array([timestamp_s,msg.data])

    def left_encoder_motor_callback(self, msg: Float64):

    
        timestamp_s = self.get_timestamp_s()
        self.data.left_motor_encoder = np.array([timestamp_s,msg.data])

    def right_encoder_motor_callback(self, msg: Float64):
    
        timestamp_s = self.get_timestamp_s()
        self.data.right_motor_encoder =  np.array([timestamp_s,msg.data])


    def publish_vizualisations(self):
        current_state = self.calib.state
        
        global_frame = "map"

        # Current state
        current_state_msg = String()
        current_state_msg.data = current_state
        self.viz_current_state.publish(current_state_msg)

        # Help msg
        help_msg = String()
        help_msg.data = self.calib.screen_msg
        self.viz_help_msg_pub.publish(help_msg)

        # Predicted path
        poses = []
        if self.calib.state == "trajectory_vizualization":
            v_x, omega_z = self.calib.current_calbiration.command_to_send

            x, y, z, roll, pitch, yaw = self.robot.pose
            t = 0.0
            dt = 1.0 / self.params.protocol_frequency  # s

            while t <= self.calib.current_calbiration.step_duration_s:
                x += v_x * dt * np.cos(yaw)
                y += v_x * dt * np.sin(yaw)
                yaw += omega_z * dt

                pose = PoseStamped()
                pose.header.frame_id = global_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                poses.append(pose)

                t += dt

        path_msg = Path()
        path_msg.header.frame_id = global_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses

        self.viz_path_pub.publish(path_msg)

        # Goal
        if self.current_goal is not None:
            quat = tf_transformations.quaternion_from_euler(
                self.current_goal[3], self.current_goal[4], self.current_goal[5]
            )

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

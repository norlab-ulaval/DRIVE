#!/usr/bin/env python3

import os

from dataclasses import dataclass
import datetime

import pathlib

import numpy as np
import rclpy
from drive_ros.node_utils import (
    redirect_logging_to_ros2,
    declare_parameter_from_dataclass,
    update_parameter_from_dataclass,
)
from std_msgs.msg import String
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Point32, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Path, Odometry


from DRIVE.common import Pose
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
class DriveRosBridgeParams:
    seed: int = 0  # 0 = random, anything else = set seed
    nb_steps: int = 10
    step_duration_s: float = 6.0

    dataset_directory: str = f"{pathlib.Path.home()}/drive_datasets/{datetime.datetime.now().strftime(f'%Y-%m-%d_%H-%M-%S')}/drive_node_data"
    protocol_frequency: float = 10.0

    # Protocol limits in body frame
    min_linear_speed: float = -1.0
    max_linear_speed: float = 1.0
    min_angular_speed: float = -2.0
    max_angular_speed: float = 2.0

    command_sampling_strategy: str = "diff_drive"
    
    # Localization message type ("PoseStamped" or "Odometry")
    localization_topic_type: str = "PoseStamped"
    # Cmd vel message type ("Twist" or "TwistStamped")
    cmd_vel_topic_type: str = "Twist"

    # Diff drive sampling strategy parameters
    wheel_radius: float = 1.0
    base_width: float = 1.0
    min_wheel_speed: float = -1.0
    max_wheel_speed: float = 1.0


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        redirect_logging_to_ros2(self)

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.params = DriveRosBridgeParams()
        declare_parameter_from_dataclass(self, self.params)
        self.create_timer(1.0, lambda: update_parameter_from_dataclass(self, self.params))

        self.current_goal: Pose | None = None

        seed = self.params.seed
        if self.params.seed == 0:
            seed = None

        self.dataset_directory = pathlib.Path(self.params.dataset_directory)
        if self.dataset_directory.exists() and any(self.dataset_directory.iterdir()):
            self.get_logger().error(f"Dataset directory {self.dataset_directory} already exists and is not empty. Exiting.")
            exit(1)

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        strategy = CommandSamplingFactory.create_sampling_strategy(
            self.params.command_sampling_strategy, self.params.__dict__, seed
        )
        self.drive = Drive(
            self.robot,
            strategy,
            self.params.nb_steps,
            self.params.step_duration_s,
            self.dataset_directory,
        )

        # ROS setup
        delay = 1.0 / self.params.protocol_frequency
        self.get_logger().info(f"Control loop frequency: {self.params.protocol_frequency} Hz (delay: {delay:.3f} s)")
        self.timer = self.create_timer(delay, self.control_loop)

        # Pubs
        if self.params.cmd_vel_topic_type == "TwistStamped":
            self.cmd_pub = self.create_publisher(TwistStamped, "cmd_drive", 10)
        else:  # Default to Twist
            self.cmd_pub = self.create_publisher(Twist, "cmd_drive", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)

        # Subs
        if self.params.localization_topic_type == "Odometry":
            self.loc_sub = self.create_subscription(Odometry, "pose", self.loc_callback, 10)
        else:  # Default to PoseStamped
            self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "pause_drive", self.deadman_callback, 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)

        # ROS visualization
        self.viz_geofence_pub = self.create_publisher(PolygonStamped, "drive/viz/geofence", 10)
        self.viz_path_pub = self.create_publisher(Path, "drive/viz/predicted_path", 10)
        self.viz_goal_pub = self.create_publisher(PoseStamped, "drive/viz/goal", 10)
        self.viz_current_state = self.create_publisher(String, "drive/viz/current_state", 10)
        self.viz_nb_steps_completed = self.create_publisher(String, "drive/viz/nb_steps_completed", 10)
        self.viz_help_msg_pub = self.create_publisher(String, "drive/viz/help_msg", 10)
        self.create_service(Empty, "drive/next_state", self.next_state_cb)
        self.create_service(Empty, "drive/skip_step", self.skip_step_cb)
        self.create_service(Empty, "drive/stop_drive", self.stop_drive_cb)

        self.get_logger().info("Drive ROS bridge started")

    def extract_pose_from_message(self, msg):
        """Extract pose array from either PoseStamped or Odometry message"""
        if isinstance(msg, PoseStamped):
            pose_msg = msg.pose
        elif isinstance(msg, Odometry):
            pose_msg = msg.pose.pose
        else:
            self.get_logger().error(f"Unsupported message type: {type(msg)}")
            return None
            
        quaternion = [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ]
        roll, pitch, yaw = R.from_quat(quaternion).as_euler("xyz")
        pose = np.array(
            [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, roll, pitch, yaw]
        )
        return pose

    def control_loop(self):
        current_time_ns = self.get_timestamp_ns()

        # Drive core loop
        self.drive.run(current_time_ns)

        # ROS visualization
        self.publish_vizualisations()

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        if self.params.cmd_vel_topic_type == "TwistStamped":
            msg_stamped = TwistStamped()
            msg_stamped.header.stamp = self.get_clock().now().to_msg()
            msg_stamped.twist = msg
            self.cmd_pub.publish(msg_stamped)
        else:
            self.cmd_pub.publish(msg)

    def send_goal(self, goal_pose: Pose):
        quat = R.from_euler("xyz", goal_pose[3:6]).as_quat()

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

    def loc_callback(self, msg):
        """Handle both PoseStamped and Odometry messages"""
        pose = self.extract_pose_from_message(msg)
        if pose is None:
            return
            
        current_time_ns = self.get_clock().now().nanoseconds
        self.robot.pose_callback(pose, current_time_ns)

    def deadman_callback(self, msg: Bool):
        self.robot.deadman_switch_callback(not msg.data)

    def get_timestamp_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_vizualisations(self):
        current_state = self.drive.current_state.__class__
        global_frame = "map"

        # Current state
        current_state_msg = String()
        current_state_msg.data = self.drive.current_state.get_state_name()
        self.viz_current_state.publish(current_state_msg)

        # Help msg
        help_msg = String()
        help_msg.data = self.drive.get_help_message()
        self.viz_help_msg_pub.publish(help_msg)

        # Nb steps completed
        nb_step_msg = String()
        if current_state not in (RunningState, PausedState, BackToGeofenceState):
            nb_step_msg.data = "Not started"
        else:
            current = len(self.drive.completed_commands)
            target = self.drive.target_nb_steps
            nb_step_msg.data = f"{current} / {target} steps completed ({(current/target)*100:.0f}%)"
        self.viz_nb_steps_completed.publish(nb_step_msg)

        # Geofence
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = global_frame
        polygon_msg.header.stamp = self.get_clock().now().to_msg()

        point_msgs = []
        for point in self.drive.get_geofence_points():
            point_msgs.append(Point32(x=point[0], y=point[1], z=0.0))

        polygon_msg.polygon.points = point_msgs

        self.viz_geofence_pub.publish(polygon_msg)

        # Predicted path
        poses = []
        if self.drive.current_step is not None and current_state == RunningState:
            v_x, omega_z = self.drive.current_step.command
            x, y, z, roll, pitch, yaw = self.drive.current_step.start_pose
            t = 0.0
            dt = 1.0 / self.params.protocol_frequency  # s

            while t <= self.drive.step_duration_s:
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

    def next_state_cb(self, req, resp):
        current_state = self.drive.current_state.__class__
        timestamp_ns = self.get_timestamp_ns()

        if current_state == WaitingState:
            self.drive.start_geofence(timestamp_ns)
        elif current_state == GeofenceCreationState:
            self.drive.confirm_geofence(timestamp_ns)
        elif current_state == ReadyState:
            self.drive.start_drive(timestamp_ns)
        elif current_state == BackToGeofenceState:
            self.drive.resume_drive(timestamp_ns)

        return resp

    def skip_step_cb(self, req, resp):
        current_state = self.drive.current_state.__class__
        timestamp_ns = self.get_timestamp_ns()

        if current_state in (RunningState, PausedState, BackToGeofenceState):
            self.drive.skip_current_step(timestamp_ns)

        return resp

    def stop_drive_cb(self, req, resp):
        timestamp_ns = self.get_timestamp_ns()
        current_state = self.drive.current_state.__class__

        if current_state in (RunningState, PausedState, BackToGeofenceState):
            self.drive.stop_drive(timestamp_ns)

        return resp


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosBridge()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

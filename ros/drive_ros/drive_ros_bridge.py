#!/usr/bin/env python3

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
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Point32, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Path


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
    nb_steps: int = 10
    step_duration_s: float = 6.0
    command_sampling_strategy: str = "random"
    min_linear_speed: float = 0.5
    max_linear_speed: float = -0.5
    min_angular_speed: float = 1.0
    max_angular_speed: float = -1.0
    datasets_directory: str = f"{pathlib.Path.home()}/drive_datasets"
    dataset_name: str = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S")
    protocol_frequency: float = 40.0


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        redirect_logging_to_ros2(self)

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.params = DriveRosBridgeParams()
        declare_parameter_from_dataclass(self, self.params)
        self.create_timer(1.0, lambda: update_parameter_from_dataclass(self, self.params))

        self.dataset_directory = pathlib.Path(self.params.datasets_directory) / self.params.dataset_name

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.command_sampling_strategy = CommandSamplingFactory.create_sampling_strategy(
            self.command_sampling_strategy_str,  # type: ignore
            self.params.min_linear_speed,
            self.params.max_linear_speed,
            self.params.min_angular_speed,
            self.params.max_angular_speed,
        )
        self.drive = Drive(
            self.robot,
            self.command_sampling_strategy,
            self.params.nb_steps,
            self.params.step_duration_s,
            self.dataset_directory,
        )

        # ROS setup
        delay = 1.0 / self.params.protocol_frequency
        self.get_logger().info(f"Control loop frequency: {self.params.protocol_frequency} Hz (delay: {delay:.3f} s)")
        self.timer = self.create_timer(delay, self.control_loop)

        # Pubs
        self.cmd_pub = self.create_publisher(Twist, "cmd_drive", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)

        # Subs
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "deadman", self.deadman_callback, 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)

        # ROS visualization
        self.viz_geofence_pub = self.create_publisher(PolygonStamped, "drive/viz/geofence", 10)
        self.viz_path_pub = self.create_publisher(Path, "drive/viz/predicted_path", 10)
        self.viz_current_state = self.create_publisher(String, "drive/viz/current_state", 10)
        self.viz_nb_steps_completed = self.create_publisher(String, "drive/viz/nb_steps_completed", 10)
        self.viz_help_msg_pub = self.create_publisher(String, "drive/viz/help_msg", 10)
        self.create_service(Empty, "drive/next_state", self.next_state_cb)
        self.create_service(Empty, "drive/skip_step", self.skip_step_cb)
        self.create_service(Empty, "drive/stop_drive", self.stop_drive_cb)

        self.get_logger().info("Drive ROS bridge started")

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

        self.goal_pub.publish(pose_msg)

    def goal_reached_callback(self, pose_msg: PoseStamped):
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
        self.robot.deadman_switch_callback(msg.data)

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
            dt = 0.025  # s

            while t <= self.drive.step_duration_s:
                x += v_x * np.cos(yaw)
                y += v_x * np.sin(yaw)
                yaw += omega_z

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

    def next_state_cb(self, req, resp):
        current_state = self.drive.current_state.__class__
        timestamp_ns = self.get_timestamp_ns()

        if current_state == WaitingState:
            self.drive.start_geofence(timestamp_ns)
        elif current_state == GeofenceCreationState:
            self.drive.confirm_geofence(timestamp_ns)
        elif current_state == ReadyState:
            self.drive.start_drive(timestamp_ns)
        elif current_state in (RunningState, PausedState, BackToGeofenceState):
            self.drive.stop_drive(timestamp_ns)

        return resp

    def skip_step_cb(self, req, resp):
        current_state = self.drive.current_state.__class__
        timestamp_ns = self.get_timestamp_ns()

        if current_state in (RunningState, PausedState, BackToGeofenceState):
            self.drive.skip_current_step(timestamp_ns)

        return resp

    def stop_drive_cb(self, req, resp):
        timestamp_ns = self.get_timestamp_ns()

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

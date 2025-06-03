import datetime
import logging
import pathlib
from threading import Thread

import numpy as np
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Empty
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.msg import Path


from DRIVE.common import Pose
from DRIVE.drive import (
    BackToCenterState,
    Drive,
    GeofenceCreationState,
    PausedState,
    ReadyState,
    RunningState,
    WaitingState,
)
from DRIVE.robot import Robot
from DRIVE.sampling import CommandSamplingFactory


def redirect_logging_to_ros2():
    ros2_logger = rclpy.logging.get_logger("DRIVE")  # type: ignore

    class ROS2Handler(logging.Handler):
        def emit(self, record):
            log_entry = self.format(record)
            if record.levelno == logging.DEBUG:
                ros2_logger.debug(log_entry)
            elif record.levelno == logging.INFO:
                ros2_logger.info(log_entry)
            elif record.levelno == logging.WARNING:
                ros2_logger.warn(log_entry)
            elif record.levelno == logging.ERROR:
                ros2_logger.error(log_entry)
            elif record.levelno == logging.CRITICAL:
                ros2_logger.fatal(log_entry)

    root_logger = logging.getLogger()
    for handler in root_logger.handlers:
        root_logger.removeHandler(handler)

    ros2_handler = ROS2Handler()
    formatter = logging.Formatter("%(levelname)s: %(message)s")
    ros2_handler.setFormatter(formatter)
    root_logger.addHandler(ros2_handler)
    root_logger.setLevel(logging.INFO)


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        redirect_logging_to_ros2()

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Get ROS parameters
        self.declare_parameter("nb_steps", 10)
        self.declare_parameter("step_duration_s", 6.0)
        self.declare_parameter("command_sampling_strategy", "random")
        self.declare_parameter("min_linear_speed", 0.0)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", -1.0)
        self.declare_parameter("min_angular_speed", 1.0)
        self.declare_parameter("datasets_directory", f"{pathlib.Path.home()}/drive_datasets")
        self.declare_parameter("current_dataset_name", "drive-test")

        self.nb_steps: int = self.get_parameter("nb_steps").get_parameter_value().integer_value
        self.step_duration_s: float = self.get_parameter("step_duration_s").get_parameter_value().double_value
        self.command_sampling_strategy_str = self.get_parameter("command_sampling_strategy").value
        self.min_linear_speed: float = self.get_parameter("min_linear_speed").get_parameter_value().double_value
        self.max_linear_speed: float = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.min_angular_speed: float = self.get_parameter("min_angular_speed").get_parameter_value().double_value
        self.max_angular_speed: float = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.datasets_directory_str: str = self.get_parameter("datasets_directory").get_parameter_value().string_value
        self.current_dataset_name: str = self.get_parameter("current_dataset_name").get_parameter_value().string_value

        dataset_name = datetime.datetime.now().strftime(f"%Y-%m-%d_%H-%M-%S_{self.current_dataset_name}")
        self.dataset_directory = pathlib.Path(self.datasets_directory_str) / dataset_name

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.command_sampling_strategy = CommandSamplingFactory.create_sampling_strategy(
            self.command_sampling_strategy_str,  # type: ignore
            self.min_linear_speed,
            self.max_linear_speed,
            self.min_angular_speed,
            self.max_angular_speed,
        )
        self.drive = Drive(
            self.robot,
            self.command_sampling_strategy,
            self.nb_steps,
            self.step_duration_s,
            self.dataset_directory,
        )

        # ROS setup
        self.timer = self.create_timer(0.1, self.control_loop)

        # Pubs
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "deadman", self.deadman_callback, 10)

        # Subs
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)

        # ROS visualization
        self.viz_geofence_pub = self.create_publisher(PolygonStamped, "drive/viz/geofence", 10)
        self.viz_path_pub = self.create_publisher(Path, "drive/viz/predicted_path", 10)
        self.viz_current_state = self.create_publisher(String, "drive/viz/current_state", 10)
        self.viz_nb_steps_completed = self.create_publisher(String, "drive/viz/nb_steps_completed", 10)
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

    def get_timestamp_ns(self) -> float:
        return self.get_clock().now().nanoseconds

    def publish_vizualisations(self):
        current_state = self.drive.current_state.__class__
        global_frame = "map"

        # Current state
        current_state_msg = String()
        current_state_msg.data = self.drive.current_state.get_state_name()
        self.viz_current_state.publish(current_state_msg)

        # Nb steps completed
        nb_step_msg = String()
        if current_state not in (RunningState, PausedState, BackToCenterState):
            nb_step_msg.data = "Not started"
        else:
            current = len(self.drive.commands) - 1
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
        elif current_state in (RunningState, PausedState, BackToCenterState):
            self.drive.stop_drive(timestamp_ns)

        return resp

    def skip_step_cb(self, req, resp):
        current_state = self.drive.current_state.__class__
        timestamp_ns = self.get_timestamp_ns()

        if current_state in (RunningState, PausedState, BackToCenterState):
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

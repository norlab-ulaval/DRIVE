#!/usr/bin/env python3

from dataclasses import dataclass
import numpy as np
import rclpy
from drive_ros.node_utils import declare_parameter_from_dataclass, update_parameter_from_dataclass
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


@dataclass
class PControllerParams:
    goal_tolerance: float = 0.5
    max_linear_speed: float = 1.0
    max_angular_speed: float = 1.0
    linear_gain: float = 0.5
    angular_gain: float = 0.5
    angle_thesold_rotate_in_place: float = np.deg2rad(20.0)


class PController(Node):
    def __init__(self):
        super().__init__("p_controller", parameter_overrides=[])

        self.goal = None
        self.params = PControllerParams()

        declare_parameter_from_dataclass(self, self.params)
        self.create_timer(1.0, lambda: update_parameter_from_dataclass(self, self.params))

        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "goal", self.goal_callback, 10)

        self.goal_reached_pub = self.create_publisher(PoseStamped, "goal_reached", 10)
        self.command_pub = self.create_publisher(Twist, "cmd_ctrl", 10)

        self.get_logger().info("Diff drive sim node started")

    def goal_callback(self, goal_msg: PoseStamped):
        x = goal_msg.pose.position.x
        y = goal_msg.pose.position.y
        yaw = tf_transformations.euler_from_quaternion(
            [
                goal_msg.pose.orientation.x,
                goal_msg.pose.orientation.y,
                goal_msg.pose.orientation.z,
                goal_msg.pose.orientation.w,
            ]
        )[2]

        self.get_logger().info("Goal received, taking control")
        self.goal = (x, y, yaw)

    def loc_callback(self, pose_msg: PoseStamped):
        if self.goal is None:
            return

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        yaw = tf_transformations.euler_from_quaternion(
            [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )[2]
        goal_x, goal_y, goal_yaw = self.goal

        # Compute distance and heading to goal
        dx = goal_x - x
        dy = goal_y - y
        distance = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = (angle_to_goal - yaw + np.pi) % (2 * np.pi) - np.pi

        # Check if the goal is reached
        if distance < self.params.goal_tolerance:
            self.get_logger().info("Goal reached")
            self.goal = None
            stop_twist = Twist()
            self.command_pub.publish(stop_twist)
            self.goal_reached_pub.publish(pose_msg)
            return

        # P control
        linear_speed = self.params.linear_gain * distance
        angular_speed = self.params.angular_gain * angle_diff

        # Clip speeds to max limits
        linear_speed = np.clip(linear_speed, -self.params.max_linear_speed, self.params.max_linear_speed)
        angular_speed = np.clip(angular_speed, -self.params.max_angular_speed, self.params.max_angular_speed)

        # If the heading error is large, rotate in place
        if abs(angle_diff) > self.params.angle_thesold_rotate_in_place:
            linear_speed = 0.0

        # Publish the command
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.command_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    p_controller = PController()

    rclpy.spin(p_controller)

    p_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

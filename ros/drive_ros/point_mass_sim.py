#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R


class PointMassSim(Node):
    def __init__(self):
        super().__init__("point_mass_sim", parameter_overrides=[])
        self.pose = (0.0, 0.0, 0.0)

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.execute_command, 10)

        self.loc_pub = self.create_publisher(PoseStamped, "pose", 10)

        self.dt = 1.0 / 10.0
        self.loc_timer = self.create_timer(self.dt, self.localize)

        self.left_wheel_speed_pub = self.create_publisher(Float64, "left_wheel_encoder", 10)
        self.right_wheel_speed_pub = self.create_publisher(Float64, "right_wheel_encoder", 10)
        self.wheel_radius = 3.0
        self.base_width = 0.5
        self.get_logger().info("Point mass sim node started")

    def execute_command(self, twist: Twist):
        x, y, yaw = self.pose
        v_x, omega_z = twist.linear.x, twist.angular.z

        x += v_x * self.dt * np.cos(yaw)
        y += v_x * self.dt * np.sin(yaw)
        yaw += omega_z * self.dt
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

        self.pose = np.array([x, y, yaw])

        self.left_wheel_speed_pub.publish(Float64(data=v_x/self.wheel_radius))
        self.right_wheel_speed_pub.publish(Float64(data=v_x/self.wheel_radius)) 

    def localize(self):
        # Simulate some localization noise
        noisy_pose = self.pose + np.random.normal(0, 0.025, 3)

        quat = R.from_euler("xyz", [0.0, 0.0, noisy_pose[2]]).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = noisy_pose[0]
        pose_msg.pose.position.y = noisy_pose[1]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.loc_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    point_mass_sim = PointMassSim()

    rclpy.spin(point_mass_sim)

    point_mass_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

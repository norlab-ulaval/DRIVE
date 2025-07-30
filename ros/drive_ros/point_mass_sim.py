#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
from calibration_node_utils import inverse_kin, forward_kin

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
        self.left_wheel_speed_cmd_pub = self.create_publisher(Float64, "left_wheel_cmd", 10)
        self.right_wheel_speed_cmd_pub = self.create_publisher(Float64, "right_wheel_cmd", 10)
        self.wheel_radius = 1.0
        self.base_width = 1.0
        self.get_logger().info("Point mass sim node started")

        self.encoder_noise = 0.1  # Standard deviation of the noise added to the wheel speed
        self.max_linalg_speed = 0.5  # Maximum linear speed of the point mass
        self.max_angular_speed = 0.5  # Maximum angular speed of the point mass


    def execute_command(self, twist: Twist):
        
        x, y, yaw = self.pose
        v_x, omega_z = twist.linear.x, twist.angular.z

        if v_x > self.max_linalg_speed:
            v_x = self.max_linalg_speed
        elif v_x < -self.max_linalg_speed:
            v_x = -self.max_linalg_speed    
        if omega_z > self.max_angular_speed:
            omega_z = self.max_angular_speed
        elif omega_z < -self.max_angular_speed:
            omega_z = -self.max_angular_speed   

        x += v_x * self.dt * np.cos(yaw)
        y += v_x * self.dt * np.sin(yaw)
        yaw += omega_z * self.dt
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

        self.pose = np.array([x, y, yaw])
        
        jacobian = self.wheel_radius * np.array([[1 / 2, 1 / 2], [-1 / (self.base_width), 1 / (self.base_width)]])

        wheel_cmd = np.linalg.pinv(jacobian) @ np.array([v_x, omega_z])
       
        noize = np.random.normal(0, self.encoder_noise, wheel_cmd.shape)
        wheel_encoder = wheel_cmd + noize
        
        self.left_wheel_speed_pub.publish(Float64(data=wheel_encoder[0]))
        self.right_wheel_speed_pub.publish(Float64(data=wheel_encoder[1])) 

        self.left_wheel_speed_cmd_pub.publish(Float64(data=wheel_cmd[0]))
        self.right_wheel_speed_cmd_pub.publish(Float64(data=wheel_cmd[1]))

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

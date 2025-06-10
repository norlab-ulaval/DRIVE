import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class DiffDriveSim(Node):
    def __init__(self):
        super().__init__("diff_drive_sim", parameter_overrides=[])
        self.pose = (0.0, 0.0, 0.0)

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.execute_command, 10)

        self.loc_pub = self.create_publisher(PoseStamped, "pose", 10)

        self.loc_timer = self.create_timer(0.1, self.localize)

        self.get_logger().info("Diff drive sim node started")

    def execute_command(self, twist: Twist):
        x, y, yaw = self.pose
        v_x, omega_z = twist.linear.x, twist.angular.z

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])

    def localize(self):
        # Simulate some localization noise
        noisy_pose = self.pose + np.random.normal(0, 0.05, 3)

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, noisy_pose[2])

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

    diff_drive_sim = DiffDriveSim()

    rclpy.spin(diff_drive_sim)

    diff_drive_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

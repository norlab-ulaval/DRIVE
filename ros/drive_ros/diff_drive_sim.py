import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class DiffDriveSim(Node):
    def __init__(self):
        super().__init__("diff_drive_sim", parameter_overrides=[])
        self.pose = (0.0, 0.0, 0.0)
        self.goal = None

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.execute_command, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "goal", self.receive_goal, 10)
        self.loc_pub = self.create_publisher(PoseStamped, "pose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_reached", 10)
        self.loc_timer = self.create_timer(0.1, self.localize)

        self.get_logger().info("Diff drive sim node started")

    def execute_command(self, twist: Twist):
        x, y, yaw = self.pose
        v_x, omega_z = twist.linear.x, twist.angular.z

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])

    def receive_goal(self, goal_msg: PoseStamped):
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

        self.goal = (x, y, yaw)
        self.goal_msg = goal_msg
        self.goal_timer = self.create_timer(0.1, self.go_to_goal)

    def localize(self):
        # Simulate some localization noise
        noisy_pose = self.pose + np.random.normal(0, 0.05, 3)

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, noisy_pose[2])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = noisy_pose[0]
        pose_msg.pose.position.y = noisy_pose[1]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.loc_pub.publish(pose_msg)

    def go_to_goal(self):
        if self.goal is None:
            return

        x, y, yaw = self.pose
        goal_x, goal_y, goal_yaw = self.goal

        distance = np.sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)
        angle_to_goal = np.arctan2(goal_y - y, goal_x - x)
        angle_diff = angle_to_goal - yaw

        if distance < 0.1:
            self.get_logger().info("Goal reached")
            self.goal_pub.publish(self.goal_msg)
            self.goal = None
            self.goal_timer.cancel()
            return

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = angle_diff

        self.execute_command(twist)


def main(args=None):
    rclpy.init(args=args)

    diff_drive_sim = DiffDriveSim()

    rclpy.spin(diff_drive_sim)

    diff_drive_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

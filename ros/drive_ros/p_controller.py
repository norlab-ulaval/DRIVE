import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class PController(Node):
    def __init__(self):
        super().__init__("p_controller", parameter_overrides=[])
        self.goal = None

        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "goal", self.goal_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, "goal_reached", 10)
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

        distance = np.sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)
        angle_to_goal = np.arctan2(goal_y - y, goal_x - x)
        angle_diff = angle_to_goal - yaw

        if distance < 0.1:
            self.get_logger().info("Goal reached")
            self.goal_pub.publish(pose_msg)
            self.goal = None
            return

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = angle_diff
        self.command_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    p_controller = PController()

    rclpy.spin(p_controller)

    p_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

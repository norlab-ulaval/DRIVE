from typing import Callable

from DRIVE.common import Command, Pose


class Robot:
    """
    The Robot class provides a simple interface to connect DRIVE to your robot's control, navigation and localization
    systems. This class must be configured for each robot platform you want to use with DRIVE. First, you need to
    provide how to send control commands and goal poses to your robot. Then, you need to configure your system to
    call the necessary functions to update the robot's state.

    See drive_ros_bridge.py for an example of how to use this class with ROS2.
    """

    def __init__(
        self, initial_pose: Pose, send_command_fn: Callable[[Command], None], send_goal_fn: Callable[[Pose], None]
    ) -> None:
        self.pose = initial_pose
        self.send_command_fn = send_command_fn
        self.send_goal_fn = send_goal_fn

        self.deadman_switch_pressed = False
        self.goal_reached = False

        self.poses_buffer: list[tuple[Pose, int]] = []
        self.speeds_buffer: list[tuple[Pose, int]] = []
        self.accelerations_buffer: list[tuple[Pose, int]] = []

    def pose_callback(self, pose: Pose, timestamp_ns: int) -> None:
        self.poses_buffer.append((pose, timestamp_ns))
        self.pose = pose

    def speed_callback(self, speed: Pose, timestamp_ns: int) -> None:
        self.speeds_buffer.append((speed, timestamp_ns))

    def acceleration_callback(self, acceleration: Pose, timestamp_ns: int) -> None:
        self.accelerations_buffer.append((acceleration, timestamp_ns))

    def deadman_switch_callback(self, pressed: bool) -> None:
        self.deadman_switch_pressed = pressed

    def goal_reached_callback(self) -> None:
        self.goal_reached = True

    def send_command(self, command: Command) -> None:
        self.send_command_fn(command)

    def send_goal(self, goal_pose: Pose) -> None:
        self.goal_reached = False
        self.send_goal_fn(goal_pose)

    def empty_buffers(self):
        self.poses_buffer = []
        self.speeds_buffer = []
        self.accelerations_buffer = []

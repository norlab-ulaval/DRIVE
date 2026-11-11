# Robot Module

The robot module (`robot.py`) provides a simple interface to connect DRIVE to your robot's control, navigation and localization systems. This class must be configured for each robot platform you want to use with DRIVE. First, you need to provide how to send control commands and goal poses to your system. Then, you need to configure your system to call the necessary functions to update the robot's state.

## ROS2 implementation

Since localization, navigation and teleoperation is different from robot to robot, you will have a ROS node that creates a Robot class and properly listens and publishes to the topics of your system.

### Publishers

You will need to provide two functions variable to the robot class to handle goal and command sending to your system.

`self.send_command_fn`: When called, this function takes a linear and angular velocity and publishes the control command of your robot on a topic.

`self.send_goal_fn`: When called, this function takes a goal pose and publishes the goal to your robot's navigation system.

### Subscribers

Each of the following information will need to come from a topic of your system. Once you receive a message, you need to call the appropriate callback.

`pose_callback`: This should be called from your robot's localization topic.

`deadman_switch_callback`: This should be called from your robot's teleop topic.

`goal_reached_callback`: This should be called from your robot's navigation topic once it has reached it's current goal.

### Example

You can see an example in the `drive_ros_bridge.py` node.

import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String, Float64
# from warthog_msgs.msg import Status

import numpy as np


class DoughnutCalibratorNode(Node):
    """
    Class that sends out commands to calibrate mobile ground robots
    """
    # def __init__(self, max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
    #              step_len, dead_man_button, dead_man_index, dead_man_threshold, ramp_trigger_button, ramp_trigger_index,
    #              calib_trigger_button, calib_trigger_index, response_model_window, steady_state_std_dev_threshold,
    #              cmd_rate_param, encoder_rate_param):
    def __init__(self):
        super().__init__('doughnut_calib_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ramp_gain', 0.05),
                ('max_lin_speed', 2.0),
                ('min_lin_speed', 0.0),
                ('lin_speed_step', 0.5),
                ('max_ang_speed', 0.0),
                ('ang_steps', 0),
                ('step_len', 0.0),
                ('dead_man_button', True),
                ('dead_man_index', 0),
                ('dead_man_threshold', 0.5),
                ('ramp_trigger_button', False),
                ('ramp_trigger_index', 0),
                ('calib_trigger_button', False),
                ('calib_trigger_index', 0),
                ('response_model_window', 0),
                ('steady_state_std_dev_threshold', 0.5),
                ('cmd_rate_param', 20),
                ('encoder_rate_param', 4),
            ]
        )
        self.ramp_gain = self.get_parameter('ramp_gain').get_parameter_value().double_value
        self.max_lin_speed = self.get_parameter('max_lin_speed').get_parameter_value().double_value
        self.min_lin_speed = self.get_parameter('min_lin_speed').get_parameter_value().double_value
        self.lin_speed_step = self.get_parameter('lin_speed_step').get_parameter_value().double_value
        self.max_ang_speed = self.get_parameter('max_ang_speed').get_parameter_value().double_value
        self.n_ang_steps = self.get_parameter('ang_steps').get_parameter_value().integer_value
        self.step_len = self.get_parameter('step_len').get_parameter_value().double_value
        self.dead_man_button = self.get_parameter('dead_man_button').get_parameter_value().bool_value
        self.dead_man_index = self.get_parameter('dead_man_index').get_parameter_value().integer_value
        self.dead_man_threshold = self.get_parameter('dead_man_threshold').get_parameter_value().double_value
        self.ramp_trigger_button = self.get_parameter('ramp_trigger_button').get_parameter_value().bool_value
        self.ramp_trigger_index = self.get_parameter('ramp_trigger_index').get_parameter_value().integer_value
        self.calib_trigger_button = self.get_parameter('calib_trigger_button').get_parameter_value().bool_value
        self.calib_trigger_index = self.get_parameter('calib_trigger_index').get_parameter_value().integer_value
        # self.response_model_window = self.get_parameter('response_model_window').get_parameter_value().double_value
        # self.steady_state_std_dev_threshold = self.get_parameter('steady_state_std_dev_threshold').get_parameter_value().double_value()
        self.cmd_rate_param = self.get_parameter('cmd_rate_param').get_parameter_value().integer_value
        self.cmd_rate = self.create_rate(self.cmd_rate_param)
        self.encoder_rate = self.get_parameter('encoder_rate_param').get_parameter_value().integer_value

        self.n_lin_steps = int((self.max_lin_speed - self.min_lin_speed) / self.lin_speed_step) + 1
        self.ang_step = 2 * self.max_ang_speed / self.n_ang_steps

        self.full_vels_array = np.zeros((self.n_lin_steps, self.n_ang_steps+1, 2))
        angular_direction_positive = True
        for i in range(0, self.n_lin_steps):
            self.full_vels_array[i, :, 0] = self.min_lin_speed + i * self.lin_speed_step
            if angular_direction_positive:
                self.full_vels_array[i, 0, 1] = -self.max_ang_speed
                for j in range(1, self.n_ang_steps + 1):
                    self.full_vels_array[i, j, 1] = -self.max_ang_speed + j * self.ang_step
            else:
                self.full_vels_array[i, 0, 1] = self.max_ang_speed
                for j in range(1, self.n_ang_steps + 1):
                    self.full_vels_array[i, j, 1] = self.max_ang_speed - j * self.ang_step
            angular_direction_positive = not angular_direction_positive

        self.get_logger().info('\n' + np.array2string(self.full_vels_array[:, :, 0]))
        self.get_logger().info('\n' + np.array2string(self.full_vels_array[:, :, 1]))

        self.ang_inc = 0
        self.step_t = 0
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.calib_step_lin = 0
        self.calib_step_ang = 0
        self.calib_lin_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 0]
        self.calib_ang_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 1]
        # TODO: Use this if doing realtime steady-state check
        # self.left_wheel_vel_array = np.empty((int(self.response_model_window * self.encoder_rate), 3))
        # self.left_wheel_vel_array[:, :] = np.nan
        # self.right_wheel_vel_array = np.empty((int(self.response_model_window * self.encoder_rate), 3))
        # self.right_wheel_vel_array[:, :] = np.nan

        if self.min_lin_speed < 0:
            self.forward_bool = False
        else:
            self.forward_bool = True

        self.cmd_msg = Twist()
        self.joy_bool = Bool()
        self.good_calib_step = Bool()
        self.good_calib_step.data = False
        self.imu_msg = Imu()
        self.left_wheel_msg = Float64()
        self.right_wheel_msg = Float64()
        self.state_msg = String()
        self.state_msg.data = "idle"  # 4 possible states : idle, ramp_up, ramp_down, calib

        self.joy_listener = self.create_subscription(
            Joy,
            'joy_in',
            self.joy_callback,
            1000)
        self.imu_listener = self.create_subscription(
            Imu,
            'imu_in',
            self.imu_callback,
            1000)
        self.left_wheel_listener = self.create_subscription(
            Float64,
            'left_wheel_in',
            self.left_wheel_callback,
            1000)
        self.right_wheel_listener = self.create_subscription(
            Float64,
            'right_wheel_in',
            self.right_wheel_callback,
            1000)
        # self.keyboard_ramp_listener = self.create_subscription(
        #     String,
        #     'keyboard_ramp_control',
        #     self.keyboard_ramp_callback,
        #     1000)
        # self.keyboard_skip_listener = self.create_subscription(
        #     String,
        #     'keyboard_skip_control',
        #     self.keyboard_skip_callback,
        #     1000)
        # self.keyboard_prev_listener = self.create_subscription(
        #     String,
        #     'keyboard_prev_control',
        #     self.keyboard_prev_callback,
        #     1000)
        self.keyboard_listener = self.create_subscription(
            String,
            'glyphkey_pressed',
            self.keyboard_callback,
            1000)
        # self.estop_listener = self.create_subscription(
        #     Status,
        #     'mcu/status',
        #     self.estop_callback,
        #     1000)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.joy_pub = self.create_publisher(Bool, 'joy_switch', 10)
        self.good_calib_step_pub = self.create_publisher(Bool, 'good_calib_step', 10)
        self.state_pub = self.create_publisher(String, 'calib_state', 10)

        # self.create_rate(2).sleep()  # 2 seconds before init to allow proper boot

        self.dead_man = False
        self.ramp_trigger = False
        self.calib_trigger = False
        self.skip_calib_step_trigger = False
        self.prev_calib_step_trigger = False
        self.steady_state = False
        self.calibration_end = False
        self.first_order_calib = False
        self.step_skip_bool = False
        self.step_prev_bool = False
        # self.estop_bool = False

    def joy_callback(self, joy_data):
        global dead_man
        global dead_man_index
        if self.dead_man_button == False:
            if joy_data.axes[self.dead_man_index] >= self.dead_man_threshold \
                    and joy_data.axes[self.ramp_trigger_index] == 0 \
                    and joy_data.axes[self.calib_trigger_index] == 0 \
                    and joy_data.buttons[self.ramp_trigger_index] == 0 \
                    and joy_data.buttons[self.calib_trigger_index] == 0 :

                self.dead_man = True
            else:
                self.dead_man = False
        else:
            if joy_data.buttons[self.dead_man_index] >= self.dead_man_threshold \
                    and joy_data.buttons[self.ramp_trigger_index] == 0 \
                    and joy_data.buttons[self.calib_trigger_index] == 0 \
                    and joy_data.buttons[self.ramp_trigger_index] == 0 \
                    and joy_data.buttons[self.calib_trigger_index] == 0:

                self.dead_man = True
            else:
                self.dead_man = False

        if self.ramp_trigger_button == False:
            if joy_data.axes[self.ramp_trigger_index] <= -0.8:
                self.ramp_trigger = True
            else:
                self.ramp_trigger = False
        else:
            if joy_data.buttons[self.ramp_trigger_index] <= -0.8:
                self.ramp_trigger = True
            else:
                self.ramp_trigger = False

        if self.calib_trigger_button == False:
            if joy_data.axes[self.calib_trigger_index] >= 0.8:
                self.calib_trigger = True
            else:
                self.calib_trigger = False
        else:
            if joy_data.buttons[self.calib_trigger_index] >= 0.8:
                self.calib_trigger = True
            else:
                self.calib_trigger = False

    # def keyboard_ramp_callback(self, keyboard_ramp_msg):
    #     if keyboard_ramp_msg.data == "Up":
    #         self.calib_trigger = True
    #     else:
    #         self.calib_trigger = False
    #
    #     if keyboard_ramp_msg.data == "Down":
    #         self.ramp_trigger = True
    #     else:
    #         self.ramp_trigger = False
    #
    # def keyboard_skip_callback(self, keyboard_skip_msg):
    #     self.skip_calib_step_trigger = keyboard_skip_msg.data
    #
    # def keyboard_prev_callback(self, keyboard_prev_msg):
    #     self.prev_calib_step_trigger = keyboard_prev_msg.data

    def keyboard_callback(self, keyboard_msg):
        if keyboard_msg.data == "u":
            self.calib_trigger = True
        else:
            self.calib_trigger = False

        if keyboard_msg.data == "d":
            self.ramp_trigger = True
        else:
            self.ramp_trigger = False

        if keyboard_msg.data == "s":
            self.skip_calib_step_trigger = True
        else:
            self.skip_calib_step_trigger = False

        if keyboard_msg.data == "p":
            self.prev_calib_step_trigger = True
        else:
            self.prev_calib_step_trigger = False

    def imu_callback(self, imu_data):
        self.imu_msg = imu_data

    def left_wheel_callback(self, left_wheel_data):
        self.left_wheel_msg = left_wheel_data

    def right_wheel_callback(self, right_wheel_data):
        self.right_wheel_msg = right_wheel_data

    # def estop_callback(self, estop_data):
    #     self.estop_bool = estop_data.stop_engaged

    def powertrain_vel(self, cmd, last_vel, tau_c):
        return last_vel + (1 / tau_c) * (cmd - last_vel) * (1 / self.encoder_rate)

    def publish_cmd(self):
        self.cmd_vel_pub.publish(self.cmd_msg)
        self.cmd_rate.sleep()

    def publish_joy_switch(self):
        self.joy_pub.publish(self.joy_bool)

    def publish_state(self):
        self.state_pub.publish(self.state_msg)

    def ramp_up(self):
        """
        Function to ramp linear velocity up to current step
        :return:
        """
        # TODO: Ramp up for angular vel as well (4 cases necessary instead of 2)
        if self.calib_lin_speed < 0 and self.calib_ang_speed < 0:
            while self.lin_speed > self.calib_lin_speed or self.ang_speed > self.calib_ang_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                # if self.dead_man == False and self.estop_bool == False:
                if self.dead_man == False:
                    if self.lin_speed > self.calib_lin_speed:
                        self.lin_speed -= self.ramp_gain
                    if self.ang_speed > self.calib_ang_speed:
                        self.ang_speed -= self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
                    return False
            self.state_msg.data = "calib"

        if self.calib_lin_speed < 0 and self.calib_ang_speed >= 0:
            while self.lin_speed > self.calib_lin_speed or self.ang_speed < self.calib_ang_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed > self.calib_lin_speed:
                        self.lin_speed -= self.ramp_gain
                    if self.ang_speed < self.calib_ang_speed:
                        self.ang_speed += self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
                    return False
            self.state_msg.data = "calib"

        if self.calib_lin_speed >= 0 and self.calib_ang_speed < 0:
            while self.lin_speed < self.calib_lin_speed or self.ang_speed > self.calib_ang_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed < self.calib_lin_speed:
                        self.lin_speed += self.ramp_gain
                    if self.ang_speed > self.calib_ang_speed:
                        self.ang_speed -= self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
                    return False
            self.state_msg.data = "calib"

        if self.calib_lin_speed >= 0 and self.calib_ang_speed >= 0:
            while self.lin_speed < self.calib_lin_speed or self.ang_speed < self.calib_ang_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed < self.calib_lin_speed:
                        self.lin_speed += self.ramp_gain
                    if self.ang_speed < self.calib_ang_speed:
                        self.ang_speed += self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
                    return False
            self.state_msg.data = "calib"

        self.cmd_rate.sleep()
        return True

    def ramp_down(self):
        """
        Function to ramp linear velocity down to idle
        :return:
        """
        # TODO: Ramp down for angular vel as well (4 cases necessary instead of 2)
        if self.calib_lin_speed < 0 and self.calib_ang_speed < 0:
            while self.lin_speed < -0.1 or self.ang_speed < -0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed < -0.1:
                        self.lin_speed += self.ramp_gain
                    if self.ang_speed < -0.1:
                        self.ang_speed += self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed < 0 and self.calib_ang_speed >= 0:
            while self.lin_speed < -0.1 or self.ang_speed > 0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed < -0.1:
                        self.lin_speed += self.ramp_gain
                    if self.ang_speed > 0.1:
                        self.ang_speed -= self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed >= 0 and self.calib_ang_speed < 0:
            while self.lin_speed >= 0.1 or self.ang_speed < -0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed > 0.1:
                        self.lin_speed -= self.ramp_gain
                    if self.ang_speed < -0.1:
                        self.ang_speed += self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed >= 0 and self.calib_ang_speed >= 0:
            while self.lin_speed >= 0.1 or self.ang_speed >= 0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed > 0.1:
                        self.lin_speed -= self.ramp_gain
                    if self.ang_speed > 0.1:
                        self.ang_speed -= self.ramp_gain
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0.0
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        self.state_msg.data = "idle"

        self.cmd_rate.sleep()
        return True

    def calibrate_input_space(self):
        while self.calibration_end == False:
            self.get_logger().info(str(self.right_wheel_msg.data))
            self.linear_calib = True

            ## TODO: reverse-engineer cmd model

            ## TODO: find linear limits
            while self.linear_calib:
                self.lin_speed += self.ramp_gain
            self.publish_cmd()

            ## TODO: find angular limits

            ## TODO: generate calibration points

    def calibrate_kinematic(self):
        """
        Main doughnut calibration function, alternating between ramps and calibration steps
        :return:
        """
        while self.calibration_end == False:
            self.publish_state()

            if self.state_msg.data == "idle":
                if self.calib_trigger == True:
                    self.ramp_up()
                else:
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.publish_cmd()

            elif self.state_msg.data == "calib":

                if self.ramp_trigger == True:
                    self.step_t = 0
                    self.ramp_down()
                    continue

                if self.dead_man == False:
                    if self.calib_step_ang == self.n_ang_steps + 1:
                        self.calib_step_ang = 0
                        if self.calib_step_lin + 1 == self.n_lin_steps:
                            self.get_logger().info("Calibration complete. Ramping down.")
                            self.calibration_end = True
                            break
                        self.calib_step_lin += 1
                        self.calib_lin_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 0]
                        self.lin_speed = self.calib_lin_speed

                    self.calib_ang_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 1]
                    self.ang_speed = self.calib_ang_speed
                    self.cmd_msg.linear.x = self.calib_lin_speed
                    self.cmd_msg.angular.z = self.calib_ang_speed
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                    self.step_t += 0.05
                    if self.step_t >= self.step_len or self.skip_calib_step_trigger:
                        self.calib_step_ang += 1
                        self.good_calib_step.data = True
                        self.good_calib_step_pub.publish(self.good_calib_step)
                        self.good_calib_step.data = False
                        self.step_t = 0

                    # TODO: Fix previous step function
                    # if self.prev_calib_step_trigger:
                    #     if self.calib_ang_speed == 0:
                    #         self.calib_step_ang -= 1
                    #         self.step_t = 0
                    #     else:
                    #         self.calib_step_lin -= 1
                    #         self.calib_lin_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 0]
                    #         self.lin_speed = self.calib_lin_speed
                    #         self.step_t = 0

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")

                    self.lin_speed = 0.0
                    self.state_msg.data = "idle"
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
        self.ramp_down()
        self.calibration_end == False


def main(args=None):
    rclpy.init(args=args)
    calibrator_node = DoughnutCalibratorNode()
    thread = threading.Thread(target=rclpy.spin, args=(calibrator_node, ), daemon=True)
    thread.start()
    calibrator_node.calibrate_input_space()
    calibrator_node.calibrate_kinematic()
    rclpy.spin(calibrator_node)
    calibrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

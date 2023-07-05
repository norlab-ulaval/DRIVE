import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String, Float64, Int32
# from warthog_msgs.msg import Status

import os
import numpy as np
import pandas as pd

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
                ('max_lin_speed', 2.0),
                ('max_ang_speed', 2.0),
                ('step_len', 0.0),
                ('n_calib_steps', 20),
                ('dead_man_button', True),
                ('dead_man_index', 0),
                ('dead_man_threshold', 0.5),
                ('ramp_trigger_button', False),
                ('ramp_trigger_index', 0),
                ('calib_trigger_button', False),
                ('calib_trigger_button', False),
                ('calib_trigger_index', 0),
                ('cmd_rate_param', 20),
                ('encoder_rate_param', 4),
                ('load_input_space_calib_data', False),
                ('path_to_input_space_calib_data', '/home/robot/ros2_ws/src/doughnut_calib/calib_data'),
            ]
        )

        self.max_lin_speed = self.get_parameter('max_lin_speed').get_parameter_value().double_value
        self.max_ang_speed = self.get_parameter('max_ang_speed').get_parameter_value().double_value
        self.step_len = self.get_parameter('step_len').get_parameter_value().double_value
        self.n_calib_steps = self.get_parameter('n_calib_steps').get_parameter_value().integer_value
        self.dead_man_button = self.get_parameter('dead_man_button').get_parameter_value().bool_value
        self.dead_man_index = self.get_parameter('dead_man_index').get_parameter_value().integer_value
        self.dead_man_threshold = self.get_parameter('dead_man_threshold').get_parameter_value().double_value
        self.ramp_trigger_button = self.get_parameter('ramp_trigger_button').get_parameter_value().bool_value
        self.ramp_trigger_index = self.get_parameter('ramp_trigger_index').get_parameter_value().integer_value
        self.calib_trigger_button = self.get_parameter('calib_trigger_button').get_parameter_value().bool_value
        self.calib_trigger_index = self.get_parameter('calib_trigger_index').get_parameter_value().integer_value
        self.cmd_rate_param = self.get_parameter('cmd_rate_param').get_parameter_value().integer_value
        self.cmd_rate = self.create_rate(self.cmd_rate_param)
        self.encoder_rate = self.get_parameter('encoder_rate_param').get_parameter_value().integer_value
        self.load_input_space_calib_data = self.get_parameter('load_input_space_calib_data').get_parameter_value().bool_value
        self.path_to_input_space_calib_data = self.get_parameter('path_to_input_space_calib_data').get_parameter_value().string_value

        self.cmd_msg = Twist()
        self.joy_bool = Bool()
        self.good_calib_step = Bool()
        self.good_calib_step.data = False
        self.calib_step_msg = Int32()
        self.calib_step_msg.data = 0
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
        self.keyboard_listener = self.create_subscription(
            String,
            'glyphkey_pressed',
            self.keyboard_callback,
            1000)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.joy_pub = self.create_publisher(Bool, 'joy_switch', 10)
        self.good_calib_step_pub = self.create_publisher(Bool, 'good_calib_step', 10)
        self.calib_step_pub = self.create_publisher(Int32, 'calib_step', 10)
        self.state_pub = self.create_publisher(String, 'calib_state', 10)

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

    def powertrain_vel(self, cmd, last_vel, tau_c):
        return last_vel + (1 / tau_c) * (cmd - last_vel) * (1 / self.encoder_rate)

    def publish_cmd(self):
        self.cmd_vel_pub.publish(self.cmd_msg)
        self.cmd_rate.sleep()

    def publish_joy_switch(self):
        self.joy_pub.publish(self.joy_bool)

    def publish_state(self):
        self.state_pub.publish(self.state_msg)

    def reverse_engineer_command_model(self):
        left_encoder_vels_list = []
        right_encoder_vels_list = []
        command_linear_calibration = self.max_lin_speed / 2
        self.calib_lin_speed = command_linear_calibration
        self.calib_ang_speed = 0.0
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        linear_vel_time_window = self.step_len
        linear_vel_elapsed_time = 0.0
        previous_time = self.get_clock().now().nanoseconds * 1e-9
        while linear_vel_elapsed_time < linear_vel_time_window:
            self.cmd_msg.linear.x = command_linear_calibration
            self.cmd_msg.angular.z = 0.0
            self.publish_cmd()
            linear_vel_elapsed_time += (self.get_clock().now().nanoseconds * 1e-9 - previous_time)
            previous_time = self.get_clock().now().nanoseconds * 1e-9
            if linear_vel_elapsed_time >= self.step_len/3:
                left_encoder_vels_list.append(self.left_wheel_msg.data)
                right_encoder_vels_list.append(self.right_wheel_msg.data)
        # self.cmd_msg.linear.x = 0.0
        # self.publish_cmd()
        self.calib_lin_speed = command_linear_calibration
        self.calib_ang_speed = 0.0
        self.lin_speed = command_linear_calibration
        self.ang_speed = 0.0
        left_encoder_vels_array = np.array(left_encoder_vels_list)
        left_encoder_vels_mean = np.mean(left_encoder_vels_array)
        right_encoder_vels_array = np.array(right_encoder_vels_list)
        right_encoder_vels_mean = np.mean(right_encoder_vels_array)
        ## TODO: validate if wheel velocities are symmetrical
        self.calibrated_wheel_radius = command_linear_calibration / left_encoder_vels_mean
        self.get_logger().info("calibrated wheel radius :" + str(self.calibrated_wheel_radius))

        left_encoder_vels_list = []
        right_encoder_vels_list = []
        command_angular_calibration = self.max_ang_speed / 2
        self.calib_lin_speed = 0.0
        self.calib_ang_speed = command_angular_calibration
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        angular_vel_time_window = 2
        angular_vel_elapsed_time = 0.0
        previous_time = self.get_clock().now().nanoseconds * 1e-9
        while angular_vel_elapsed_time < angular_vel_time_window:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = command_angular_calibration
            self.publish_cmd()
            angular_vel_elapsed_time += (self.get_clock().now().nanoseconds * 1e-9 - previous_time)
            previous_time = self.get_clock().now().nanoseconds * 1e-9
            if angular_vel_elapsed_time >= 1.0:
                left_encoder_vels_list.append(self.left_wheel_msg.data)
                right_encoder_vels_list.append(self.right_wheel_msg.data)
        self.calib_lin_speed = 0.0
        self.calib_ang_speed = command_angular_calibration
        self.lin_speed = 0.0
        self.ang_speed = command_angular_calibration
        left_encoder_vels_array = np.array(left_encoder_vels_list)
        left_encoder_vels_mean = np.mean(left_encoder_vels_array)
        right_encoder_vels_array = np.array(right_encoder_vels_list)
        right_encoder_vels_mean = np.mean(right_encoder_vels_array)
        ## TODO: validate if wheel velocities are symmetrical
        self.calibrated_baseline = -2 * self.calibrated_wheel_radius * left_encoder_vels_mean / command_angular_calibration
        self.get_logger().info("calibrated baseline: " + str(self.calibrated_baseline))

        self.command_diff_drive_jacobian = self.calibrated_wheel_radius * np.array([[0.5, 0.5],
                                                                                    [-1/self.calibrated_baseline, 1/self.calibrated_baseline]])
        self.command_diff_drive_jacobian_inverse = np.linalg.inv(self.command_diff_drive_jacobian)
    def command_to_input_vector(self, command_linear_vel, command_angular_vel):
        command_vector = np.array([command_linear_vel, command_angular_vel])
        encoder_vels_vector = self.command_diff_drive_jacobian_inverse @ command_vector
        # reverse_body_command_vector = self.command_diff_drive_jacobian @ encoder_vels_vector
        # self.get_logger().info("reverse_compute_angular_vel :" + str(reverse_body_command_vector[1]))
        return encoder_vels_vector

    def input_to_command_vector(self, left_wheel_vel, right_wheel_vel):
        input_vector = np.array([left_wheel_vel, right_wheel_vel])
        body_vels_vector = self.command_diff_drive_jacobian @ input_vector
        # reverse_body_command_vector = self.command_diff_drive_jacobian @ encoder_vels_vector
        # self.get_logger().info("reverse_compute_angular_vel :" + str(reverse_body_command_vector[1]))
        return body_vels_vector

    def calibrate_maximum_linear_limits(self):
        encoders_saturated = False
        command_linear_maximum_limit = self.max_lin_speed * 10
        linear_vel_elapsed_time = 0.0
        previous_time = self.get_clock().now().nanoseconds * 1e-9
        max_vel_step = 0.25
        left_encoder_vels_sum = 0
        left_encoder_vels_num = 0
        left_encoder_vels_list = []
        right_encoder_vels_list = []
        while not linear_vel_elapsed_time >= 6.0:
            self.cmd_msg.linear.x = command_linear_maximum_limit
            self.cmd_msg.angular.z = 0.0
            self.publish_cmd()
            # self.get_logger().info("cmd_linear_x :" + str(self.cmd_msg.linear.x))
            # self.get_logger().info("left_encoder_measure :" + str(self.left_wheel_msg.data))
            self.encoder_command_vector = self.command_to_input_vector(self.cmd_msg.linear.x, self.cmd_msg.angular.z)
            # self.get_logger().info("left_encoder_command :" + str(self.encoder_command_vector[0]))
            linear_vel_elapsed_time += (self.get_clock().now().nanoseconds * 1e-9 - previous_time)
            previous_time = self.get_clock().now().nanoseconds * 1e-9
            if linear_vel_elapsed_time >= 2.0:
                left_encoder_vels_list.append(self.left_wheel_msg.data)
                right_encoder_vels_list.append(self.right_wheel_msg.data)
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.publish_cmd()

        self.maximum_wheel_vel = np.mean(np.array([np.abs(np.median(np.asarray(left_encoder_vels_list))),
                                                   np.abs(np.median(np.asarray(right_encoder_vels_list)))]))
        self.minimum_wheel_vel = -self.maximum_wheel_vel


    def calibrate_input_space(self):
        self.reverse_engineer_command_model()
        self.calibrate_maximum_linear_limits()
        self.get_logger().info(str(self.maximum_wheel_vel))
        self.get_logger().info(str(self.minimum_wheel_vel))
        # self.step_len = self.transitory_time_max * 3
        self.maximum_linear_vel_positive = self.input_to_command_vector(self.maximum_wheel_vel, self.maximum_wheel_vel)[1]
        self.maximum_linear_vel_negative = self.input_to_command_vector(-self.maximum_wheel_vel, -self.maximum_wheel_vel)[1]
        self.input_space_array = np.array([self.calibrated_wheel_radius,
                                           self.calibrated_baseline,
                                           self.maximum_wheel_vel,
                                           self.minimum_wheel_vel])
        cols = ['calibrated_radius [m]',
                'calibrated_baseline [m]',
                'maximum_wheel_vel_positive [rad/s]',
                'maximum_wheel_vel_negative [rad/s]',
                ]
        self.input_space_array_dataframe = pd.DataFrame(self.input_space_array.reshape((1, len(cols))), columns=cols)

        self.input_space_array_dataframe.to_pickle(self.path_to_input_space_calib_data)
        return None

    def uniform_calibration_input_space_sampling(self):
        wheel_vels = np.random.uniform(self.minimum_wheel_vel, self.maximum_wheel_vel, size=2)
        body_vels = self.input_to_command_vector(wheel_vels[0], wheel_vels[1])
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.calib_lin_speed = body_vels[0]
        self.calib_ang_speed = body_vels[1]
        self.step_t = 0
        while self.calibration_end == False:
            self.publish_state()
            self.calib_step_pub.publish(self.calib_step_msg)

            if self.state_msg.data == "idle":
                self.step_t = 0
                if self.calib_trigger == True:
                    # self.ramp_up()
                    self.state_msg.data = 'calib'
                else:
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.publish_cmd()

            elif self.state_msg.data == "calib":

                if self.ramp_trigger == True:
                    self.step_t = 0
                    continue

                if self.dead_man == False:
                    self.cmd_msg.linear.x = body_vels[0]
                    self.cmd_msg.angular.z = body_vels[1]
                    self.joy_bool.data = False
                    self.publish_cmd()
                    self.publish_joy_switch()

                    self.step_t += 0.05
                    if self.step_t >= self.step_len or self.skip_calib_step_trigger:
                        wheel_vels = np.random.uniform(self.minimum_wheel_vel, self.maximum_wheel_vel, size=2)
                        body_vels = self.input_to_command_vector(wheel_vels[0], wheel_vels[1])
                        self.good_calib_step.data = True
                        self.good_calib_step_pub.publish(self.good_calib_step)
                        self.good_calib_step.data = False
                        self.step_t = 0
                        self.calib_step_msg.data += 1

                else:
                    self.get_logger().info("Incoming command from controller, calibration suspended.")

                    self.lin_speed = 0.0
                    self.state_msg.data = "idle"
                    self.joy_switch = Bool()
                    self.joy_switch.data = True
                    self.publish_joy_switch()
        self.calibration_end = False
        self.state_msg.data = "idle"

    def run_calibration(self):
        if self.load_input_space_calib_data:
            self.input_space_array_dataframe = pd.read_pickle(self.path_to_input_space_calib_data)
            self.calibrated_wheel_radius = self.input_space_array_dataframe['calibrated_radius [m]'].to_numpy()[0]
            self.calibrated_baseline = self.input_space_array_dataframe['calibrated baseline [m]'].to_numpy()[0]
            self.maximum_wheel_vel = self.input_space_array_dataframe['maximum_wheel_vel_positive [rad/s]']
            self.minimum_wheel_vel = self.input_space_array_dataframe['minimum_wheel_vel_positive [rad/s]']
            self.command_diff_drive_jacobian = self.calibrated_wheel_radius * np.array([[0.5, 0.5],
                                                                                        [-1 / self.calibrated_baseline,
                                                                                         1 / self.calibrated_baseline]])
            self.command_diff_drive_jacobian_inverse = np.linalg.inv(self.command_diff_drive_jacobian)
        else:
            self.calibrate_input_space()

        self.uniform_calibration_input_space_sampling()

def main(args=None):
    rclpy.init(args=args)
    calibrator_node = DoughnutCalibratorNode()
    thread = threading.Thread(target=rclpy.spin, args=(calibrator_node, ), daemon=True)
    thread.start()
    calibrator_node.run_calibration()
    # rclpy.spin(calibrator_node)
    calibrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

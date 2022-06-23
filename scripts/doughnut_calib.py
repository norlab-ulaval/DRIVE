#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String, Float64

import numpy as np
import scipy.signal as sig
import scipy.optimize as opti
import scipy.special as sp

class DoughnutCalibrator:
    """
    Class that sends out commands to calibrate mobile ground robots
    """
    def __init__(self, max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
                 step_len, dead_man_index, dead_man_threshold, ramp_trigger_index, calib_trigger_index,
                 response_model_window, steady_state_std_dev_threshold, cmd_rate_param, encoder_rate_param):

        self.max_lin_speed = max_lin_speed
        self.min_lin_speed = min_lin_speed
        self.lin_speed_step = lin_speed_step
        self.max_ang_speed = max_ang_speed
        self.ang_steps = ang_steps
        self.step_len = step_len
        self.dead_man_index = dead_man_index
        self.dead_man_threshold = dead_man_threshold
        self.ramp_trigger_index = ramp_trigger_index
        self.calib_trigger_index = calib_trigger_index
        self.response_model_window = response_model_window
        self.steady_state_std_dev_threshold = steady_state_std_dev_threshold
        self.cmd_rate = rospy.Rate(cmd_rate_param)
        self.encoder_rate = encoder_rate_param

        self.ang_inc = 0
        self.step_t = 0
        self.lin_speed = 0
        self.calib_lin_speed = self.min_lin_speed
        self.left_wheel_vel_array = np.empty((int(self.response_model_window * encoder_rate_param), 3))
        self.left_wheel_vel_array[:, :] = np.nan
        self.right_wheel_vel_array = np.empty((int(self.response_model_window * encoder_rate_param), 3))
        self.right_wheel_vel_array[:, :] = np.nan

        if self.min_lin_speed < 0:
            self.forward_bool = False
        else:
            self.forward_bool = True

        self.cmd_msg = Twist()
        self.joy_bool = Bool()
        self.imu_msg = Imu()
        self.left_wheel_msg = Float64()
        self.right_wheel_msg = Float64()
        self.state_msg = String()
        self.state_msg.data = "idle"  # 4 possible states : idle, ramp_up, ramp_down, calib

        self.joy_listener = rospy.Subscriber("joy_in", Joy, self.joy_callback)
        self.imu_listener = rospy.Subscriber("imu_in", Imu, self.imu_callback)
        self.left_wheel_listener = rospy.Subscriber("left_wheel_in", Float64, self.left_wheel_callback)
        self.right_wheel_listener = rospy.Subscriber("right_wheel_in", Float64, self.right_wheel_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        self.joy_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher('calib_state', String, queue_size=10)

        rospy.sleep(2)  # 10 seconds before init to allow proper boot

        self.dead_man = False
        self.ramp_trigger = False
        self.calib_trigger = False
        self.steady_state = False
        self.calibration_end = False
        self.first_order_calib = False

    def joy_callback(self, joy_data):
        global dead_man
        global dead_man_index
        if joy_data.axes[self.dead_man_index] >= self.dead_man_threshold \
                and joy_data.axes[self.ramp_trigger_index] == 0 \
                and joy_data.axes[self.calib_trigger_index] == 0 :

            self.dead_man = True
        else:
            self.dead_man = False

        if joy_data.axes[self.ramp_trigger_index] <= -0.8:
            self.ramp_trigger = True
        else:
            self.ramp_trigger = False

        if joy_data.axes[self.calib_trigger_index] >= 0.8:
            self.calib_trigger = True
        else:
            self.calib_trigger = False

    def imu_callback(self, imu_data):
        self.imu_msg = imu_data

    def left_wheel_callback(self, left_wheel_data):
        self.left_wheel_msg = left_wheel_data

    def right_wheel_callback(self, right_wheel_data):
        self.right_wheel_msg = right_wheel_data

    # def steady_state_test(self):
    #     # TODO: compute std_dev of window of measures in the past, stop if below threshold
    #     self.measure_array = np.roll(self.measure_array, 1)
    #     self.measure_array[0] = self.imu_msg.angular_velocity.z
    #
    #     if np.std(self.measure_array) < self.steady_state_std_dev_threshold:
    #         self.steady_state = True
    #
    #     return None

    def powertrain_vel(self, cmd, last_vel, tau_c):
        return last_vel + (1 / tau_c) * (cmd - last_vel) * (1 / self.encoder_rate)

    def tune_first_order_system(self):
        """
        Function that models a first order step response
        :return:
        """
        # TODO: Take both encoders into account
        left_measure_index = 1
        right_measure_index = 1
        self.left_wheel_vel_array[0, 0], self.right_wheel_vel_array[0.0] = rospy.get_time()
        self.left_wheel_vel_array[0, 1] = self.left_wheel_msg.data
        self.right_wheel_vel_array[0, 1] = self.right_wheel_msg.data
        # print(self.wheel_vel_array.shape[0])
        while self.first_order_calib == True:
            print(self.left_wheel_msg.data)
            print(self.left_wheel_vel_array[left_measure_index - 1, 0])
            if self.left_wheel_msg.data != self.left_wheel_vel_array[left_measure_index - 1, 0] :
                # TODO: Manage the case when encoder values come in faster than command.
                self.left_wheel_vel_array[left_measure_index, 0] = rospy.get_time()
                self.left_wheel_vel_array[left_measure_index, 1] = self.left_wheel_msg.data
                print(left_measure_index)

                if left_measure_index == self.left_wheel_vel_array.shape[0]-1:
                    self.first_order_calib = False

                left_measure_index = left_measure_index + 1

            # if self.right_wheel_msg.data != self.right_wheel_vel_array[right_measure_index - 1, 0] :
            #     # TODO: Manage the case when encoder values come in faster than command.
            #     self.right_wheel_vel_array[right_measure_index, 0] = rospy.get_time()
            #     self.right_wheel_vel_array[right_measure_index, 1] = self.right_wheel_msg.data
            #     print(right_measure_index)
            #
            #     if right_measure_index == self.right_wheel_vel_array.shape[0]-1:
            #         self.first_order_calib = False
            #
            #     right_measure_index = right_measure_index + 1

        self.left_wheel_vel_array[:, :] = np.nan

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
        if self.calib_lin_speed < 0:
            while self.lin_speed > self.calib_lin_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                if self.dead_man == False:
                    self.lin_speed = self.lin_speed - 0.1
                    ang_speed = 0.0
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
            self.state_msg.data = "calib"

        if self.calib_lin_speed >= 0:
            while self.lin_speed < self.calib_lin_speed:
                self.state_msg.data = "ramp_up"
                self.publish_state()
                if self.dead_man == False:
                    self.lin_speed = self.lin_speed + 0.1
                    ang_speed = 0.0
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.state_msg.data = "idle"
            self.state_msg.data = "calib"

            self.cmd_rate.sleep()
            return True

    def ramp_down(self):
        """
        Function to ramp linear velocity down to idle
        :return:
        """
        if self.calib_lin_speed < 0:
            while self.lin_speed < -0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    self.lin_speed = self.lin_speed + 0.1
                    ang_speed = 0.0
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
            self.robot_state = "idle"

        if self.calib_lin_speed >= 0:
            while self.lin_speed > 0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    self.lin_speed = self.lin_speed - 0.1
                    ang_speed = 0.0
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
            self.state_msg.data = "idle"

            self.cmd_rate.sleep()
            return True

    def calibrate(self):
        """
        Main doughnut calibration function, alternating between ramps and calibration steps
        :return:
        """
        # TODO: define conditions for various steps
        # while np.abs(self.max_lin_speed - self.calib_lin_speed) > 0 and self.ang_inc < self.ang_steps:
        while self.calibration_end == False:
            self.publish_state()

            if self.state_msg.data == "idle":
                if self.calib_trigger == True:
                    self.ramp_up()
                else:
                    self.cmd_msg.linear.x = 0
                    self.cmd_msg.angular.z = 0
                    self.publish_cmd()
            elif self.state_msg.data == "calib":
                if self.ramp_trigger == True:
                    self.ramp_down()

                if self.dead_man == False:
                    if self.ang_inc == self.ang_steps:
                        self.ang_inc = 0
                        self.calib_lin_speed = self.calib_lin_speed + self.lin_speed_step
                        self.lin_speed = self.calib_lin_speed

                    #ang_speed = max_ang_speed * np.sin(ang_inc * 2 * np.pi / ang_steps)
                    self.ang_speed = (self.max_ang_speed * 2 / np.pi) * np.arcsin(np.sin(2 * np.pi * self.ang_inc / self.ang_steps))
                    self.cmd_msg.linear.x = self.calib_lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                    # self.steady_state_test()
                    # TODO: Define steady-state condition here
                    # if self.steady_state == True:
                    #     self.ang_inc = self.ang_inc + 1
                    #     self.steady_state = False

                    self.tune_first_order_system()

                    self.step_t += 0.05
                    if self.step_t >= self.step_len:
                        self.ang_inc = self.ang_inc + 1
                        self.step_t = 0
                        self.first_order_calib = True

                    if np.abs(self.max_lin_speed - self.calib_lin_speed) > 0 and self.ang_inc < self.ang_steps:
                        self.calibration_end == True

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.state_msg.data = "idle"
                    joy_switch = Bool(True)
                    self.publish_joy_switch()

if __name__ == '__main__':
    try:
        rospy.init_node('doughnut_calib', anonymous=True)
        max_lin_speed = rospy.get_param('/doughnut_calib/max_lin_speed', 2.0)
        min_lin_speed = rospy.get_param('/doughnut_calib/min_lin_speed', 0.0)
        lin_speed_step = rospy.get_param('/doughnut_calib/lin_speed_step', 0.0)
        max_ang_speed = rospy.get_param('/doughnut_calib/max_ang_speed', 0.0)
        ang_steps = rospy.get_param('/doughnut_calib/ang_steps', 0.0)
        step_len = rospy.get_param('/doughnut_calib/step_len', 0.0)
        dead_man_index = rospy.get_param('/doughnut_calib/dead_man_index', 0)
        dead_man_threshold = rospy.get_param('/doughnut_calib/dead_man_threshold', 0)
        ramp_trigger_index = rospy.get_param('/doughnut_calib/ramp_trigger_index', 0)
        calib_trigger_index = rospy.get_param('/doughnut_calib/calib_trigger_index', 0)
        steady_state_window = rospy.get_param('/doughnut_calib/steady_state_window', 0)
        steady_state_std_dev_threshold = rospy.get_param('/doughnut_calib/steady_state_std_dev_threshold', 0)
        cmd_rate_param = rospy.get_param('/doughnut_calib/cmd_rate', 20)
        encoder_rate_param = rospy.get_param('/doughnut_calib/encoder_rate', 4)
        calibrator = DoughnutCalibrator(max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
                                        step_len, dead_man_index, dead_man_threshold, ramp_trigger_index,
                                        calib_trigger_index, steady_state_window, steady_state_std_dev_threshold,
                                        cmd_rate_param, encoder_rate_param)

        calibrator.calibrate()
    except rospy.ROSInterruptException:
        pass

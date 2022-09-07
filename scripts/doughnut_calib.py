#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String, Float64

import numpy as np
import scipy.signal as sig
import scipy.optimize as opti
import scipy.special as sp

import threading
import sys
import termios, tty
from select import select

class DoughnutCalibrator:
    """
    Class that sends out commands to calibrate mobile ground robots
    """
    def __init__(self, max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
                 step_len, dead_man_button, dead_man_index, dead_man_threshold, ramp_trigger_button, ramp_trigger_index,
                 calib_trigger_button, calib_trigger_index, response_model_window, steady_state_std_dev_threshold,
                 cmd_rate_param, encoder_rate_param):

        self.max_lin_speed = max_lin_speed
        self.min_lin_speed = min_lin_speed
        self.lin_speed_step = lin_speed_step
        self.max_ang_speed = max_ang_speed
        self.n_ang_steps = ang_steps

        self.n_lin_steps = int(max_lin_speed - min_lin_speed / self.lin_speed_step) + 1
        self.ang_step = 2 * self.max_ang_speed / ang_steps

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

        rospy.loginfo(self.full_vels_array[:, :, 0])
        rospy.loginfo(self.full_vels_array[:, :, 1])

        self.step_len = step_len
        self.dead_man_button = dead_man_button
        self.dead_man_index = dead_man_index
        self.dead_man_threshold = dead_man_threshold
        self.ramp_trigger_button = ramp_trigger_button
        self.ramp_trigger_index = ramp_trigger_index
        self.calib_trigger_button = calib_trigger_button
        self.calib_trigger_index = calib_trigger_index
        self.response_model_window = response_model_window
        self.steady_state_std_dev_threshold = steady_state_std_dev_threshold
        self.cmd_rate = rospy.Rate(cmd_rate_param)
        self.encoder_rate = encoder_rate_param

        self.ang_inc = 0
        self.step_t = 0
        self.lin_speed = 0
        self.ang_speed = 0
        self.calib_step_lin = 0
        self.calib_step_ang = 0
        self.calib_lin_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 0]
        self.calib_ang_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 1]
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
        self.keyboard_ramp_listener = rospy.Subscriber("keyboard_ramp_control", String, self.keyboard_ramp_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        self.joy_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)
        self.good_calib_step_pub = rospy.Publisher('good_calib_step', Bool, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher('calib_state', String, queue_size=10)

        rospy.sleep(2)  # 10 seconds before init to allow proper boot

        self.dead_man = False
        self.ramp_trigger = False
        self.calib_trigger = False
        self.skip_calib_step_trigger = False
        self.steady_state = False
        self.calibration_end = False
        self.first_order_calib = False
        self.good_calib_step = False

        # # Code from https://code.activestate.com/recipes/572182-how-to-implement-kbhit-on-linux/
        # # save the terminal settings
        # fd = sys.stdin.fileno()
        # new_term = termios.tcgetattr(fd)
        # old_term = termios.tcgetattr(fd)
        # # new terminal setting unbuffered
        # new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        # atexit.register(termios.tcsetattr(fd, termios.TCSAFLUSH, old_term))
        # termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

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

    def keyboard_ramp_callback(self, keyboard_ramp_msg):
        if keyboard_ramp_msg.data == "Up":
            self.calib_trigger = True
        else:
            self.calib_trigger = False

        if keyboard_ramp_msg.data == "Down":
            self.ramp_trigger = True
        else:
            self.ramp_trigger = False

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
                if self.dead_man == False:
                    if self.lin_speed > self.calib_lin_speed:
                        self.lin_speed -= 0.1
                    if self.ang_speed > self.calib_ang_speed:
                        self.ang_speed -= 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
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
                        self.lin_speed -= 0.1
                    if self.ang_speed < self.calib_ang_speed:
                        self.ang_speed += 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
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
                        self.lin_speed += 0.1
                    if self.ang_speed > self.calib_ang_speed:
                        self.ang_speed -= 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
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
                        self.lin_speed += 0.1
                    if self.ang_speed < self.calib_ang_speed:
                        self.ang_speed += 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
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
                        self.lin_speed += 0.1
                    if self.ang_speed < -0.1:
                        self.ang_speed += 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed < 0 and self.calib_ang_speed >= 0:
            while self.lin_speed < -0.1 or self.ang_speed > 0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed < -0.1:
                        self.lin_speed += 0.1
                    if self.ang_speed > 0.1:
                        self.ang_speed -= 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed >= 0 and self.calib_ang_speed < 0:
            while self.lin_speed >= 0.1 or self.ang_speed < -0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed > 0.1:
                        self.lin_speed -= 0.1
                    if self.ang_speed < -0.1:
                        self.ang_speed += 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        if self.calib_lin_speed >= 0 and self.calib_ang_speed >= 0:
            while self.lin_speed >= 0.1 or self.ang_speed >= 0.1:
                self.state_msg.data = "ramp_down"
                self.publish_state()
                if self.dead_man == False:
                    if self.lin_speed > 0.1:
                        self.lin_speed -= 0.1
                    if self.ang_speed > 0.1:
                        self.ang_speed -= 0.1
                    self.cmd_msg.linear.x = self.lin_speed
                    self.cmd_msg.angular.z = self.ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.joy_switch = Bool(True)
                    self.publish_joy_switch()
                    self.robot_state = "idle"
                    return False

        self.state_msg.data = "idle"

        self.cmd_rate.sleep()
        return True

    def calibrate(self):
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
                    self.cmd_msg.linear.x = 0
                    self.cmd_msg.angular.z = 0
                    self.publish_cmd()

            elif self.state_msg.data == "calib":

                if self.ramp_trigger == True:
                    self.step_t = 0
                    self.ramp_down()
                    continue

                if self.dead_man == False:
                    if self.calib_step_ang == self.n_ang_steps + 1:
                        self.calib_step_ang = 0
                        if self.calib_step_lin + 2 == self.n_lin_steps:
                            rospy.loginfo("Calibration complete. Ramping down.")
                            self.calibration_end = True
                            break
                        self.calib_step_lin += 1
                        self.calib_lin_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 0]
                        self.lin_speed = self.calib_lin_speed

                    self.calib_ang_speed = self.full_vels_array[self.calib_step_lin, self.calib_step_ang, 1]
                    self.ang_speed = self.calib_ang_speed
                    self.cmd_msg.linear.x = self.calib_lin_speed
                    self.cmd_msg.angular.z = self.calib_ang_speed
                    joy_switch = Bool(False)
                    self.publish_cmd()
                    self.publish_joy_switch()

                    self.step_t += 0.05
                    if self.step_t >= self.step_len:
                        self.calib_step_ang += 1
                        self.good_calib_step = True
                        self.good_calib_step_pub.publish(self.good_calib_step)
                        self.good_calib_step = False
                        self.step_t = 0

                else:
                    rospy.loginfo("Incoming command from controller, calibration suspended.")
                    self.lin_speed = 0
                    self.state_msg.data = "idle"
                    joy_switch = Bool(True)
                    self.publish_joy_switch()
        self.ramp_down()


if __name__ == '__main__':
    try:
        rospy.init_node('doughnut_calib', anonymous=True)
        max_lin_speed = rospy.get_param('/doughnut_calib/max_lin_speed', 2.0)
        min_lin_speed = rospy.get_param('/doughnut_calib/min_lin_speed', 0.0)
        lin_speed_step = rospy.get_param('/doughnut_calib/lin_speed_step', 0.0)
        max_ang_speed = rospy.get_param('/doughnut_calib/max_ang_speed', 0.0)
        ang_steps = rospy.get_param('/doughnut_calib/ang_steps', 0.0)
        step_len = rospy.get_param('/doughnut_calib/step_len', 0.0)
        dead_man_button = rospy.get_param('/doughnut_calib/dead_man_button', True)
        dead_man_index = rospy.get_param('/doughnut_calib/dead_man_index', 0)
        dead_man_threshold = rospy.get_param('/doughnut_calib/dead_man_threshold', 0)
        ramp_trigger_button = rospy.get_param('/doughnut_calib/ramp_trigger_button', False)
        ramp_trigger_index = rospy.get_param('/doughnut_calib/ramp_trigger_index', 0)
        calib_trigger_button = rospy.get_param('/doughnut_calib/calib_trigger_button', False)
        calib_trigger_index = rospy.get_param('/doughnut_calib/calib_trigger_index', 0)
        steady_state_window = rospy.get_param('/doughnut_calib/steady_state_window', 0)
        steady_state_std_dev_threshold = rospy.get_param('/doughnut_calib/steady_state_std_dev_threshold', 0)
        cmd_rate_param = rospy.get_param('/doughnut_calib/cmd_rate', 20)
        encoder_rate_param = rospy.get_param('/doughnut_calib/encoder_rate', 4)
        calibrator = DoughnutCalibrator(max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
                                        step_len, dead_man_button, dead_man_index, dead_man_threshold, ramp_trigger_button,
                                        ramp_trigger_index, calib_trigger_button, calib_trigger_index, steady_state_window,
                                        steady_state_std_dev_threshold, cmd_rate_param, encoder_rate_param)

        calibrator.calibrate()
    except rospy.ROSInterruptException:
        pass

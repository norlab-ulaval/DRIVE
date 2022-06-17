#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool

import numpy as np

dead_man = 0
dead_man_index = 0
max_lin_speed = 0
min_lin_speed = 0
lin_step = 0
max_ang_speed = 0
ang_steps = 0
step_len = 0
dead_man_index = 0



# def cmd_vel_pub():
#     global dead_man
#     global dead_man_index
#     global max_lin_speed
#     global min_lin_speed
#     global lin_step
#     global max_ang_speed
#     global ang_steps
#     global step_len
#     global dead_man_index
#     max_lin_speed = rospy.get_param('/odom_calib_cmd/max_lin_speed', 0.0)
#     min_lin_speed = rospy.get_param('/odom_calib_cmd/min_lin_speed', 0.0)
#     lin_step = rospy.get_param('/odom_calib_cmd/lin_step', 0.0)
#     max_ang_speed = rospy.get_param('/odom_calib_cmd/max_ang_speed', 0.0)
#     ang_steps = rospy.get_param('/odom_calib_cmd/ang_steps', 0.0)
#     step_len = rospy.get_param('/odom_calib_cmd/step_len', 0.0)
#     dead_man_index = rospy.get_param('/odom_calib_cmd/dead_man_index', 0.0)
#
#     ang_inc = 0
#     step_t = 0
#     lin_speed = 0
#
#     rospy.Subscriber("joy_in", Joy, callback)
#
#     pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
#     joy_switch_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)
#     rate = rospy.Rate(20) # 20hz
#     cmd_msg = Twist()
#     joy_switch = Bool()
#     rospy.sleep(10) #10 seconds before init to allow proper boot
#
#     # ramp up
#     while lin_speed > min_lin_speed + 0.1:
#         if dead_man > -750:
#             lin_speed = lin_speed - 0.1
#             ang_speed = 0.0
#             cmd_msg.linear.x = lin_speed
#             cmd_msg.angular.z = ang_speed
#             joy_switch = Bool(False)
#             pub.publish(cmd_msg)
#             joy_switch_pub.publish(joy_switch)
#
#         else:
#             rospy.loginfo("Incoming command from controller, calibration suspended.")
#             joy_switch = Bool(True)
#             joy_switch_pub.publish(joy_switch)
#
#         rate.sleep()
#
#     # calibration
#     while lin_speed <= max_lin_speed:
#         if dead_man > -750:
#             if ang_inc == ang_steps:
#                 ang_inc = 0
#                 lin_speed = lin_speed + lin_step
#
#             #ang_speed = max_ang_speed * np.sin(ang_inc * 2 * np.pi / ang_steps)
#             ang_speed = (max_ang_speed * 2 / np.pi) * np.arcsin(np.sin(2 * np.pi * ang_inc / ang_steps))
#             cmd_msg.linear.x = lin_speed
#             cmd_msg.angular.z = ang_speed
#             joy_switch = Bool(False)
#             pub.publish(cmd_msg)
#             joy_switch_pub.publish(joy_switch)
#             step_t += 0.05
#             if step_t >= step_len:
#                 ang_inc = ang_inc + 1
#                 step_t = 0
#
#         else:
#             rospy.loginfo("Incoming command from controller, calibration suspended.")
#             joy_switch = Bool(True)
#             joy_switch_pub.publish(joy_switch)
#
#         rate.sleep()
#
#         # ramp down
#     while lin_speed > 0:
#         if dead_man > -750:
#             lin_speed = lin_speed - 0.1
#             ang_speed = 0.0
#             cmd_msg.linear.x = lin_speed
#             cmd_msg.angular.z = ang_speed
#             joy_switch = Bool(False)
#             pub.publish(cmd_msg)
#             joy_switch_pub.publish(joy_switch)
#
#         else:
#             rospy.loginfo("Incoming command from controller, calibration suspended.")
#             joy_switch = Bool(True)
#             joy_switch_pub.publish(joy_switch)
#
#         rate.sleep()

def calib_switch_on():
    switch = Bool(True)
    switch_pub = rospy.Publisher('calib_switch', Bool, queue_size=10, latch=True)
    switch_pub.publish(switch)

def calib_switch_off():
    switch = Bool(False)
    switch_pub = rospy.Publisher('calib_switch', Bool, queue_size=10, latch=True)
    switch_pub.publish(switch)

class DoughnutCalibrator:
    """
    Class that sends out commands to calibrate mobile ground robots
    """
    def __int__(self):
        self.max_lin_speed = rospy.get_param('/dougnhut_calib/max_lin_speed', 0.0)
        self.min_lin_speed = rospy.get_param('/dougnhut_calib/min_lin_speed', 0.0)
        self.lin_step = rospy.get_param('/dougnhut_calib/lin_step', 0.0)
        self.max_ang_speed = rospy.get_param('/dougnhut_calib/max_ang_speed', 0.0)
        self.ang_steps = rospy.get_param('/dougnhut_calib/ang_steps', 0.0)
        self.step_len = rospy.get_param('/dougnhut_calib/step_len', 0.0)
        self.dead_man_index = rospy.get_param('/dougnhut_calib/dead_man_index', 0)
        self.ramp_down_index = rospy.get_param('/dougnhut_calib/ramp_down_index', 0)
        rate_param = rospy.get_param('/dougnhut_calib/rate', 20)
        self.rate = rospy.Rate(rate_param)

        self.ang_inc = 0
        self.step_t = 0
        self.lin_speed = 0

        if self.min_lin_speed < 0:
            self.forward_bool = False
        else:
            self.forward_bool = True

        self.cmd_msg = Twist()
        self.joy_bool = Bool()
        self.imu_msg = Imu()

        self.joy_listener = rospy.Subscriber("joy_in", Joy, self.joy_callback())
        self.imu_listener = rospy.Subscriber("imu_in", Imu, self.imu_callback())

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        self.joy_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)

        pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        joy_switch_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)
        rate = rospy.Rate(20)  # 20hz
        cmd_msg = Twist()
        joy_switch = Bool()
        rospy.sleep(10)  # 10 seconds before init to allow proper boot

    def joy_callback(self, joy_data):
        global dead_man
        global dead_man_index
        self.dead_man = joy_data.axes[dead_man_index]
        # TODO : Add user ramp-down button, with self.ramp_down_index attribute

    def imu_callback(self, imu_data):
        self.imu_msg = imu_data

    def publish_cmd(self):
        self.cmd_vel_pub.publish(self.cmd_msg)

    def publish_joy_switch(self):
        self.joy_pub.publish(self.joy_bool)

    def ramp_up(self):
        # TODO: if statement depending on direction of robot
        # TODO: Combine ramp for both linear ang angular vels
        while self.lin_speed > self.min_lin_speed + 0.1:
            if dead_man > -750:
                self.lin_speed = self.lin_speed - 0.1
                ang_speed = 0.0
                self.cmd_msg.linear.x = self.lin_speed
                self.cmd_msg.angular.z = ang_speed
                joy_switch = Bool(False)
                self.publish_cmd()
                self.publish_joy_switch()

            else:
                rospy.loginfo("Incoming command from controller, calibration suspended.")
                self.joy_switch = Bool(True)
                self.publish_joy_switch()

            self.rate.sleep()

    def ramp_down(self):
        # TODO: Code opposite of ramp_up

    def calibrate(self):
        # TODO: define conditions for various steps

if __name__ == '__main__':
    try:
        rospy.init_node('doughnut_calib', anonymous=True)
        calib_switch_on()
        cmd_vel_pub()
        calib_switch_off()
    except rospy.ROSInterruptException:
        pass

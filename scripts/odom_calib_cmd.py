#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import numpy as np

dead_man = 0
dead_man_index = 0

def callback(data):
    global dead_man
    global dead_man_index
    dead_man = data.axes[dead_man_index]

def cmd_vel_pub():
    global dead_man
    global dead_man_index
    max_lin_speed = rospy.get_param('/odom_calib_cmd/max_lin_speed', 0.0)
    min_lin_speed = rospy.get_param('/odom_calib_cmd/min_lin_speed', 0.0)
    lin_step = rospy.get_param('/odom_calib_cmd/lin_step', 0.0)
    max_ang_speed = rospy.get_param('/odom_calib_cmd/max_ang_speed', 0.0)
    ang_steps = rospy.get_param('/odom_calib_cmd/ang_steps', 0.0)
    step_len = rospy.get_param('/odom_calib_cmd/step_len', 0.0)
    dead_man_index = rospy.get_param('/odom_calib_cmd/dead_man_index', 0.0)

    ang_inc = 0
    step_t = 0
    lin_speed = 0

    rospy.Subscriber("joy_in", Joy, callback)

    pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
    joy_switch_pub = rospy.Publisher('joy_switch', Bool, queue_size=10, latch=True)
    rate = rospy.Rate(20) # 20hz
    cmd_msg = Twist()
    joy_switch = Bool()
    rospy.sleep(10) #10 seconds before init to allow proper boot

    # ramp up
    while lin_speed > min_lin_speed + 0.1:
        if dead_man > -750:
            lin_speed = lin_speed - 0.1
            ang_speed = 0.0
            cmd_msg.linear.x = lin_speed
            cmd_msg.angular.z = ang_speed
            joy_switch = Bool(False)
            pub.publish(cmd_msg)
            joy_switch_pub.publish(joy_switch)

        else:
            rospy.loginfo("Incoming command from controller, calibration suspended.")
            joy_switch = Bool(True)
            joy_switch_pub.publish(joy_switch)

        rate.sleep()

    # calibration
    while lin_speed <= max_lin_speed:
        if dead_man > -750:
            if ang_inc == ang_steps:
                ang_inc = 0
                lin_speed = lin_speed + lin_step

            #ang_speed = max_ang_speed * np.sin(ang_inc * 2 * np.pi / ang_steps)
            ang_speed = (max_ang_speed * 2 / np.pi) * np.arcsin(np.sin(2 * np.pi * ang_inc / ang_steps))
            cmd_msg.linear.x = lin_speed
            cmd_msg.angular.z = ang_speed
            joy_switch = Bool(False)
            pub.publish(cmd_msg)
            joy_switch_pub.publish(joy_switch)
            step_t += 0.05
            if step_t >= step_len:
                ang_inc = ang_inc + 1
                step_t = 0

        else:
            rospy.loginfo("Incoming command from controller, calibration suspended.")
            joy_switch = Bool(True)
            joy_switch_pub.publish(joy_switch)

        rate.sleep()

        # ramp down
    while lin_speed > 0:
        if dead_man > -750:
            lin_speed = lin_speed - 0.1
            ang_speed = 0.0
            cmd_msg.linear.x = lin_speed
            cmd_msg.angular.z = ang_speed
            joy_switch = Bool(False)
            pub.publish(cmd_msg)
            joy_switch_pub.publish(joy_switch)

        else:
            rospy.loginfo("Incoming command from controller, calibration suspended.")
            joy_switch = Bool(True)
            joy_switch_pub.publish(joy_switch)

        rate.sleep()

def calib_switch_on():
    switch = Bool(True)
    switch_pub = rospy.Publisher('calib_switch', Bool, queue_size=10, latch=True)
    switch_pub.publish(switch)

def calib_switch_off():
    switch = Bool(False)
    switch_pub = rospy.Publisher('calib_switch', Bool, queue_size=10, latch=True)
    switch_pub.publish(switch)

if __name__ == '__main__':
    try:
        rospy.init_node('odom_calib_cmd', anonymous=True)
        calib_switch_on()
        cmd_vel_pub()
        calib_switch_off()
    except rospy.ROSInterruptException:
        pass

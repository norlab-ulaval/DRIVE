#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import numpy as np

dead_man = 0
dead_man_index = 0

def callback(data):
    global dead#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

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
    lin_step = rospy.get_param('/odom_calib_cmd/lin_step', 0.0)
    max_ang_speed = rospy.get_param('/odom_calib_cmd/max_ang_speed', 0.0)
    ang_steps = rospy.get_param('/odom_calib_cmd/ang_steps', 0.0)
    dead_man_index = rospy.get_param('/odom_calib_cmd/dead_man_index', 0.0)

    lin_speed = 0
    ang_inc = 0

    rospy.Subscriber("joy_in", Joy, callback)

    pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
    rospy.init_node('odom_calib_cmd', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    cmd_msg = Twist()
    while lin_speed < max_lin_speed:
        if dead_man > -750:
            if ang_inc == ang_steps:
                ang_inc = 0
                lin_speed = lin_speed + lin_step

            #ang_speed = max_ang_speed * np.sin(ang_inc * 2 * np.pi / ang_steps)
            ang_speed = (max_ang_speed * 2 / np.pi) * np.arcsin(np.sin(2 * np.pi * ang_inc / ang_steps))
            cmd_msg.linear.x = lin_speed
            cmd_msg.angular.z = ang_speed
            pub.publish(cmd_msg)
            ang_inc = ang_inc + 1

        else:
            rospy.loginfo("Incoming command from controller, calibration suspended.")

        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_pub()
    except rospy.ROSInterruptException:
        pass_man
    global dead_man_index
    dead_man = data.axes[dead_man_index]

def cmd_vel_pub():
    global dead_man
    global dead_man_index
    max_lin_speed = rospy.get_param('/odom_calib_cmd/max_lin_speed', 0.0)
    lin_step = rospy.get_param('/odom_calib_cmd/lin_step', 0.0)
    max_ang_speed = rospy.get_param('/odom_calib_cmd/max_ang_speed', 0.0)
    ang_steps = rospy.get_param('/odom_calib_cmd/ang_steps', 0.0)
    dead_man_index = rospy.get_param('/odom_calib_cmd/dead_man_index', 0.0)

    lin_speed = 0
    ang_inc = 0

    rospy.Subscriber("joy_in", Joy, callback)

    pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
    rospy.init_node('odom_calib_cmd', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    cmd_msg = Twist()
    while lin_speed < max_lin_speed:
        if dead_man > -750:
            if ang_inc == ang_steps:
                ang_inc = 0
                lin_speed = lin_speed + lin_step

            ang_speed = max_ang_speed * np.sin(ang_inc * 2 * np.pi / ang_steps)
            cmd_msg.linear.x = lin_speed
            cmd_msg.angular.z = ang_speed
            pub.publish(cmd_msg)
            ang_inc = ang_inc + 1

        else:
            rospy.loginfo("Incoming command from controller, calibration suspended.")

        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_pub()
    except rospy.ROSInterruptException:
        pass
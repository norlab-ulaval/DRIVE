#!/usr/bin/env python

from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import String, Bool

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


RampMsg = String
SkipMsg = Bool
PrevMsg = Bool

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.ramp_publisher = rospy.Publisher('keyboard_ramp_control', RampMsg, queue_size = 1)
        self.skip_publisher = rospy.Publisher('keyboard_skip_control', SkipMsg, queue_size = 1)
        self.prev_publisher = rospy.Publisher('keyboard_prev_control', PrevMsg, queue_size = 1)
        self.ramp_str = "None"
        self.skip_bool = False
        self.prev_bool = False
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, ramp_str, skip_bool, prev_bool):
        self.condition.acquire()
        self.ramp_str = ramp_str
        self.skip_bool = skip_bool
        self.prev_bool = prev_bool
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update("None", False)
        self.join()

    def run(self):
        ramp_msg = RampMsg()
        skip_msg = SkipMsg()
        prev_msg = PrevMsg()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            ramp_msg.data = self.ramp_str
            skip_msg.data = self.skip_bool
            prev_msg.data = self.prev_bool

            self.condition.release()

            # Publish.
            self.ramp_publisher.publish(ramp_msg)
            self.skip_publisher.publish(skip_msg)
            self.prev_publisher.publish(prev_msg)

        # Publish stop message when thread exits.
        ramp_str = "None"
        skip_bool = False
        prev_bool = False
        self.ramp_publisher.publish(ramp_str)
        self.skip_publisher.publish(skip_bool)
        self.prev_publisher.publish(prev_bool)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('keyboard_str_publisher')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.05)

    pub_thread = PublishThread(repeat)

    ramp_str = "None"
    skip_bool = False
    prev_bool = False

    try:
        pub_thread.update(ramp_str, skip_bool, prev_bool)
        while(1):
            key = getKey(settings, key_timeout)
            if key == 'd':
                ramp_str = 'Down'
            elif key == 'u':
                ramp_str = 'Up'
            elif key == 's':
                skip_bool = True
            elif key == 'p':
                prev_bool = True
            else:
                ramp_str = 'None'
                skip_bool = False
                prev_bool = False

            pub_thread.update(ramp_str, skip_bool, prev_bool)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

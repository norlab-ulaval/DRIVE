#!/usr/bin/env python

from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# class PublishThreadNode(threading.Thread, Node):
class PublishThreadNode(Node):
    def __init__(self, rate):
        super(PublishThreadNode, self).__init__(node_name='doughnut_keyboard_node')
        self.ramp_publisher = self.create_publisher(String, 'keyboard_ramp_control', 1)
        self.skip_publisher = self.create_publisher(Bool, 'keyboard_skip_control', 1)
        self.prev_publisher = self.create_publisher(Bool, 'keyboard_prev_control', 1)
        self.ramp_str = "None"
        self.skip_bool = False
        self.prev_bool = False
        self.condition = threading.Condition()
        self.done = False

        self.thread = threading.Thread(self.update())

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        print('before start')
        self.start()
        print('after start')

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
        ramp_msg = String()
        skip_msg = Bool()
        prev_msg = Bool()

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
        self.ramp_str = "None"
        self.skip_bool = False
        self.prev_bool = False
        self.ramp_publisher.publish(ramp_msg)
        self.skip_publisher.publish(skip_msg)
        self.prev_publisher.publish(prev_msg)


    def getKey(self, settings, timeout):
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

    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)

    def restoreTerminalSettings(self, old_settings):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):
    rclpy.init(args=args)
    print('start')
    repeat = 0.0
    key_timeout = 0.05
    publish_thread_node = PublishThreadNode(repeat)
    rclpy.spin(publish_thread_node)

    settings = publish_thread_node.saveTerminalSettings()

    ramp_str = 'None'
    skip_bool = False
    prev_bool = False

    try:
        publish_thread_node.update(ramp_str, skip_bool, prev_bool)
        while (1):
            key = publish_thread_node.getKey(settings, key_timeout)
            print('while')
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

            publish_thread_node.update(ramp_str, skip_bool, prev_bool)

    except Exception as e:
        print(e)

    finally:
        publish_thread_node.stop()
        publish_thread_node.restoreTerminalSettings(settings)

    publish_thread_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()






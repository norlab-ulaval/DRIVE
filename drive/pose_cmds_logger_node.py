#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import threading

from std_msgs.msg import Float64, Bool, String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from norlab_controllers_msgs.srv import ExportData
from std_srvs.srv import Empty
import message_filters

import numpy as np
import pandas as pd

from multiprocessing import Lock

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')
        self.declare_parameters(
            namespace='',
        parameters=[
                ('record_wheel_current',False),
                ('record_wheel_voltage',False),
            ]
        )

        self.record_wheel_current = self.get_parameter('record_wheel_current').get_parameter_value().bool_value
        self.record_wheel_voltage = self.get_parameter('record_wheel_voltage').get_parameter_value().bool_value

        #self.calib_sub = self.create_subscription(
        #    Odometry,
        #    'calib_switch',
        #    self.switch_callback,
        #    10)
        self.calib_step_sub = self.create_subscription(
            Int32,
            'calib_step',
            self.calib_step_callback,
            10)
        self.joy_sub = self.create_subscription(
            Bool,
            'drive/joy_switch',
            self.joy_callback,
            10)
        # self.estop_sub = self.create_subscription(
        #     Odometry,
        #     'mcu/status',
        #     self.estop_callback,
        #     10)
        self.calib_state_sub = self.create_subscription(
            String,
            'calib_state',
            self.calib_state_callback,
            10)
        self.icp_sub = self.create_subscription(
            Odometry,
            # 'icp_odom',
            'odometry_in',
            self.pose_callback,
            10)
        self.encoder_left_sub = self.create_subscription(
            Float64,
            # '/left_drive/status/speed',
            'wheel_vel_left_measured',
            self.velocity_left_meas_callback,
            10)
        self.encoder_right_sub = self.create_subscription(
            Float64,
            # '/right_drive/status/speed',
            'wheel_vel_right_measured',
            self.velocity_right_meas_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            # 'MTI_imu/data_unbiased',
            'imu_in',
            self.imu_callback,
            10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'doughnut_cmd_vel',
            self.cmd_vel_callback,
            10)

        #self.is_wheel_current_measured = self.create_subscription(
        #    Bool,
        #    'is_wheel_current_measured',
        #    self.is_wheel_current_measured_callback, 
        #    10)
        #
        #self.is_wheel_voltage_measured = self.create_subscription(
        #    Bool,
        #    'is_wheel_voltage_measured',
        #    self.is_wheel_voltage_measured_callback,
        #    10)
        
        
        #self.record_wheel_current = Bool()
        #self.record_wheel_voltage = Bool()

        

        self.calib_switch = Bool()
        self.joy_switch = Bool()
        self.pose = Odometry()
        self.velocity_left_cmd = Float64()
        self.velocity_left_meas = Float64()
        self.volt_left = Float64()
        self.current_left = Float64()
        self.velocity_right_cmd = Float64()
        self.velocity_right_meas = Float64()
        self.volt_right = Float64()
        self.current_right = Float64()
        self.imu_vel = Imu()
        self.cmd_vel = Twist()
        self.calib_state = String()
        self.calib_step = Int32()

        self.left_wheel_current_msg = Float64()
        self.right_wheel_current_msg = Float64()

        self.left_wheel_voltage_msg = Float64()
        self.right_wheel_voltage_msg = Float64()


        
        self.rate = self.create_rate(20, self.get_clock())
        self.save_service = self.create_service(ExportData, 'export_data', self.save_data_callback)

        self.array = np.zeros((1, 22))
        if self.record_wheel_current:
            self.array = np.hstack((self.array,np.zeros((1,2))))
        if self.record_wheel_voltage:
            self.array = np.hstack((self.array,np.zeros((1,2))))

        self.odom_index = 0
        self.prev_icp_x = 0
        self.prev_icp_y = 0
        self.icp_index = 0
        self.kill_node_trigger = False

        # self.set_parameter('use_sim_time', True)
        if self.record_wheel_current:
            self.right_wheel_current_listener = self.create_subscription(
            Float64,
            'right_wheel_current_in',
            self.right_wheel_current_callback,
            10)

            self.left_wheel_current_listener = self.create_subscription(
            Float64,
            'left_wheel_current_in',
            self.left_wheel_current_callback,
            10)
            
        if self.record_wheel_voltage:
            self.right_wheel_voltage_listener = self.create_subscription(
            Float64,
            'right_wheel_voltage_in',
            self.right_wheel_voltage_callback,
            10)
            
            self.left_wheel_voltage_listener = self.create_subscription(
            Float64,
            'left_wheel_voltage_in',
            self.left_wheel_voltage_callback,
            10)
            

    #def is_wheel_current_measured_callback(self,msg):
    #    self.record_wheel_current = msg
    #    
    #def is_wheel_voltage_measured_callback(self,msg):
    #    self.is_wheel_voltage_measured = msg
    
    def right_wheel_current_callback(self, right_wheel_current_data):
        self.right_wheel_current_msg = right_wheel_current_data

    def left_wheel_current_callback(self, left_wheel_current_data):
        self.left_wheel_current_msg = left_wheel_current_data
    
    def right_wheel_voltage_callback(self, right_wheel_voltage_data):
        self.right_wheel_voltage_msg = right_wheel_voltage_data

    def left_wheel_voltage_callback(self, left_wheel_voltage_data):
        self.left_wheel_voltage_msg = left_wheel_voltage_data

    
    def switch_callback(self, msg):
        self.calib_switch = msg

    def calib_step_callback(self, msg):
        self.calib_step = msg

    def joy_callback(self, msg):
        self.joy_switch = msg
    def calib_state_callback(self, msg):
        self.calib_state = msg

    def pose_callback(self, msg):
        self.pose = msg

    def velocity_left_meas_callback(self, msg):
        self.velocity_left_meas = msg

    def velocity_right_meas_callback(self, msg):
        self.velocity_right_meas = msg

    def imu_callback(self, msg):
        self.imu_vel = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        ## TODO: Find a better way to run the self.log_msgs() function when spinning

    def log_msgs(self):
        # Create numpy array with adequate poses
        if (self.pose.pose.pose.position.x != self.prev_icp_x
                and self.pose.pose.pose.position.y != self.prev_icp_y):
            self.prev_icp_x = self.pose.pose.pose.position.x
            self.prev_icp_y = self.pose.pose.pose.position.y
            self.icp_index += 1

        current_time_nanoseconds = int(self.get_clock().now().nanoseconds)
        ## DEBUG
        # self.get_logger().info(str(self.imu_vel.linear_acceleration.x))
        # self.get_logger().info(str(self.imu_vel.linear_acceleration.y))

        ## TODO: Fix clock call
        new_row = np.array(([current_time_nanoseconds, self.joy_switch.data, self.icp_index, self.calib_state.data, self.calib_step.data,
                             self.velocity_left_meas.data, self.velocity_right_meas.data,
                             self.cmd_vel.linear.x, self.cmd_vel.angular.z,
                             self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.pose.pose.pose.position.z,
                             self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                             self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w,
                             self.imu_vel.angular_velocity.x, self.imu_vel.angular_velocity.y,
                             self.imu_vel.angular_velocity.z, self.imu_vel.linear_acceleration.x,
                             self.imu_vel.linear_acceleration.y, self.imu_vel.linear_acceleration.z]))

        if self.record_wheel_voltage:
            new_row = np.hstack((new_row,np.array([self.left_wheel_voltage_msg.data,self.right_wheel_voltage_msg.data])))
        if self.record_wheel_current:
            new_row = np.hstack((new_row,np.array([self.left_wheel_current_msg.data,self.right_wheel_current_msg.data])))

        self.array = np.vstack((self.array, new_row))

# TODO: Add /mcu/status/stop_engaged listener

    def save_data_callback(self, req, res):
        self.get_logger().info('Converting Array to DataFrame')
        basic_column = ['ros_time', 'joy_switch', 'icp_index', 'calib_state', 'calib_step',
                                                   'meas_left_vel', 'meas_right_vel',
                                                   'cmd_vel_x', 'cmd_vel_omega',
                                                   'icp_pos_x', 'icp_pos_y', 'icp_pos_z',
                                                   'icp_quat_x', 'icp_quat_y',
                                                   'icp_quat_z', 'icp_quat_w',
                                                   'imu_x', 'imu_y', 'imu_z',
                                                    'imu_acceleration_x', 'imu_acceleration_y', 'imu_acceleration_z']
        if self.record_wheel_voltage:
            basic_column += ["left_wheel_voltage","right_wheel_voltage"]
        if self.record_wheel_current:
            basic_column += ["left_wheel_current","right_wheel_current"]
            
        df = pd.DataFrame(data=self.array, columns=basic_column)
        self.get_logger().info('Exporting DataFrame as .pkl')
        df.to_pickle(req.export_path.data)
        self.get_logger().info('Data export done!')
        self.kill_node_trigger = True
        return res

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    try:
        # declare the node constructor
        logger_node = LoggerNode()
        executor = SingleThreadedExecutor()
        executor.add_node(logger_node)

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(logger_node,), daemon=True)
        thread.start()

        try:
            while rclpy.ok() and not logger_node.kill_node_trigger:
                # executor.spin_once()
                logger_node.rate.sleep()
                logger_node.log_msgs()

        finally:
            # executor.shutdown()
            logger_node.destroy_node()
    finally:
        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
    main()

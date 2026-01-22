import numpy as np
from drive.model_training.models.powertrain.bounded_powertrain import *
from drive.model_training.models.kinematic.ideal_diff_drive import *

class Ideal_diff_drive_bounded:
    
    def __init__(self, r, baseline, dt,min_wheel_vel=-10.0, max_wheel_vel=10.0, time_constant=0.5, time_delay=0.1):
        
        self.idd = Ideal_diff_drive(r, baseline, dt)
        self.powertrain = Bounded_powertrain(min_wheel_vel, max_wheel_vel, time_constant, time_delay, dt)
        self.dt = dt

    def predict(self, init_state, input, current_wheel_vel):
        """
        :param init_state: initial state array [x, y, z, roll, pitch, yaw]
        :param input: input array [omega_l, omega_r]
        :return: next_state
        """
        adjusted_wheel_speed = self.powertrain.compute_bounded_wheel_vels(input, current_wheel_vel, self.dt)
        new_state = self.idd.predict(init_state, adjusted_wheel_speed)
        return new_state, adjusted_wheel_speed
    
    def predict_2d(self, init_state, input, current_wheel_vel):
        """
        :param init_state: initial state array [x, y, z, roll, pitch, yaw]
        :param input: input array [omega_l, omega_r]
        :return: next_state
        """
        adjusted_wheel_speed = self.powertrain.compute_bounded_wheel_vels(input, current_wheel_vel, self.dt)
        new_state = self.idd.predict_2d(init_state, adjusted_wheel_speed)
        return new_state

    


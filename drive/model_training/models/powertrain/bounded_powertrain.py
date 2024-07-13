import numpy as np
from drive.util.transform_algebra import *

class Bounded_powertrain:
    def __init__(self, min_vel, max_vel, time_constant, time_delay, dt):
        self.min_vel = min_vel
        self.max_vel = max_vel

        self.time_constant = time_constant
        self.time_delay = time_delay
        self.dt = dt

    def compute_bounded_wheel_vels(self, cmd_wheel_vel, prev_wheel_vel, cmd_elapsed_time):
        if cmd_elapsed_time <= self.time_delay:
            cmd_wheel_vel = prev_wheel_vel
        transitory_wheel_vel = prev_wheel_vel + (1 / self.time_constant) * (cmd_wheel_vel - prev_wheel_vel) * self.dt
        bounded_wheel_vel = np.clip(transitory_wheel_vel, self.min_vel, self.max_vel)
        return bounded_wheel_vel

    def update_params(self, new_params):
        self.time_constant = new_params[0]
        self.time_delay = new_params[1]
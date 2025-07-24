import shapely 

from dataclasses import dataclass

import numpy as np 
import matplotlib.pyplot as plt
import pathlib 
#from ros.drive_ros.calibration_node_utils import DriveRosBridgeParams
import datetime
from ros.drive_ros.calibration_node import DriveRosBridgeParams



def compute_sampling_space(maximum_wheel_speed: float, params: DriveRosBridgeParams):


    
    jacobian = params.wheel_radius * np.array([[1/2, 1/2],
                        [-1/(params.base_width), 1/(params.base_width)]])
    

    body_frame_constraints = np.array([[-params.max_linear_speed, params.max_angular_speed],
                                    [params.max_linear_speed, params.max_angular_speed],
                                    [params.max_linear_speed, -params.max_angular_speed],
                                    [-params.max_linear_speed, -params.max_angular_speed]])
    
    wheel_constraints = np.array([[maximum_wheel_speed,maximum_wheel_speed,-maximum_wheel_speed,-maximum_wheel_speed],
                                  [-maximum_wheel_speed,maximum_wheel_speed,maximum_wheel_speed,-maximum_wheel_speed]])
    
    body_frame_wheel_constraints = jacobian @ wheel_constraints
    print(body_frame_constraints)
    polygon_bf_constraints = shapely.Polygon(body_frame_constraints)
    print(body_frame_wheel_constraints, jacobian)
    pol_wheel_bf_constraints = shapely.Polygon(body_frame_wheel_constraints.T)
    
    sampling_space = shapely.intersection(pol_wheel_bf_constraints,polygon_bf_constraints)

    list_polygon = [pol_wheel_bf_constraints, polygon_bf_constraints, sampling_space]
    labels = ["Wheel speed based", "Body max speed", "Intersection"]
    ylabel = "Linear command [m/s]"
    xlabel = "Angular speed [rad/s]"

    fig,axs = plt.subplots(1,1)

    for pol, label in zip(list_polygon,labels):
        print(pol.exterior.xy[0])
        axs.plot(pol.exterior.xy[1],pol.exterior.xy[0], label = label)

    axs.legend()
    axs.set_xlabel(xlabel)
    axs.set_ylabel(ylabel)
    axs.set_aspect("equal")
    axs.set_title("Sampling based obtained from combining \n max wheel speed, max lin speed, max ang speed.")
    fig.savefig(pathlib.Path(params.datasets_directory)/"sampling_space.png")
    
    return sampling_space





if __name__ =="__main__":

    max_linear_speed = 3.0
    wheel_radius = 0.3
    max_wheel_speed = max_linear_speed/wheel_radius

    data = DriveRosBridgeParams(max_linear_speed=max_linear_speed, max_angular_speed=5.0,
                                base_width=1.08, wheel_radius=wheel_radius,
                                datasets_directory= "/home/nicolassamson/workspaces/DRIVE/ros/drive_ros")
    
     
    compute_sampling_space(max_wheel_speed,data)
    plt.show()
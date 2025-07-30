import shapely

from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt
import pathlib

# from ros.drive_ros.calibration_node_utils import DriveRosBridgeParams
import datetime
from drive_ros.drive_ros_bridge import DriveRosBridgeParams

import subprocess
import yaml

@dataclass
class SamplingSpaces:
    wheel_polygons: shapely.Polygon = shapely.Polygon()
    body_polygons: shapely.Polygon = shapely.Polygon()
    sampling_space: shapely.Polygon = shapely.Polygon()



def compute_sampling_space(maximum_wheel_speed: float, params: DriveRosBridgeParams):

    jacobian = params.wheel_radius * np.array([[1 / 2, 1 / 2], [-1 / (params.base_width), 1 / (params.base_width)]])

    body_frame_constraints = np.array(
        [
            [-params.max_linear_speed, params.max_angular_speed],
            [params.max_linear_speed, params.max_angular_speed],
            [params.max_linear_speed, -params.max_angular_speed],
            [-params.max_linear_speed, -params.max_angular_speed],
        ]
    )

    wheel_constraints = np.array(
        [
            [maximum_wheel_speed, maximum_wheel_speed, -maximum_wheel_speed, -maximum_wheel_speed],
            [-maximum_wheel_speed, maximum_wheel_speed, maximum_wheel_speed, -maximum_wheel_speed],
        ]
    )

    body_frame_wheel_constraints = jacobian @ wheel_constraints
    print(body_frame_constraints)
    polygon_bf_constraints = shapely.Polygon(body_frame_constraints)
    print(body_frame_wheel_constraints, jacobian)
    pol_wheel_bf_constraints = shapely.Polygon(body_frame_wheel_constraints.T)

    sampling_space = shapely.intersection(pol_wheel_bf_constraints, polygon_bf_constraints)

    
    dico_polygons = {"wheel_polygons": pol_wheel_bf_constraints,
                     "body_polygons": polygon_bf_constraints,
                     "sampling_space": sampling_space}  
    
    labels = ["Wheel speed based", "Body max speed", "Intersection"]
    ylabel = "Linear command [m/s]"
    xlabel = "Angular speed [rad/s]"

    fig, axs = plt.subplots(1, 1)

    for key, label in zip(dico_polygons.keys(), labels):
        pol = dico_polygons[key]
        print(pol.exterior.xy[0])
        axs.plot(pol.exterior.xy[1], pol.exterior.xy[0], label=key)

    axs.legend()
    axs.set_xlabel(xlabel)
    axs.set_ylabel(ylabel)
    axs.set_aspect("equal")
    axs.set_title("Sampling based obtained from combining \n max wheel speed, max lin speed, max ang speed.")
    folder_path = pathlib.Path(params.datasets_directory) / params.dataset_name
    if not folder_path.exists():
        pathlib.Path(folder_path).mkdir(parents=True, exist_ok=True)

    path_to_save = folder_path / "sampling_space.png"
    fig.savefig(path_to_save)
    print(f"Sampling space saved to {pathlib.Path(params.datasets_directory)/'sampling_space.png'}")

    result = subprocess.run(["echo", str(path_to_save)])

    return dico_polygons


def forward_kin(wheel_speeds: np.ndarray, params: DriveRosBridgeParams):

    jacobian = params.wheel_radius * np.array([[1 / 2, 1 / 2], [-1 / (params.base_width), 1 / (params.base_width)]])

    body_speed = jacobian @ wheel_speeds

    return body_speed


def inverse_kin(body_speed: np.ndarray, params: DriveRosBridgeParams):

    jacobian = params.wheel_radius * np.array([[1 / 2, 1 / 2], [-1 / (params.base_width), 1 / (params.base_width)]])

    wheel_speeds = np.linalg.pinv(jacobian) @ body_speed

    return wheel_speeds


def save_sampling_space(list_polygon, path_config_file: pathlib.Path):

    
    dico = {}
    
    for key,polygon in list_polygon.items():
        if not isinstance(polygon, shapely.Polygon):
            raise ValueError("Expected a list of shapely.Polygon objects.")
        # Extract coordinates from the exterior ring (without closing point)
        
        x = polygon.exterior.xy[0]
        y = polygon.exterior.xy[1]
        xy_array = np.vstack((x, y)).T
        
        # Convert to list of [x, y] (no z assumed)
        coord_list = [[float(xy_array[i,0]), float(xy_array[i,1])] for i in range(xy_array.shape[0])]
        
        
        # [pol_wheel_bf_constraints, polygon_bf_constraints, sampling_space]
        dico[key] = coord_list
    

    with open(path_config_file, "w") as file:
        
        yaml.safe_dump(dico, file)

def load_sampling_spaces(path_config: pathlib.Path):
    

    with open(path_config, "r") as file:
        sampling_spaces = yaml.safe_load(file)
    dico_polygons = {}
    for key, coords in sampling_spaces.items():
        dico_polygons[key] = shapely.Polygon(coords)

    return dico_polygons

if __name__ == "__main__":

    max_linear_speed = 3.0
    wheel_radius = 0.3
    max_wheel_speed = max_linear_speed / wheel_radius

    data = DriveRosBridgeParams(
        max_linear_speed=max_linear_speed,
        max_angular_speed=5.0,
        base_width=1.08,
        wheel_radius=wheel_radius,
        datasets_directory="/home/ws/ros/drive_ros",
    )

    list_polygon = compute_sampling_space(max_wheel_speed, data)

    path = pathlib.Path("/home/ws/ros/config/drive_ros_bridge.yaml")
    save_sampling_space(list_polygon, path)

    dico = load_sampling_spaces(path_config=path)
    
    print(dico["body_polygons"])
    
    plt.show()

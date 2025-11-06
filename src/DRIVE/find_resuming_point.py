import numpy as np
import matplotlib.pyplot as plt
from DRIVE.common import Pose, Command
from DRIVE.robot import Robot
from DRIVE.geofencing import Geofence

from scipy.spatial import transform
from shapely.geometry import Polygon




def compute_horizon_pose(cmd: Command, time_horizon: float, delta_time: float, current_pose: Pose) -> list[np.ndarray]:
    """
    Compute the predicted pose of the robot after a given time horizon based on the current pose and command.

    Args:
        cmd (Command): The command containing linear and angular velocities.
        time_horizon (float): The time horizon in seconds.
        current_pose (Pose): The current pose of the robot.

    Returns:
        Pose: The predicted pose after the time horizon.
    """
    x, y, z, roll, pitch, yaw = current_pose
    v = cmd[0]
    omega = cmd[1]

    # Current pose
    rot_matrix = transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = rot_matrix
    transform_matrix[0:3, 3] = [x, y, z]

    # COmpute cmd transform
    delta_yaw = omega * delta_time
    delta_lin = v * delta_time

    rot_matrix_cmd = transform.Rotation.from_euler("z", delta_yaw).as_matrix()
    transform_matrix_cmd = np.eye(4)
    transform_matrix_cmd[0:3, 0:3] = rot_matrix_cmd
    transform_matrix_cmd[0:3, 3] = [delta_lin, 0, 0]

    list_pose = [current_pose]

    for i in range(int(time_horizon / delta_time)):
        transform_matrix = transform_matrix @ transform_matrix_cmd

        list_pose.append(
            np.array(
                [
                    transform_matrix[0, 3],
                    transform_matrix[1, 3],
                    transform_matrix[2, 3],
                    *transform.Rotation.from_matrix(transform_matrix[0:3, 0:3]).as_euler("xyz"),
                ]
            )
        )

    return list_pose


def plot_poses(list_pose, color, label, ax):

    scale = 0.5

    for i, robot_pose in enumerate(list_pose):
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        orientation_x = robot_x + np.cos(robot_pose[5])  # yaw
        orientation_y = robot_y + np.sin(robot_pose[5])

        if i == 0:
            ax.quiver(
                robot_x,
                robot_y,
                orientation_x - robot_x,
                orientation_y - robot_y,
                color=color,
                angles="xy",
                scale_units="xy",
                scale=scale,
                label=label,
            )
            ax.scatter(robot_x, robot_y, color="black", marker="X")

        else:
            ax.quiver(
                robot_x,
                robot_y,
                orientation_x - robot_x,
                orientation_y - robot_y,
                color=color,
                angles="xy",
                scale_units="xy",
                scale=scale,
            )


if __name__ == "__main__":

    # Create a dummy geofence
    coordinates = [(-5, -3), (-5, 3), (5, 3), (5, -3)]
    origin = (0, 0)
    geofence = Geofence(coordinates, origin=origin)

    radius = 3.0 
    circle = np.linspace(0, 2 * np.pi, 100)
    x_circle = geofence.origin[0] + radius* np.cos(circle)
    y_circle = geofence.origin[1] + radius * np.sin(circle)
    
    # Create the robot
    robot_pose = np.array([0.0, 5.0, 0.0, 0.0, 0.0, 1.575])  # x, y, z, roll, pitch, yaw


    # Command used

    cmd = np.array([1.0, 0.3])
    time_cmd_is_maintained = 3.0
    time_step = 0.4

    list_pose = compute_horizon_pose(cmd, time_cmd_is_maintained, time_step, robot_pose)

    # Plotting
    fig, ax = plt.subplots(1, 1)

    # Plot geofence
    geofence_x, geofence_y = zip(*coordinates + [coordinates[0]])  # Close the polygon
    ax.fill(geofence_x, geofence_y, color="blue", alpha=0.5, label="Geofence")

    plot_poses(list_pose, "orange", "Predicted Poses", ax)
    plot_poses([robot_pose], "red", "current pose", ax)
    ax.scatter(geofence.origin[0], geofence.origin[1], label="Geofence Center")


    #ax.plot(x_circle, y_circle, color="green", linestyle="--", label="Possible pose ?")
    # Plot robot pose

    ax.axhline(0, color="black", linewidth=0.5, ls="--")
    ax.axvline(0, color="black", linewidth=0.5, ls="--")
    ax.grid()
    ax.legend()
    ax.set_title("Geofence and Robot Pose")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_ylim(-5, 10)
    ax.axis("equal")
    plt.show()

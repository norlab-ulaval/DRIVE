import numpy as np
from shapely.geometry import Point, Polygon

from DRIVE.common import Command, Pose


class Geofence:
    def __init__(self, coordinates: list[tuple[float, float]], origin: tuple[float, float] | None = None):
        """
        Initialize the Geofence with a list of (x, y) coordinates.
        :param coordinates: List of tuples representing the polygon's vertices.
        :param origin: Optional tuple representing the point of origin. Defaults to the polygon's centroid.
        """
        self.polygon = Polygon(coordinates)
        self.origin = origin if origin is not None else (self.polygon.centroid.x, self.polygon.centroid.y)

        if not self.is_point_inside(self.origin):
            raise ValueError("Origin point is outside the geofence.")

    def is_point_inside(self, point: tuple[float, float] | np.ndarray) -> bool:
        """
        Check if a point is inside the geofence.
        :param point: A tuple (x, y) representing the point.
        :return: True if the point is inside, False otherwise.
        """
        return self.polygon.contains(Point(point))


class GeofencingController:
    def __init__(self, geofence: Geofence, drive_speed: float = 0.4, turn_speed: float = 0.5):
        """
        Initializes the geofencing controller.
        :param geofence: The geofence object defining the allowed area.
        :param drive_speed: The forward movement speed of the robot when returning.
        :param turn_speed: The angular rotation speed of the robot when adjusting direction.
        """
        self.geofence = geofence
        self.drive_speed = drive_speed
        self.turn_speed = turn_speed
        self.override = False

    def update(self, pose: Pose):
        """
        Updates the override status based on the current position.
        If the robot is outside the geofence, override is enabled.
        :param pose: The current (x, y, yaw) position of the robot.
        """
        x, y, _ = pose
        if not self.geofence.is_point_inside((x, y)):
            self.override = True

    def get_command(self, pose: Pose) -> Command:
        """
        Generates movement commands to return the robot to the geofence origin.
        The robot will first turn toward the target point, then move forward.
        :param pose: The current (x, y, yaw) position of the robot.
        :return: A numpy array containing velocity commands [v_x, v_yaw].
                 v_x controls forward movement, and v_yaw controls rotation.
        """
        x, y, yaw = pose
        target_x, target_y = self.geofence.origin

        desired_yaw = np.arctan2(target_y - y, target_x - x)
        yaw_error = desired_yaw - yaw

        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        v_yaw = np.clip(yaw_error, -self.turn_speed, self.turn_speed)

        if abs(yaw_error) < 0.1:
            v_x = self.drive_speed
        else:
            v_x = 0.0

        if np.hypot(target_x - x, target_y - y) < 0.5:
            self.override = False

        return np.array([v_x, v_yaw])

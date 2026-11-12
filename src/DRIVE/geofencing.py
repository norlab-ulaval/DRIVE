import numpy as np
import shapely
from shapely.geometry import Point, Polygon

from DRIVE.common import Pose


class Geofence:
    def __init__(self, coordinates: list[tuple[float, float]], origin: tuple[float, float] | None = None):
        """
        Initialize the Geofence with a list of (x, y) coordinates.
        :param coordinates: List of tuples representing the polygon's vertices.
        :param origin: Optional tuple representing the point of origin. Defaults to the polygon's centroid.
        """
        self.polygon = Polygon(coordinates)
        self.points = self.polygon.exterior.coords
        self.origin = origin if origin is not None else (self.polygon.centroid.x, self.polygon.centroid.y)

        # if not self.is_point_inside(self.origin):
        #    raise ValueError("Origin point is outside the geofence.")

    def is_point_inside(self, point: tuple[float, float] | np.ndarray) -> bool:
        """
        Check if a point is inside the geofence.
        :param point: A tuple (x, y) representing the point.
        :return: True if the point is inside, False otherwise.
        """
        return self.polygon.contains(Point(point))

    def shrunk_polygon(self, margin_m: float) -> Polygon:
        """The geofence polygon, shrunk by a margin on every side."""
        return self.polygon.buffer(-margin_m) if margin_m > 0 else self.polygon

    def is_path_inside(self, path: np.ndarray, margin_m: float = 0.0) -> bool:
        """
        Check if every point of a path is inside the geofence.
        :param path: Array of shape (N, 2) with the (x, y) points of the path.
        :param margin_m: Minimum distance between the path and the edge of the geofence.
        """
        polygon = self.shrunk_polygon(margin_m)
        return bool(np.all(shapely.contains_xy(polygon, path[:, 0], path[:, 1])))

    def find_start_pose(
        self,
        local_path: np.ndarray,
        current_pose: Pose,
        nb_positions: int = 400,
        nb_headings: int = 16,
        heading_cost_m_per_rad: float = 1.0,
        min_distance_m: float = 1.0,
        margin_m: float = 0.0,
    ) -> Pose | None:
        """
        Find a start pose from which a path stays inside the geofence, as close as possible to the current pose.
        :param local_path: Array of shape (N, 2), the path relative to its start pose (start at (0, 0), facing +x).
        :param current_pose: Pose the robot would drive from to reach the start pose.
        :param nb_positions: Approximate number of candidate positions, spread on a grid over the geofence.
        :param nb_headings: Number of candidate headings for each position.
        :param heading_cost_m_per_rad: Cost of a heading that differs from the direction the robot drives to reach
            the start pose. A robot driven to a goal usually arrives facing that direction.
        :param min_distance_m: Minimum distance from the current pose. A closer goal can be considered reached right
            away by the robot's controller, without the robot moving or turning.
        :param margin_m: Minimum distance between the path and the edge of the geofence.
        :return: The start pose (x, y, z, roll, pitch, yaw), or None if the path does not fit anywhere.
        """
        polygon = self.shrunk_polygon(margin_m)
        if polygon.is_empty:
            return None

        min_x, min_y, max_x, max_y = polygon.bounds
        spacing = np.sqrt(polygon.area / nb_positions)

        grid_x, grid_y = np.meshgrid(np.arange(min_x, max_x, spacing), np.arange(min_y, max_y, spacing))
        positions = np.column_stack([grid_x.ravel(), grid_y.ravel()])
        positions = positions[shapely.contains_xy(polygon, positions[:, 0], positions[:, 1])]
        headings = np.linspace(-np.pi, np.pi, nb_headings, endpoint=False)

        candidate_xy = np.repeat(positions, len(headings), axis=0)
        candidate_yaw = np.tile(headings, len(positions))

        cos, sin = np.cos(candidate_yaw)[:, None], np.sin(candidate_yaw)[:, None]
        path_x = candidate_xy[:, :1] + cos * local_path[:, 0] - sin * local_path[:, 1]
        path_y = candidate_xy[:, 1:] + sin * local_path[:, 0] + cos * local_path[:, 1]
        fits = np.all(shapely.contains_xy(polygon, path_x, path_y), axis=1)

        offset = candidate_xy - current_pose[:2]
        distance = np.linalg.norm(offset, axis=1)
        valid = fits & (distance >= min_distance_m)
        if not np.any(valid):
            return None

        arrival_heading = np.arctan2(offset[:, 1], offset[:, 0])
        heading_error = np.abs((candidate_yaw - arrival_heading + np.pi) % (2 * np.pi) - np.pi)
        cost = np.where(valid, distance + heading_cost_m_per_rad * heading_error, np.inf)
        best = np.argmin(cost)

        x, y = candidate_xy[best]
        return np.array([x, y, 0.0, 0.0, 0.0, candidate_yaw[best]])

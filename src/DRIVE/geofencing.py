import numpy as np
from shapely.geometry import Point, Polygon


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

        #if not self.is_point_inside(self.origin):
        #    raise ValueError("Origin point is outside the geofence.")

    def is_point_inside(self, point: tuple[float, float] | np.ndarray) -> bool:
        """
        Check if a point is inside the geofence.
        :param point: A tuple (x, y) representing the point.
        :return: True if the point is inside, False otherwise.
        """
        return self.polygon.contains(Point(point))

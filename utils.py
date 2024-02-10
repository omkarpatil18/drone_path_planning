class TransformCoordinates:
    """Transform coordinates from world frame to occupancy grid frame and the other-way."""

    def __init__(self, drone_in_occ_grid, drone_in_world) -> None:
        self.occ_grid_in_world = drone_in_world - drone_in_occ_grid

    def occ_grid_to_world(self, path):
        world_path = []
        for waypoint in path:  # path is in occ_grid frame
            world_path.append(self.occ_grid_in_world + waypoint)
        return world_path

    def world_to_occ_grid(self, path):
        occ_grid_path = []
        for waypoint in path:  # path is in world frame
            occ_grid_path.append(waypoint - self.occ_grid_in_world)
        return occ_grid_path


import numpy as np


def get_points_on_line(p1, p2):
    """
    Get points on a line connecting two airgen Vector3r points in 3D.
    Points cover all integers for collision checking.

    Args:
      p1: The first point.
      p2: The second point.

    Returns:
      A list of points on the line.
    """

    # Get the direction vector.
    vec = p2 - p1

    # Normalize the direction vector.
    dist = vec.get_length()
    vec /= dist

    # Create a list of points on the line.
    points = []
    for i in range(int(dist) + 1):
        # Calculate the point at the ith position on the line.
        point = p1 + vec * i

        # Add the point to the list.
        points.append(point)
    points.append(p2)

    return points

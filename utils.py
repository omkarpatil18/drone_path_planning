import os
import sys
import ctypes
import numpy as np
import airgen

C_IMPL_DIR = "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl"
MAP_LOCATION = "/home/local/ASUAD/opatil3/Simulators/map.binvox"
# PLAN_FREQ = int(os.environ["PLAN_FREQ"])  # in meters. Should be <= HORIZON_LEN//2
# HORIZON_LEN = int(
#     os.environ["HORIZON_LEN"]
# )  # in meters. Keep in the form of 2x+1, preferrably small.
# SCALE = int(os.environ["SCALE"])
PLAN_FREQ = 49
HORIZON_LEN = 100
SCALE = 3

# Scale of voxel in the occupancy grid relative to the simulator world
DIST_THRESH = max(
    2 * SCALE, 7
)  # Maximum distance from the goal to be considered successful
ITER_THRESH = 100  # Maximum number of iterations to find a path

# cc -fPIC -g -shared -o mikami.so mikami.c

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
from binvox_rw import read_as_3d_array


class TransformCoordinates:
    """Transform coordinates from world frame to occupancy grid frame and the other-way."""

    def __init__(self, drone_in_occ_grid, drone_in_world) -> None:
        self.occ_grid_in_world = drone_in_world - self.rotate_frame(
            drone_in_occ_grid * SCALE
        )

    def occ_grid_to_world(self, path):
        world_path = []
        for waypoint in path:  # path is in occ_grid frame
            world_path.append(
                self.occ_grid_in_world + self.rotate_frame(waypoint * SCALE)
            )
        return world_path

    def world_to_occ_grid(self, path):
        occ_grid_path = []
        for waypoint in path:  # path is in world frame
            occ_grid_path.append(
                self.rotate_frame(waypoint - self.occ_grid_in_world) / SCALE
            )
        return occ_grid_path

    def rotate_frame(self, vec_3r):
        """If occ grid is XYZ, then Airgen is YX(-Z)"""
        temp_x = vec_3r.x_val
        vec_3r.x_val = vec_3r.y_val
        vec_3r.y_val = temp_x
        vec_3r.z_val = -vec_3r.z_val
        return vec_3r


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


# Classes to store the occupancy grid and the path found by C implementations
class OccupancyGrid(ctypes.Structure):
    _fields_ = [
        ("array", (((ctypes.c_float * HORIZON_LEN) * (HORIZON_LEN)) * (HORIZON_LEN)))
    ]


class Path(ctypes.Structure):
    _fields_ = [
        ("path_len", ctypes.c_int),
        ("array", (ctypes.c_float * 3) * (HORIZON_LEN * HORIZON_LEN * HORIZON_LEN)),
    ]


def get_vector3r_pose(px, py, pz):
    """Returns a Vector3r pose object given the x, y, z coordinates"""
    return airgen.Pose(
        airgen.Vector3r(px, py, pz),
        airgen.Quaternionr(airgen.Vector3r(0, 0, 0)),
    )

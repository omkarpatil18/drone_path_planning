import ctypes
import numpy as np
from constants import PLAN_FREQ, HORIZON_LEN

C_IMPL_DIR = "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl"


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
        ("array", ((ctypes.c_float * HORIZON_LEN) * (HORIZON_LEN) * (HORIZON_LEN)))
    ]


class Path(ctypes.Structure):
    _fields_ = [("array", (ctypes.c_float * 3) * (PLAN_FREQ + 1))]


######### Code to test the calling of C functions in Python #########
# Load c function
# so_file = f"{C_IMPL_DIR}/sline/sline.so"
# my_functions = ctypes.cdll.LoadLibrary(so_file)
# main = my_functions.planner
# array_type = ctypes.c_float * 3

# # Defining types and structures
# main.argtypes = (
#     ctypes.POINTER(ctypes.c_float),
#     ctypes.POINTER(ctypes.c_float),
#     ctypes.POINTER(Path),
#     ctypes.POINTER(OccupancyGrid),
# )
# main.restype = None

# # Call the c code
# path = Path()
# main(
#     array_type(*[0.0, 0.0, 0.0]),
#     array_type(*[3.0, 4.0, 5.0]),
#     ctypes.byref(path),
#     None,
# )
# print(np.ndarray((PLAN_FREQ + 1, 3), "f", path.array, order="C"))

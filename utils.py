import ctypes
import numpy as np
from constants import PLAN_FREQ, HORIZON_LEN, SCALE

C_IMPL_DIR = "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl"
# cc -fPIC -g -shared -o mikami.so mikami.c

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
        ("array", (ctypes.c_float * 3) * (PLAN_FREQ * 3)),
    ]


######### Code to test the calling of C straight line planner in Python #########
# # Load c function
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


######### Code to test the calling of C a-star planner in Python #########
# # Load c function
# so_file = f"{C_IMPL_DIR}/astar/astar.so"
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
# occ_grid = OccupancyGrid()
# for i in range(HORIZON_LEN):
#     for j in range(HORIZON_LEN):
#         for k in range(HORIZON_LEN):
#             occ_grid.array[i][j][k] = 0
#             if i == j == k and i != 0:
#                 occ_grid.array[i][j][k] = 1


# main(
#     array_type(*[0.0, 0.0, 0.0]),
#     array_type(*[5.0, 6.0, 5.0]),
#     ctypes.byref(path),
#     ctypes.byref(occ_grid),
# )
# print(np.ndarray((path.path_len, 3), "f", path.array, order="C"))


######### Code to test the calling of C a-star planner in Python #########
# # Load c function
# so_file = f"{C_IMPL_DIR}/mikami/mikami.so"
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
# occ_grid = OccupancyGrid()
# for i in range(HORIZON_LEN):
#     for j in range(HORIZON_LEN):
#         for k in range(HORIZON_LEN):
#             occ_grid.array[i][j][k] = 0
#             if i == j == k and i != 0:
#                 occ_grid.array[i][j][k] = 1


# main(
#     array_type(*[0.0, 0.0, 0.0]),
#     array_type(*[1.0, 57.0, 90.0]),
#     ctypes.byref(path),
#     ctypes.byref(occ_grid),
# )
# print(np.ndarray((path.path_len, 3), "f", path.array, order="C"))

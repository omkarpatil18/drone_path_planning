import airgen
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from constants import HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController
from utils import get_points_on_line, C_IMPL_DIR, Path, OccupancyGrid


class PyStraightLine(DroneController):
    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Python implementation of the actual planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """
        return get_points_on_line(start_pose, goal_pose)


class CStraightLine(DroneController):
    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Calls the C implementation of the actual planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """
        so_file = f"{C_IMPL_DIR}/sline/sline.so"
        planner_lib = ctypes.cdll.LoadLibrary(so_file)
        planner = planner_lib.planner
        array_type = ctypes.c_float * 3

        # Defining types and structures
        planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(Path),
            ctypes.POINTER(OccupancyGrid),
        )
        planner.restype = None

        # Call the c code
        path = Path()
        occ_grid = OccupancyGrid()

        planner(
            array_type(start_pose.x_val, start_pose.y_val, start_pose.z_val),
            array_type(goal_pose.x_val, goal_pose.y_val, goal_pose.z_val),
            ctypes.byref(path),
            ctypes.byref(occ_grid),
        )
        path = np.ndarray((PLAN_FREQ + 1, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(PLAN_FREQ):
            vector_path.append(airgen.Vector3r(*path[i][:]))
        return vector_path

# Plan path to random point
controller = CStraightLine()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

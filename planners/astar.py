import airgen
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from constants import HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController
from utils import get_points_on_line, C_IMPL_DIR, Path, OccupancyGrid


class CAStar(DroneController):

    def __init__(self):
        super().__init__()
        so_file = f"{C_IMPL_DIR}/astar/astar.so"
        planner_lib = ctypes.cdll.LoadLibrary(so_file)
        self.planner = planner_lib.planner
        self.array_type = ctypes.c_float * 3

        # Defining types and structures
        self.planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(Path),
            ctypes.POINTER(OccupancyGrid),
        )
        self.planner.restype = None

    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Calls the C implementation of the actual planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """

        path = Path()
        occ_grid_obj = OccupancyGrid()
        for i in range(HORIZON_LEN):
            for j in range(HORIZON_LEN):
                for k in range(HORIZON_LEN):
                    occ_grid_obj.array[i][j][k] = 1 if occ_grid.data[i][j][k] else 0

        # # Zero drone's voxels
        # for i in range(HORIZON_LEN // 2, (HORIZON_LEN // 2) + 1):
        #     for j in range(HORIZON_LEN // 2, (HORIZON_LEN // 2) + 1):
        #         for k in range(HORIZON_LEN // 2, (HORIZON_LEN // 2) + 1):
        #             occ_grid_obj.array[i][j][k] = 0
        occ_grid_obj.array[HORIZON_LEN // 2][HORIZON_LEN // 2][HORIZON_LEN // 2] = 0

        # Call the c code
        self.planner(
            self.array_type(start_pose.x_val, start_pose.y_val, start_pose.z_val),
            self.array_type(goal_pose.x_val, goal_pose.y_val, goal_pose.z_val),
            ctypes.byref(path),
            ctypes.byref(occ_grid_obj),
        )
        path_len = path.path_len
        path = np.ndarray((path_len, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(path_len):
            vector_path.append(airgen.Vector3r(*path[path_len - i - 1][:]))
        return vector_path


# Plan path to random point
controller = CAStar()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

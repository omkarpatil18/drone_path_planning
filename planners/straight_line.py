import airgen
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from utils import HORIZON_LEN, PLAN_FREQ
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
    def __init__(self):
        super().__init__()
        self.so_file = f"{C_IMPL_DIR}/sline/sline.so"


# Plan path to random point
controller = CStraightLine()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

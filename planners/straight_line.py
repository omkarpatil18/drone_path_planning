import airgen
import sys

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from constants import HORIZON_LEN
from drone_controller import DroneController
from utils import get_points_on_line


class StraightLine(DroneController):
    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Implementation of the actual planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """
        return get_points_on_line(start_pose, goal_pose)


# Plan path to random point
controller = StraightLine()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

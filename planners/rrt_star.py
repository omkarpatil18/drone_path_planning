import airgen
import numpy as np
from constants import MAP_LOCATION, HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController


class RRTStar(DroneController):
    def __init__(self, kwargs):
        super().__init__(**kwargs)

    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Implementing RRT* in the occupancy grid
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """

        trajectory = []
        return trajectory


# Plan path to random point
controller = RRTStar()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

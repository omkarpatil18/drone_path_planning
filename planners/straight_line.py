import airgen
from constants import HORIZON_LEN
from drone_controller import DroneController


class StraightLine(DroneController):
    def __init__(self, kwargs):
        super().__init__(**kwargs)

    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Transforms coordinates to within the occupancy grid and calls the planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """
        x_vec = goal_pose.location.x_val - start_pose.location.x_val
        y_vec = goal_pose.location.y_val - start_pose.location.y_val
        z_vec = goal_pose.location.z_val - start_pose.location.z_val

        transformed_start = (HORIZON_LEN // 2, HORIZON_LEN // 2, HORIZON_LEN // 2)
        transformed_goal = (
            HORIZON_LEN // 2 + x_vec,
            HORIZON_LEN // 2 + y_vec,
            HORIZON_LEN // 2 + z_vec,
        )

        return self.planner(transformed_start, transformed_goal, occ_grid)

    def planner(self, start, goal, occ_grid):
        """
        Implementation of the actual planner
        """
        pass


# Plan path to random point
controller = StraightLine()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

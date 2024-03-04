import airgen
import sys
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from utils import HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController
from utils import get_points_on_line, C_IMPL_DIR, Path, OccupancyGrid


class CAStar(DroneController):

    def __init__(self):
        self.so_file = f"{C_IMPL_DIR}/astar/astar_{HORIZON_LEN}.so"
        super().__init__()


# Plan path to random point
controller = CAStar()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

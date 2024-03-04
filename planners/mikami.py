import airgen
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from utils import HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController
from utils import get_points_on_line, C_IMPL_DIR, Path, OccupancyGrid


class CMikami(DroneController):

    def __init__(self):
        self.so_file = f"{C_IMPL_DIR}/mikami/mikami_air_{HORIZON_LEN}.so"
        super().__init__()

    def process_path(self, path):
        path_len = path.path_len
        path = np.ndarray((path_len, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(path_len):
            vector_path.append(airgen.Vector3r(*path[i][:]))
        return vector_path


# Plan path to random point
controller = CMikami()
try:
    controller.plan_and_move()
except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

import airgen
import sys
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from utils import HORIZON_LEN, PLAN_FREQ
from drone_controller import DroneController
from airgen_env import POSE_SET
from utils import (
    C_IMPL_DIR,
    get_vector3r_pose,
)


class CAStar(DroneController):

    def __init__(self, **kwargs):
        self.so_file = f"{C_IMPL_DIR}/astar/astar_{HORIZON_LEN}.so"
        super().__init__(**kwargs)


# ENV = "AbandonedCableFactory"
# ENV = "Blocks"
ENV = "NearNeighborhood"

POSES = POSE_SET[ENV]
# Plan path to random point
controller = CAStar(env_name=ENV, planner_name="A-Star")
try:
    for spos, gpos in POSES:
        sp = get_vector3r_pose(*spos)
        gp = get_vector3r_pose(*gpos)
        controller.plan_and_move((sp, gp))
    controller.write_results()

except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

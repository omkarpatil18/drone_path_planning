"""
TODO:
- Set num levels to 3
- Run with vertically raised goal point bias if a path is not found
- Artificially inflate the occupancy grid for small horizon lengths 
"""

import airgen
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from utils import HORIZON_LEN, PLAN_FREQ, SCALE, INFLATE
from drone_controller import DroneController
from airgen_env import POSE_SET
from utils import (
    C_IMPL_DIR,
    get_vector3r_pose,
)


class CMikami(DroneController):
    

    def __init__(self, **kwargs):
        self.so_file = f"{C_IMPL_DIR}/mikami/mikami_{HORIZON_LEN}.so"
        super().__init__(**kwargs)

    def process_path(self, path):
        path_len = path.path_len
        path = np.ndarray((path_len, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(path_len):
            vector_path.append(airgen.Vector3r(*path[i][:]))
        return vector_path


# ENV = "AbandonedCableFactory"
ENV = "Blocks"
# ENV = "OilRig"
# ENV = "ElectricCentral"
# ENV = "FarNeighborhood"
# ENV = "NearNeighborhood"

POSES = POSE_SET[ENV]
# Plan path to random point
print(
    "Running setting horizon length: ",
    HORIZON_LEN,
    ", plan frequency: ",
    PLAN_FREQ,
    "and scale to: ",
    SCALE,
    "for environment: ",
    ENV,
    "inflate: ",
    INFLATE,
)

controller = CMikami(env_name=ENV, planner_name="Mikami")
try:
    for spos, gpos in POSES[:25]:
        sp = get_vector3r_pose(*spos)
        gp = get_vector3r_pose(*gpos)
        controller.plan_and_move((sp, gp))
    controller.write_results()

except KeyboardInterrupt:
    print("Keyboard interrupt detected, exiting...")
    client = airgen.MultirotorClient()
    client.reset()

# controller.drone_client.reset()
# controller.drone_client.confirmConnection()
# controller.drone_client.enableApiControl(True)
# controller.drone_client.takeoffAsync().join()
# cntr = 0
# while cntr < 50:
#     sp, gp = controller.spawn_poses()
#     if (
#         sp.position.distance_to(gp.position) < 50
#         and sp.position.distance_to(gp.position) > 30
#     ):
#         print("[", sp.position, ", ", gp.position, "],")
#         cntr += 1

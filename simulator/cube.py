import numpy as np
import sys
import os
import ctypes

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")

from binvox_rw import Voxels, write, sparse_to_dense
from utils import C_IMPL_DIR

HORIZON_LEN = 100


class CTypesGrid(ctypes.Structure):
    _fields_ = [
        ("array", (((ctypes.c_float * HORIZON_LEN) * (HORIZON_LEN)) * (HORIZON_LEN)))
    ]


class CTypesPath(ctypes.Structure):
    _fields_ = [
        ("path_len", ctypes.c_int),
        ("array", (ctypes.c_float * 3) * (HORIZON_LEN * HORIZON_LEN * HORIZON_LEN)),
    ]


class Cube:
    """Simulates a cube representing an environment with obstacles"""

    def __init__(self, density, horizon_len) -> None:
        self.density = density
        self.horizon_len = horizon_len
        self.dir_path = f"/home/local/ASUAD/opatil3/src/drone_path_planning/simulator/envs/dim_{HORIZON_LEN}/density_{self.density}/"
        if not os.path.isdir(self.dir_path):
            os.makedirs(self.dir_path)
        self.create_cube()

    def create_cube(self):
        self.cube = np.random.choice(
            a=[False, True],
            size=[self.horizon_len, self.horizon_len, self.horizon_len],
            replace=True,
            p=[1 - self.density, self.density],
        )
        return self.cube

    def check_for_valid_path(self):
        """
        Uses a-star to check if a valid path exists.
        Returns True if a path is found else False
        """
        # Initialize planner defined in C
        so_file = f"{C_IMPL_DIR}/astar/astar_{HORIZON_LEN}.so"
        planner_lib = ctypes.cdll.LoadLibrary(so_file)
        planner = planner_lib.planner
        array_type = ctypes.c_float * 3

        # Defining types and structures
        planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(CTypesPath),
            ctypes.POINTER(CTypesGrid),
        )
        planner.restype = None
        path = CTypesPath()
        occ_grid_obj = CTypesGrid()
        start_pose = [0, 0, 0]
        goal_pose = [self.horizon_len - 1, self.horizon_len - 1, self.horizon_len - 1]

        for i in range(self.horizon_len):
            for j in range(self.horizon_len):
                for k in range(self.horizon_len):
                    occ_grid_obj.array[i][j][k] = 1 if self.cube[i][j][k] else 0

        occ_grid_obj.array[start_pose[0]][start_pose[1]][start_pose[2]] = 0
        occ_grid_obj.array[goal_pose[0]][goal_pose[1]][goal_pose[2]] = 0

        # Call the c code
        planner(
            array_type(*start_pose),
            array_type(*goal_pose),
            ctypes.byref(path),
            ctypes.byref(occ_grid_obj),
        )
        self.path_arr = np.ndarray((path.path_len, 3), "f", path.array, order="C")
        if path.path_len == 0:
            return False
        return True

    def save_path_binvox(self, idx):
        path_cube = np.zeros(
            (self.horizon_len, self.horizon_len, self.horizon_len), dtype=int
        )
        for p_idx in range(self.path_arr.shape[0]):
            i, j, k = self.path_arr[p_idx, :]
            path_cube[int(i)][int(j)][int(k)] = 1
        voxels = Voxels(
            data=path_cube,
            dims=[self.horizon_len, self.horizon_len, self.horizon_len],
            translate=[0.0, 0.0, 0.0],
            scale=1,
            axis_order="xyz",
        )
        with open(
            os.path.join(self.dir_path, f"env_{idx}_path.binvox"),
            "wb",
        ) as fp:
            write(voxels, fp)

    def save_env_binvox(self, idx):
        voxels = Voxels(
            data=self.cube,
            dims=[self.horizon_len, self.horizon_len, self.horizon_len],
            translate=[0.0, 0.0, 0.0],
            scale=1,
            axis_order="xyz",
        )
        with open(
            os.path.join(self.dir_path, f"env_{idx}.binvox"),
            "wb",
        ) as fp:
            write(voxels, fp)


for idx in range(100):
    path_found = False
    while not path_found:
        c = Cube(density=0.5, horizon_len=HORIZON_LEN)
        path_found = c.check_for_valid_path()
        if path_found:
            c.save_env_binvox(idx)
            c.save_path_binvox(idx)

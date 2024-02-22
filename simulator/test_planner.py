import os
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
from utils import C_IMPL_DIR
from binvox_rw import read_as_3d_array

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


class TestPlanner:
    def __init__(self, dim, density, planner="") -> None:
        self.dim = dim
        self.density = density
        self.dir_path = f"/home/local/ASUAD/opatil3/src/drone_path_planning/simulator/envs/dim_{self.dim}/density_{self.density}"
        so_file = f"{C_IMPL_DIR}/{planner}/{planner}_{HORIZON_LEN}.so"
        planner_lib = ctypes.cdll.LoadLibrary(so_file)
        self.planner = planner_lib.planner
        self.array_type = ctypes.c_float * 3

        # Defining types and structures
        self.planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(CTypesPath),
            ctypes.POINTER(CTypesGrid),
        )
        self.planner.restype = None
        self.occ_grid_obj = CTypesGrid()
        self.start_pose = [0, 0, 0]
        self.goal_pose = [
            HORIZON_LEN - 1,
            HORIZON_LEN - 1,
            HORIZON_LEN - 1,
        ]

        # Store results
        self.path_found = []

    def set_occupancy_grid(self, idx):
        with open(os.path.join(self.dir_path, f"env_{idx}.binvox"), "rb") as f:
            occ_grid = read_as_3d_array(f)

        for i in range(HORIZON_LEN):
            for j in range(HORIZON_LEN):
                for k in range(HORIZON_LEN):
                    self.occ_grid_obj.array[i][j][k] = (
                        1 if occ_grid.data[i][j][k] else 0
                    )

        self.occ_grid_obj.array[self.start_pose[0]][self.start_pose[1]][
            self.start_pose[2]
        ] = 0
        self.occ_grid_obj.array[self.goal_pose[0]][self.goal_pose[1]][
            self.goal_pose[2]
        ] = 0

    def plan(self):
        # Call the c code
        path = CTypesPath()
        self.planner(
            self.array_type(*self.start_pose),
            self.array_type(*self.goal_pose),
            ctypes.byref(path),
            ctypes.byref(self.occ_grid_obj),
        )
        self.path_arr = np.ndarray((path.path_len, 3), "f", path.array, order="C")
        if path.path_len != 0:
            return True

    def test(self):
        for idx in range(100):
            self.set_occupancy_grid(idx)
            self.path_found.append(self.plan())
        print(self.path_found)


tp = TestPlanner(dim=HORIZON_LEN, density=0.01, planner="mikami")
tp.test()

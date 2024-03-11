import os
import sys
import ctypes
import numpy as np

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
from binvox_rw import read_as_3d_array

HORIZON_LEN = 100
C_IMPL_DIR = "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl"


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
    def __init__(self, dim, density, obs_size, planner="") -> None:
        self.dim = dim
        self.density = density
        self.obs_size = obs_size
        self.planner_name = planner
        self.dir_path = f"/home/local/ASUAD/opatil3/src/drone_path_planning/simulator/env_{str(obs_size)[0]}x/dim_{self.dim}/density_{self.density}"
        self.so_file = f"{C_IMPL_DIR}/{planner}/{planner}_{HORIZON_LEN}.so"
        self.start_pose = [0, 0, 0]
        self.goal_pose = [
            HORIZON_LEN - 1,
            HORIZON_LEN - 1,
            HORIZON_LEN - 1,
        ]
        # Store results
        self.path_found = []
        self.time = []
        self.occ_grid_density = []
        self.path_len = []

    def set_occupancy_grid(self, idx):
        self.occ_grid_obj = CTypesGrid()
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
        self.occ_grid_density.append(np.mean(self.occ_grid_obj.array))

    def plan(self):
        # Call the c code
        planner_lib = ctypes.CDLL(self.so_file)
        self.planner = planner_lib.planner
        self.array_type = ctypes.c_float * 3

        # Defining types and structures
        self.planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(CTypesPath),
            ctypes.POINTER(CTypesGrid),
        )
        self.planner.restype = ctypes.c_double
        path = CTypesPath()
        self.time.append(
            self.planner(
                self.array_type(*self.start_pose),
                self.array_type(*self.goal_pose),
                ctypes.byref(path),
                ctypes.byref(self.occ_grid_obj),
            )
        )
        # self.path_arr = np.ndarray((path.path_len, 3), "f", path.array, order="C")
        if path.path_len != 0:
            self.path_len.append(self.get_path_len(path))
            return True
        return False

    def get_path_len(self, path):
        """Returns path length"""
        path_len = 0
        for i in range(path.path_len - 1):
            path_len += np.linalg.norm(
                [
                    path.array[i][0] - path.array[i + 1][0],
                    path.array[i][1] - path.array[i + 1][1],
                    path.array[i][2] - path.array[i + 1][2],
                ]
            )
        return path_len

    def test(self):
        for k in range(10):
            print(
                f"#################################### Iteration {k} ########################################"
            )
            for idx in range(100):
                print(f"***************************** starting level {idx}")
                self.set_occupancy_grid(idx)
                self.path_found.append(self.plan())
        print(
            f"Results for {self.planner_name}: ", self.dim, self.density, self.obs_size
        )
        print("-------------------------------------")
        print(np.all(self.path_found))
        print("-------------------------------------")
        print(np.mean(self.occ_grid_density), np.var(self.occ_grid_density))
        print("\n-------------------------------------\n")
        print(f"Path length: {np.mean(self.path_len)}, {np.var(self.path_len)}")
        print("-------------------------------------")
        print("Time: ", np.mean(self.time), np.var(self.time))
        with open(
            os.path.join(
                f"/home/local/ASUAD/opatil3/src/drone_path_planning/simulator/results/",
                f"planner_{self.planner_name}_dim_{self.dim}_obs_{self.obs_size}_density_{self.density}",
            ),
            "w",
        ) as f:
            f.write(
                f"Results for {self.planner_name}: {self.dim}, {self.density}, {self.obs_size}"
            )
            f.write("\n-------------------------------------\n")
            f.write(f"All paths found: {np.all(self.path_found)}")
            f.write("\n-------------------------------------\n")
            f.write(
                f"Occupancy grid: {np.mean(self.occ_grid_density)}, {np.var(self.occ_grid_density)}"
            )
            f.write("\n-------------------------------------\n")
            f.write(
                f"Path length mean: {np.mean(self.path_len)}, {np.var(self.path_len)}"
            )
            f.write("\n-------------------------------------\n")
            f.write(f"Time median: {np.median(self.time)}")
            f.write("\n-------------------------------------\n")
            f.write(f"Time mean: {np.mean(self.time)}, {np.var(self.time)}")


tp = TestPlanner(dim=HORIZON_LEN, density=0.3, obs_size=50, planner="astar")
tp.test()

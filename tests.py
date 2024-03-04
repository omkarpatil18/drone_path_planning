import ctypes
import os
import sys
from utils import HORIZON_LEN, PLAN_FREQ, C_IMPL_DIR, Path, OccupancyGrid
from binvox_rw import read_as_3d_array
import numpy as np


def test_sline():
    ######## Code to test the calling of C straight line planner in Python #########
    so_file = f"{C_IMPL_DIR}/sline/sline.so"
    my_functions = ctypes.cdll.LoadLibrary(so_file)
    main = my_functions.planner
    array_type = ctypes.c_float * 3

    # Defining types and structures
    main.argtypes = (
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(Path),
        ctypes.POINTER(OccupancyGrid),
    )
    main.restype = None

    # Call the c code
    path = Path()
    main(
        array_type(*[0.0, 0.0, 0.0]),
        array_type(*[3.0, 4.0, 5.0]),
        ctypes.byref(path),
        None,
    )
    print(np.ndarray((PLAN_FREQ + 1, 3), "f", path.array, order="C"))


def test_astar():
    ######## Code to test the calling of C a-star planner in Python #########
    so_file = f"{C_IMPL_DIR}/astar/astar_10.so"
    my_functions = ctypes.cdll.LoadLibrary(so_file)
    main = my_functions.planner
    array_type = ctypes.c_float * 3

    # Defining types and structures
    main.argtypes = (
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(Path),
        ctypes.POINTER(OccupancyGrid),
    )
    main.restype = ctypes.c_double

    # Call the c code
    path = Path()
    occ_grid = OccupancyGrid()
    for i in range(HORIZON_LEN):
        for j in range(HORIZON_LEN):
            for k in range(HORIZON_LEN):
                occ_grid.array[i][j][k] = 0
                if i == j == k and i != 0:
                    occ_grid.array[i][j][k] = 1

    print(
        main(
            array_type(*[0.0, 0.0, 0.0]),
            array_type(*[5.0, 6.0, 5.0]),
            ctypes.byref(path),
            ctypes.byref(occ_grid),
        )
    )
    print(np.ndarray((path.path_len, 3), "f", path.array, order="C"))


def test_mikami():
    ######## Code to test the calling of C mikami planner in Python #########
    density = 0.1
    idx = 31
    start_pose = [0, 0, 0]
    goal_pose = [
        HORIZON_LEN - 1,
        HORIZON_LEN - 1,
        HORIZON_LEN - 1,
    ]
    so_file = f"{C_IMPL_DIR}/mikami/mikami_10.so"
    dir_path = f"/home/local/ASUAD/opatil3/src/drone_path_planning/simulator/env/dim_{HORIZON_LEN}/density_{density}"
    my_functions = ctypes.cdll.LoadLibrary(so_file)
    main = my_functions.planner
    array_type = ctypes.c_float * 3

    # Defining types and structures
    main.argtypes = (
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(Path),
        ctypes.POINTER(OccupancyGrid),
    )
    main.restype = None

    path = Path()
    occ_grid_obj = OccupancyGrid()

    with open(os.path.join(dir_path, f"env_{idx}.binvox"), "rb") as f:
        occ_grid = read_as_3d_array(f)

    for i in range(HORIZON_LEN):
        for j in range(HORIZON_LEN):
            for k in range(HORIZON_LEN):
                occ_grid_obj.array[i][j][k] = 1 if occ_grid.data[i][j][k] else 0

    occ_grid_obj.array[start_pose[0]][start_pose[1]][start_pose[2]] = 0
    occ_grid_obj.array[goal_pose[0]][goal_pose[1]][goal_pose[2]] = 0

    main(
        array_type(*start_pose),
        array_type(*goal_pose),
        ctypes.byref(path),
        ctypes.byref(occ_grid_obj),
    )
    print(np.ndarray((path.path_len, 3), "f", path.array, order="C"))

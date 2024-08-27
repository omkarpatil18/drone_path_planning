import os
import sys
import airgen
import numpy as np
import ctypes
import cmath
import threading
from copy import copy
import time

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
import binvox_rw
from utils import (
    MAP_LOCATION,
    HORIZON_LEN,
    PLAN_FREQ,
    DIST_THRESH,
    SCALE,
    ITER_THRESH,
    INFLATE,
)
from utils import TransformCoordinates, Path, OccupancyGrid, get_vector3r_pose


class DroneController:

    def __init__(self, env_name="", planner_name=""):
        # Used to store the results
        self.env_name = env_name
        self.planner_name = planner_name

        # Initialize a drone client
        self.drone_client = airgen.connect_airgen(robot_type="multirotor")

        # Get navigation map information from the simulation world
        self.nav_mesh_info = self.drone_client.getNavMeshInfo()

        # Initialize planner defined in C
        planner_lib = ctypes.cdll.LoadLibrary(self.so_file)
        self.planner = planner_lib.planner
        self.array_type = ctypes.c_float * 3

        # Defining types and structures
        self.planner.argtypes = (
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(Path),
            ctypes.POINTER(OccupancyGrid),
        )
        self.planner.restype = ctypes.c_double

        # Results
        self.path_found = []
        self.path_time = []
        self.occ_grid_density = []
        self.path_len = []
        self.latency = []

    def sample_random_pose(self):
        # calculate bounds of the nav mesh
        amplitude = (
            np.absolute(
                np.array(
                    [
                        self.nav_mesh_info[2]["x_val"] - self.nav_mesh_info[1]["x_val"],
                        self.nav_mesh_info[2]["y_val"] - self.nav_mesh_info[1]["y_val"],
                        self.nav_mesh_info[2]["z_val"] - self.nav_mesh_info[1]["z_val"],
                    ]
                )
            )
            / 2.0
        )
        # sample a random point on the nav mesh
        random_point = airgen.Vector3r(
            np.random.uniform(
                self.nav_mesh_info[0]["x_val"] - amplitude[0],
                self.nav_mesh_info[0]["x_val"] + amplitude[0],
            ),
            np.random.uniform(
                self.nav_mesh_info[0]["y_val"] - amplitude[1],
                self.nav_mesh_info[0]["y_val"] + amplitude[1],
            ),
            np.random.uniform(
                self.nav_mesh_info[0]["z_val"] - amplitude[2],
                self.nav_mesh_info[0]["z_val"] + amplitude[2],
            ),
        )
        return airgen.Pose(random_point, airgen.Quaternionr(airgen.Vector3r(0, 0, 0)))

    def spawn_poses(self):
        """Returns the start and goal pose of the drone"""
        # Get current pose of the drone in 6-DOF
        start_pose = self.drone_client.simGetVehiclePose()
        valid_goal = False
        while not valid_goal:
            # Sample a random valid pose in the environment
            goal_pose = self.sample_random_pose()
            goal_pose.position.z_val = start_pose.position.z_val

            # # Goal pose for testing
            # goal_pose = get_vector3r_pose(0, 0, 0)
            # goal_pose.position.z_val = start_pose.position.z_val

            # Make sure that the goal does not coincide with an obstacle
            self.drone_client.simCreateVoxelGrid(
                goal_pose.position,
                1,
                1,
                1,
                1,
                MAP_LOCATION,
            )
            with open(MAP_LOCATION, "rb") as f:
                occ_grid = binvox_rw.read_as_3d_array(f)
            if occ_grid.data[0][0][0] == False:
                valid_goal = True
        return start_pose, goal_pose

    def inflate_occ_grid(self, occ_grid, x):
        """
        Inflates the occ grid by x ():
        a, a, a -> ax, ax, ax
        """
        # infl_grid = occ_grid.data.repeat(x, axis=0).repeat(x, axis=1).repeat(x, axis=2)
        inflated_occ_grid = np.kron(
            occ_grid.data, np.ones((x, x, x), dtype=occ_grid.data.dtype)
        )
        # assert infl_grid.all() == inflated_occ_grid.all()
        return binvox_rw.Voxels(
            data=inflated_occ_grid,
            dims=occ_grid.dims * INFLATE,
            translate=occ_grid.translate,
            scale=occ_grid.scale,
            axis_order=occ_grid.axis_order,
        )

    def plan_and_move(self, poses=None):
        """
        Plan the path for a drone flying with a ~constant velocity of 5m/s.
        The velocity is reduced as the target approaches and the
        breaking distance is decided based on the drone velocity.
        https://github.com/Microsoft/AirSim/blob/main/docs/apis.md#drivetrain

        The default implementation is in the find_path function.
        TODO: File I/O of the voxel grid is an unnecessary overhead
        """
        global PLAN_FREQ, HORIZON_LEN, SCALE, INFLATE
        try:
            self.drone_client.reset()
            self.drone_client.confirmConnection()
            self.drone_client.enableApiControl(True)
            self.drone_client.takeoffAsync().join()
        except Exception as e:
            time.sleep(3)
            self.drone_client.reset()
            self.drone_client.confirmConnection()
            self.drone_client.enableApiControl(True)
            self.drone_client.takeoffAsync().join()

        if poses is None:
            start_pose, goal_pose = self.spawn_poses()
        else:
            _, goal_pose = poses
            start_pose = self.drone_client.simGetVehiclePose()
        print(f"Moving to goal position: {goal_pose.position}")

        # Collect metrics for each goal
        move_planar = False  # move on the same XY plane
        plan_latency = []
        path_len = 0
        occ_grid_density = []
        path_found = True

        start_t = time.perf_counter()
        while start_pose.position.distance_to(goal_pose.position) > DIST_THRESH and (
            np.linalg.norm(
                [
                    start_pose.position.x_val - goal_pose.position.x_val,
                    start_pose.position.y_val - goal_pose.position.y_val,
                ]
            )
            > DIST_THRESH / 2
            or np.abs(start_pose.position.z_val - goal_pose.position.z_val) > PLAN_FREQ
        ):
            if time.perf_counter() - start_t > 200:
                print("Could not find a path to the goal. Exiting...")
                path_found = False
                end_t = time.perf_counter()
                break

            pose_vec = goal_pose.position - start_pose.position
            x_vec = pose_vec.x_val
            y_vec = pose_vec.y_val
            z_vec = pose_vec.z_val

            # Get heading towards the final goal
            r_xy, theta = cmath.polar(complex(x_vec, y_vec))
            rS, phi = cmath.polar(complex(r_xy, z_vec))
            if rS > PLAN_FREQ * SCALE:
                rS = PLAN_FREQ * SCALE

            # local planning
            get_coords = lambda cnum: (int(cnum.real), int(cnum.imag))
            goal_xy, goal_z = get_coords(cmath.rect(rS, phi))
            goal_x, goal_y = get_coords(cmath.rect(goal_xy, theta))
            if move_planar:
                goal_z = 0
                move_planar = False

            interrim_goal_vec = airgen.Vector3r(
                start_pose.position.x_val + goal_x,
                start_pose.position.y_val + goal_y,
                start_pose.position.z_val + goal_z,
            )

            # For debugging
            print(
                "*** Current pos: ",
                start_pose.position,
                "| Interrim position: ",
                interrim_goal_vec,
                "| At a dist: ",
                rS,
            )

            # Get voxels for planning
            self.drone_client.simCreateVoxelGrid(
                start_pose.position,
                int(HORIZON_LEN * SCALE),
                int(HORIZON_LEN * SCALE),
                int(HORIZON_LEN * SCALE),
                int(SCALE * INFLATE),
                MAP_LOCATION,
            )
            with open(MAP_LOCATION, "rb") as f:
                occ_grid = binvox_rw.read_as_3d_array(f)
                occ_grid = self.inflate_occ_grid(occ_grid, INFLATE)

            occ_grid_density.append(np.mean(occ_grid.data))

            # Transform coordinates to occupancy grid frame
            # For now assume that the drone is at the center of the occcupancy grid
            drone_in_occ_grid = airgen.Vector3r(
                HORIZON_LEN // 2 -1,
                HORIZON_LEN // 2 -1,
                HORIZON_LEN // 2 -1,
            )
            trans_coords = TransformCoordinates(
                drone_in_occ_grid=copy(drone_in_occ_grid),
                drone_in_world=copy(start_pose.position),
            )
            goal_in_occ_grid = trans_coords.world_to_occ_grid([copy(interrim_goal_vec)])

            # Find a valid path
            trajectory_in_occ_grid, latency = self.find_path(
                copy(drone_in_occ_grid), copy(goal_in_occ_grid[0]), occ_grid
            )

            # Handle the cases where a path is not found. Could be because the interrim goal
            # lies on an obstacle or a path actually does not exist.
            switch = 1
            angle_mult = 0.5
            theta0 = theta
            stuck = 0  # to prevent infinite loop
            while len(trajectory_in_occ_grid) <= 1:
                # Change the direction on the XY plane if possible
                if stuck > 50:
                    break
                if angle_mult == 8:
                    stuck += 1
                    if stuck > 4:
                        print("Stuck! Changing the interrim goal position.")
                        goal_x, goal_y = (
                            stuck * np.random.choice([-1, 1]),
                            stuck * np.random.choice([-1, 1]),
                        )
                    else:
                        goal_x, goal_y = 0, 0
                        move_planar = True

                    _, goal_z = get_coords(
                        cmath.rect(max(rS / 4, SCALE + 1), -np.pi / 2)
                    )
                else:
                    if switch == 1:
                        angle_mult = angle_mult * 2
                    theta = theta0 + np.pi * switch * angle_mult / 16
                    switch = switch * -1
                    goal_xy, goal_z = get_coords(cmath.rect(rS, phi))
                    goal_x, goal_y = get_coords(cmath.rect(goal_xy, theta))
                    if move_planar:
                        goal_z = 0
                        move_planar = False

                interrim_goal_vec = airgen.Vector3r(
                    start_pose.position.x_val + goal_x,
                    start_pose.position.y_val + goal_y,
                    start_pose.position.z_val + goal_z,
                )
                # For debugging
                print(
                    "*** Current pos: ",
                    start_pose.position,
                    "| Interrim position: ",
                    interrim_goal_vec,
                    "| At a dist: ",
                    rS,
                )
                goal_in_occ_grid = trans_coords.world_to_occ_grid(
                    [copy(interrim_goal_vec)]
                )

                # Find a valid path
                trajectory_in_occ_grid, latency = self.find_path(
                    copy(drone_in_occ_grid), copy(goal_in_occ_grid[0]), occ_grid
                )
            # Add latency to the results list
            plan_latency.append(latency)

            # Transform coordinates back to world frame
            trajectory_in_world = trans_coords.occ_grid_to_world(
                copy(trajectory_in_occ_grid)
            )
            for i in range(len(trajectory_in_world) - 1):
                path_len += trajectory_in_world[i].distance_to(
                    trajectory_in_world[i + 1]
                )

            # Run this piece of code in a thread in case the drone is stuck
            # Move the drone along the planned path at a velocity of 5 m/s
            velocity = 5.0
            if SCALE * INFLATE < 3:
                velocity = 2.0
            t = threading.Thread(
                target=self.drone_client.moveOnPathAsync(
                    trajectory_in_world,
                    velocity,
                    120,
                    airgen.DrivetrainType.MaxDegreeOfFreedom,
                    airgen.YawMode(False, 0),
                    -1,
                    0,
                ).join
            )
            t.start()
            t.join(75)  # terminate the thread after 120 seconds
            if t.is_alive():
                print("Thread terminated early, the drone might be stuck.")
                path_found = False
                end_t = time.perf_counter()
                break

            # Get the pose at the end of plan traversal
            start_pose = self.drone_client.simGetVehiclePose()
        end_t = time.perf_counter()

        # Collect metrics for each goal
        self.path_len.append(path_len)
        self.path_time.append(end_t - start_t)
        self.path_found.append(path_found)
        self.occ_grid_density.append(np.mean(occ_grid_density))
        self.latency.append(np.mean(plan_latency))
        print(plan_latency)
        print(f"##### Destination {goal_pose.position} reached (or not)! #####")

    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Calls the C implementation of the actual planner
        Parameters: start_pose, goal_pose, occupancy_grid
        Returns: trajectory (list of waypoints)
        """

        path = Path()
        occ_grid_obj = OccupancyGrid()
        for i in range(HORIZON_LEN):
            for j in range(HORIZON_LEN):
                for k in range(HORIZON_LEN):
                    occ_grid_obj.array[i][j][k] = 1 if occ_grid.data[i][j][k] else 0

        # Zero drone's voxels
        for i in range(int(-INFLATE / 2), int(INFLATE / 2) + 1):
            for j in range(int(-INFLATE / 2), int(INFLATE / 2) + 1):
                for k in range(int(-INFLATE / 2), int(INFLATE / 2) + 1):
                    occ_grid_obj.array[HORIZON_LEN // 2 + i][HORIZON_LEN // 2 + j][
                        HORIZON_LEN // 2 + k
                    ] = 0

        # Call the c code
        latency = self.planner(
            self.array_type(start_pose.x_val, start_pose.y_val, start_pose.z_val),
            self.array_type(
                int(goal_pose.x_val), int(goal_pose.y_val), int(goal_pose.z_val)
            ),
            ctypes.byref(path),
            ctypes.byref(occ_grid_obj),
        )
        return self.process_path(path), latency

    def process_path(self, path):
        path_len = path.path_len
        path = np.ndarray((path_len, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(path_len):
            vector_path.append(airgen.Vector3r(*path[path_len - i - 1][:]))
        return vector_path

    def write_results(self):
        # Convert to numpy arrays
        self.path_found = np.array(self.path_found)
        self.path_len = np.array(self.path_len)
        self.path_time = np.array(self.path_time)
        self.latency = np.array(self.latency)
        self.occ_grid_density = np.array(self.occ_grid_density)

        with open(
            os.path.join(
                f"/home/local/ASUAD/opatil3/src/drone_path_planning/planners/results/",
                f"{self.planner_name}_{self.env_name}_{time.time()}.txt",
            ),
            "w",
        ) as f:
            f.write("Experiment parameters =========================================\n")
            f.write(f"Environment: {self.env_name}\n")
            f.write(f"Planner: {self.planner_name}\n")
            f.write(f"Horizon length: {HORIZON_LEN}\n")
            f.write(f"Plan frequency: {PLAN_FREQ}\n")
            f.write(f"Distance threshold: {DIST_THRESH}\n")
            f.write(f"Scale: {SCALE}\n")
            f.write(f"Iteration threshold: {ITER_THRESH}\n")
            f.write("\n")

            f.write(
                "Aggregate metrics for *successful trials only* =========================================\n"
            )
            f.write(f"Path found: {np.all(self.path_found)}\n")
            f.write(
                f"Path length: {np.mean(self.path_len[self.path_found])}, {np.var(self.path_len[self.path_found])}\n"
            )
            f.write(
                f"Path time: {np.mean(self.path_time[self.path_found])}, {np.var(self.path_time[self.path_found])}\n"
            )
            f.write(
                f"Latency: {np.mean(self.latency[self.path_found])}, {np.var(self.latency[self.path_found])}\n"
            )
            f.write(
                f"Occ grid density: {np.mean(self.occ_grid_density[self.path_found])}, {np.var(self.occ_grid_density[self.path_found])}\n"
            )
            f.write("\n")

            f.write("Data dump =========================================\n")
            f.write(f"Path found: {self.path_found}\n")
            f.write(f"Path length: {self.path_len}\n")
            f.write(f"Path time: {self.path_time}\n")
            f.write(f"Latency: {self.latency}\n")
            f.write(f"Occ grid density: {self.occ_grid_density}\n")
            f.write("\n")

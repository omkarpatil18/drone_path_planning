import sys
import airgen
import numpy as np
import ctypes
import cmath
from copy import copy

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
import binvox_rw
from utils import MAP_LOCATION, HORIZON_LEN, PLAN_FREQ, DIST_THRESH, SCALE
from utils import TransformCoordinates, Path, OccupancyGrid


class DroneController:
    def __init__(self):
        # Initialize a drone client
        self.drone_client = airgen.connect_airgen(robot_type="multirotor")

        # Reset previous state if any
        self.drone_client.reset()

        # Confirm the connection, and enable offboard (API) control
        self.drone_client.confirmConnection()
        self.drone_client.enableApiControl(True)

        # Take off
        self.drone_client.takeoffAsync().join()

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
        self.planner.restype = None

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
            while goal_pose.position.z_val > 0:
                goal_pose = self.sample_random_pose()
            # goal_pose.position.z_val = start_pose.position.z_val

            # # Goal pose for testing
            # goal_pose = airgen.Pose(
            #     airgen.Vector3r(
            #         118.9158593816289,
            #         102.44622511764848,
            #         -1.8578945398330688,
            #     ),
            #     airgen.Quaternionr(airgen.Vector3r(0, 0, 0)),
            # )
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

    def plan_and_move(self):
        """
        Plan the path for a drone flying with a ~constant velocity of 5m/s.
        The velocity is reduced as the target approaches and the
        breaking distance is decided based on the drone velocity.
        https://github.com/Microsoft/AirSim/blob/main/docs/apis.md#drivetrain

        The default implementation is in the find_path function.
        TODO: File I/O of the voxel grid is an unnecessary overhead
        """
        start_pose, goal_pose = self.spawn_poses()
        print(f"Moving to goal position: {goal_pose.position}")
        move_planar = False  # move on the same XY plane

        while start_pose.position.distance_to(goal_pose.position) > DIST_THRESH:
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
                HORIZON_LEN * SCALE,
                HORIZON_LEN * SCALE,
                HORIZON_LEN * SCALE,
                SCALE,
                MAP_LOCATION,
            )
            with open(MAP_LOCATION, "rb") as f:
                occ_grid = binvox_rw.read_as_3d_array(f)

            # Transform coordinates to occupancy grid frame
            # For now assume that the drone is at the center of the occcupancy grid
            drone_in_occ_grid = airgen.Vector3r(
                HORIZON_LEN // 2,
                HORIZON_LEN // 2,
                HORIZON_LEN // 2,
            )
            trans_coords = TransformCoordinates(
                drone_in_occ_grid=copy(drone_in_occ_grid),
                drone_in_world=copy(start_pose.position),
            )
            goal_in_occ_grid = trans_coords.world_to_occ_grid([copy(interrim_goal_vec)])

            # Find a valid path
            trajectory_in_occ_grid = self.find_path(
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

                # Find a valid pathconnect_airgen
                trajectory_in_occ_grid = self.find_path(
                    copy(drone_in_occ_grid), copy(goal_in_occ_grid[0]), occ_grid
                )

            # Transform coordinates back to world frame
            trajectory_in_world = trans_coords.occ_grid_to_world(
                copy(trajectory_in_occ_grid)
            )

            # Move the drone along the planned path at a velocity of 5 m/s
            velocity = 5.0
            self.drone_client.moveOnPathAsync(
                trajectory_in_world,
                velocity,
                120,
                airgen.DrivetrainType.MaxDegreeOfFreedom,
                airgen.YawMode(False, 0),
                -1,
                0,
            ).join()

            # Get the pose at the end of plan traversal
            start_pose = self.drone_client.simGetVehiclePose()

        print(f"##### Destination reached {goal_pose.position}")

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
        if SCALE == 1:
            for i in range((HORIZON_LEN // 2) - 1, (HORIZON_LEN // 2) + 2):
                for j in range((HORIZON_LEN // 2) - 1, (HORIZON_LEN // 2) + 2):
                    for k in range((HORIZON_LEN // 2) - 1, (HORIZON_LEN // 2) + 2):
                        occ_grid_obj.array[i][j][k] = 0
        else:
            occ_grid_obj.array[HORIZON_LEN // 2][HORIZON_LEN // 2][HORIZON_LEN // 2] = 0

        # Call the c code
        self.planner(
            self.array_type(start_pose.x_val, start_pose.y_val, start_pose.z_val),
            self.array_type(
                int(goal_pose.x_val), int(goal_pose.y_val), int(goal_pose.z_val)
            ),
            ctypes.byref(path),
            ctypes.byref(occ_grid_obj),
        )
        return self.process_path(path)

    def process_path(self, path):
        path_len = path.path_len
        path = np.ndarray((path_len, 3), "f", path.array, order="C")
        vector_path = []
        for i in range(path_len):
            vector_path.append(airgen.Vector3r(*path[path_len - i - 1][:]))
        return vector_path

import sys
import airgen
import numpy as np
import cmath
import abc

sys.path.append("/home/local/ASUAD/opatil3/src/drone_path_planning")
import binvox_rw
from constants import MAP_LOCATION, HORIZON_LEN, PLAN_FREQ, DIST_THRESH
from utils import TransformCoordinates


class DroneController(metaclass=abc.ABCMeta):
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
        # print("Nav mesh info: {}".format(self.nav_mesh_info))

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

    def plan_and_move(self):
        """
        Plan the path for a drone flying with a ~constant velocity of 5m/s.
        The velocity is reduced as the target approaches and the
        breaking distance is decided based on the drone velocity.
        https://github.com/Microsoft/AirSim/blob/main/docs/apis.md#drivetrain

        The default implementation is in the find_path function.
        TODO: File I/O of the voxel grid is an unnecessary overhead
        """

        # Get current pose of the drone in 6-DOF
        start_pose = self.drone_client.simGetVehiclePose()

        # Sample a random valid pose in the environment
        goal_pose = self.sample_random_pose()
        while goal_pose.position.z_val > 0:
            goal_pose = self.sample_random_pose()
        # Goal pose for testing
        # goal_pose = airgen.Pose(
        #     airgen.Vector3r(0.0, 0.0, -15.8605295419692993),
        #     airgen.Quaternionr(airgen.Vector3r(0, 0, 0)),
        # )

        print(f"Moving to goal position: {goal_pose.position}")
        while start_pose.position.distance_to(goal_pose.position) > DIST_THRESH:
            pose_vec = goal_pose.position - start_pose.position
            x_vec = pose_vec.x_val
            y_vec = pose_vec.y_val
            z_vec = pose_vec.z_val

            # Get heading towards the final goal
            r_xy, theta = cmath.polar(complex(x_vec, y_vec))
            rS, phi = cmath.polar(complex(r_xy, z_vec))
            if rS > PLAN_FREQ:
                rS = PLAN_FREQ

            # local planning
            get_coords = lambda cnum: (int(cnum.real), int(cnum.imag))
            goal_xy, goal_z = get_coords(cmath.rect(rS, phi))
            goal_x, goal_y = get_coords(cmath.rect(goal_xy, theta))

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
                start_pose, HORIZON_LEN, HORIZON_LEN, HORIZON_LEN, 1, MAP_LOCATION
            )
            with open(MAP_LOCATION, "rb") as f:
                occ_grid = binvox_rw.read_as_3d_array(f)

            # Transform coordinates to occupancy grid frame
            # For now assume that the drone is at the center of the occcupancy grid
            drone_in_occ_grid = airgen.Vector3r(
                HORIZON_LEN // 2, HORIZON_LEN // 2, HORIZON_LEN // 2
            )
            trans_coords = TransformCoordinates(
                drone_in_occ_grid=drone_in_occ_grid, drone_in_world=start_pose.position
            )
            goal_in_occ_grid = trans_coords.world_to_occ_grid([interrim_goal_vec])

            # Find a valid path
            trajectory_in_occ_grid = self.find_path(
                drone_in_occ_grid, goal_in_occ_grid[0], occ_grid
            )

            # Transform coordinates back to world frame
            trajectory_in_world = trans_coords.occ_grid_to_world(trajectory_in_occ_grid)

            # Move the drone along the planned path at a velocity of 5 m/s
            velocity = 10.0
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
        print(
            f"##### Destination reached: {start_pose.position} | Target: {goal_pose.position}"
        )

    @abc.abstractmethod
    def find_path(self, start_pose, goal_pose, occ_grid):
        """
        Override this function by implementing your path planning algorithm
        Parameters: start_pose (in occ_grid frame), goal_pose (in occ_grid frame), occupancy_grid
        Returns: trajectory (list of waypoints)
        """
        # # Compute a collision-free path between start and goal
        # trajectory = self.drone_client.simPlanPath(
        #     start_pose.position, goal_pose.position, True, True
        # )
        # # always check the length of the trajectory before moving
        # # if len(trajectory) < 2:
        # #     trajectory = self.drone_client.simPlanPathToRandomizeGoal(
        # #         start_pose.position, goal_pose.position, PLAN_FREQ, 1, True, True
        # #     )
        # return trajectory
        pass


# # Plan path to random point
# controller = DroneController()
# try:
#     controller.plan_and_move()
# except KeyboardInterrupt:
#     print("Keyboard interrupt detected, exiting...")
#     client = airgen.MultirotorClient()
#     client.reset()

MAP_LOCATION = "/home/local/ASUAD/opatil3/Simulators/map.binvox"
PLAN_FREQ = 10  # in meters. Should be <= HORIZON_LEN//2
HORIZON_LEN = 20  # in meters. Keep in the form of 2x+1, preferrably small.
DIST_THRESH = 3  # Maximum distance from the goal to be considered successful
SCALE = 2 # Scale of voxel in the occupancy grid relative to the simulator world

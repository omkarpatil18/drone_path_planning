#ifndef _header_h
#define _header_h

#define PLAN_FREQ 4
#define HORIZON_LEN 10
// Define a structure to represent a 3D point
typedef struct
{
    float x, y, z;
} Point3D;

// Define a structure to represent the path found
typedef struct
{
    int path_len;
    float array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN][3];
} Path;

// Define a structure to represent the occupancy grid
typedef struct
{
    float array[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
} OccupancyGrid;

// Function to calculate Euclidean distance between two points
float euclideanDistance(Point3D a, Point3D b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// Manhattan distance
static float manhattanDistance(Point3D src, Point3D dst)
{
    float x_delta, y_delta, z_delta;
    // to avoid needing abs(), subtract the smaller from the greater
    x_delta = (src.x > dst.x) ? (src.x - dst.x) : (dst.x - src.x);
    y_delta = (src.y > dst.y) ? (src.y - dst.y) : (dst.y - src.y);
    z_delta = (src.z > dst.z) ? (src.z - dst.z) : (dst.z - src.z);

    return x_delta + y_delta + z_delta;
}

#endif
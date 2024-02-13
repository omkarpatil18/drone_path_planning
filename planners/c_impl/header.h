#ifndef _header_h
#define _header_h

#define PLAN_FREQ 10
#define HORIZON_LEN 21
// Define a structure to represent a 3D point
typedef struct
{
    float x, y, z;
} Point3D;

// Define a structure to represent the path found
typedef struct
{
    float array[PLAN_FREQ][3];
} Path;

// Define a structure to represent the occupancy grid
typedef struct
{
    float array[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
} OccupancyGrid;

#endif
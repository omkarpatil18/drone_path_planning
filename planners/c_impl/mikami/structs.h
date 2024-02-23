
#ifndef _structs_h
#define _structs_h

#include "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/header.h"

typedef struct LineX LineX;
typedef struct LineY LineY;
typedef struct LineZ LineZ;

// X Line implementation
struct LineX
{
    LineY *parentLineY;
    LineZ *parentLineZ;
    int x, y, z;
    int xStart, xEnd;
};

// Y Line implementation
struct LineY
{
    LineX *parentLineX;
    LineZ *parentLineZ;
    int x, y, z;
    int yStart, yEnd;
};

// Z Line implementation
struct LineZ
{
    LineX *parentLineX;
    LineY *parentLineY;
    int x, y, z;
    int zStart, zEnd;
};

// X LineSet implementation
typedef struct
{
    LineX array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LineSetX;

// Y LineSet implementation
typedef struct
{
    LineY array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LineSetY;

// Z LineSet implementation
typedef struct
{
    LineZ array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LineSetZ;

// Struct for storing the set of obstacles
typedef struct
{
    Point3D array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int size;
} ObstacleArray;

// Struct for storing intersections
typedef struct
{
    Point3D p3D;
    LineX *lineX;
    LineY *lineY;
    LineZ *lineZ;
} Intersection;

// Struct for storing path to src or dst
typedef struct
{
    float array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN][3];
    int size;
} PathFromIntersection;

#endif
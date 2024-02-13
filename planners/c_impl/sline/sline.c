#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include </home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/header.h>

// Function to calculate the straight line path between two points
Point3D *straight_line_path(Point3D start, Point3D end, Path *st_path, int num_segments)
{

    // Calculate vector from start to end
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dz = end.z - start.z;

    // Calculate distance between start and end
    float distance = sqrt(dx * dx + dy * dy + dz * dz);

    // Calculate step size
    float step_size = distance / num_segments;

    // Add start point to path
    st_path->array[0][0] = start.x;
    st_path->array[0][1] = start.y;
    st_path->array[0][2] = start.z;

    // Iterate over segments
    for (int i = 1; i <= num_segments; i++)
    {
        // Calculate position of segment point
        float t = i * step_size / distance;
        st_path->array[i][0] = start.x + dx * t;
        st_path->array[i][1] = start.y + dy * t;
        st_path->array[i][2] = start.z + dz * t;
    }
}

extern void planner(float start[3], float end[3], Path *st_path, OccupancyGrid *occ_grid)
{
    // Define start and end points
    Point3D start_3d = {start[0], start[1], start[2]};
    Point3D end_3d = {end[0], end[1], end[2]};

    // Calculate straight line path
    straight_line_path(start_3d, end_3d, st_path, PLAN_FREQ);
}
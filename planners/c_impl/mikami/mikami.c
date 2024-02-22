#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/mikami/structs.h"

void trimLineX(LineX *, ObstacleArray *);
void trimLineY(LineY *, ObstacleArray *);
void trimLineZ(LineZ *, ObstacleArray *);
void backTraceX(LineX *, PathFromIntersection *);
void backTraceY(LineY *, PathFromIntersection *);
void backTraceZ(LineZ *, PathFromIntersection *);
void freeMemory(OccupancyGrid *, ObstacleArray *, LineSetX *, LineSetX *, LineSetY *, LineSetY *, LineSetZ *, LineSetZ *, PathFromIntersection *, PathFromIntersection *, Intersection *);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Create initial linesets from src/dst ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to create a new lineSet for X
LineSetX *createLineSetX(Point3D p3D, ObstacleArray *obsArray)
{
    LineSetX *lineSet = (LineSetX *)malloc(sizeof(LineSetX));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].xStart = 0;
    lineSet->array[0].xEnd = HORIZON_LEN - 1;
    trimLineX(&lineSet->array[0], obsArray);
    return lineSet;
}

// Function to create a new lineSet for Y
LineSetY *createLineSetY(Point3D p3D, ObstacleArray *obsArray)
{
    LineSetY *lineSet = (LineSetY *)malloc(sizeof(LineSetY));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].yStart = 0;
    lineSet->array[0].yEnd = HORIZON_LEN - 1;
    trimLineY(&lineSet->array[0], obsArray);
    return lineSet;
}

// Function to create a new lineSet for Z
LineSetZ *createLineSetZ(Point3D p3D, ObstacleArray *obsArray)
{
    LineSetZ *lineSet = (LineSetZ *)malloc(sizeof(LineSetZ));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].zStart = 0;
    lineSet->array[0].zEnd = HORIZON_LEN - 1;
    trimLineZ(&lineSet->array[0], obsArray);
    return lineSet;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Obstacle management and collision checking ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to trim the line X based on obstacles
void trimLineX(LineX *lineX, ObstacleArray *obsArray)
{
    for (int i = 0; i < obsArray->size; i++)
    {
        Point3D obs = obsArray->array[i];
        if (obs.y == lineX->y && obs.z == lineX->z)
        {
            if (obs.x > lineX->x)
            {
                if (obs.x < lineX->xEnd)
                {
                    lineX->xEnd = obs.x;
                }
            }
            else
            {
                if (obs.x > lineX->xStart)
                {
                    lineX->xStart = obs.x;
                }
            }
        }
    }
}

// Function to trim the line Y based on obstacles
void trimLineY(LineY *lineY, ObstacleArray *obsArray)
{
    for (int i = 0; i < obsArray->size; i++)
    {
        Point3D obs = obsArray->array[i];
        if (obs.x == lineY->x && obs.z == lineY->z)
        {
            if (obs.y > lineY->y)
            {
                if (obs.y < lineY->yEnd)
                {
                    lineY->yEnd = obs.y;
                }
            }
            else
            {
                if (obs.y > lineY->yStart)
                {
                    lineY->yStart = obs.y;
                }
            }
        }
    }
}

// Function to trim the line Z based on obstacles
void trimLineZ(LineZ *lineZ, ObstacleArray *obsArray)
{
    for (int i = 0; i < obsArray->size; i++)
    {
        Point3D obs = obsArray->array[i];
        if (obs.y == lineZ->y && obs.x == lineZ->x)
        {
            if (obs.z > lineZ->z)
            {
                if (obs.z < lineZ->zEnd)
                {
                    lineZ->zEnd = obs.z;
                }
            }
            else
            {
                if (obs.z > lineZ->zStart)
                {
                    lineZ->zStart = obs.z;
                }
            }
        }
    }
}

// Function to check if a point is within the occGrid bounds
bool isValidPoint(Point3D point)
{
    return (point.x >= 0 && point.x < HORIZON_LEN &&
            point.y >= 0 && point.y < HORIZON_LEN &&
            point.z >= 0 && point.z < HORIZON_LEN);
}

// Function to check if a point is passable (not an obstacle)
bool isPassable(OccupancyGrid *occGrid, Point3D point)
{
    return (isValidPoint(point) && (occGrid->array)[(int)point.x][(int)point.y][(int)point.z] == 0.0);
}

// Function to check if a point is the dst
bool isGoal(Point3D point, Point3D dst)
{
    return (point.x == dst.x && point.y == dst.y && point.z == dst.z);
}

// Generate the set of obstacles from the occupancy grid
ObstacleArray *generateObstacleSet(OccupancyGrid *occGrid)
{
    ObstacleArray *obsArray = (ObstacleArray *)malloc(sizeof(ObstacleArray));
    obsArray->size = 0;
    for (int i = 0; i < HORIZON_LEN; i++)
    {
        for (int j = 0; j < HORIZON_LEN; j++)
        {
            for (int k = 0; k < HORIZON_LEN; k++)
            {
                if (occGrid->array[i][j][k] == 1.0)
                {
                    int index = obsArray->size;
                    Point3D p3D = {i, j, k};
                    obsArray->array[index] = p3D;
                    obsArray->size++;
                }
            }
        }
    }
    return obsArray;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Check for intersection b/w lines //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to check if a number lies within a closed range
bool liesWithin(int x, int start, int end)
{
    if (x <= end && x >= start)
    {
        return true;
    }
    return false;
}

// Function to find if two XY lines intersect
Point3D lineIntersectionXY(LineX lineX, LineY lineY)
{
    Point3D p3D = {-1, -1, -1};
    if (lineX.z == lineY.z)
    {
        if (liesWithin(lineX.y, lineY.yStart, lineY.yEnd) && liesWithin(lineY.x, lineX.xStart, lineX.xEnd))
        {
            p3D.x = lineY.x;
            p3D.y = lineX.y;
            p3D.z = lineX.z;
        }
    }
    return p3D;
}

// Function to find if two XZ lines intersect
Point3D lineIntersectionXZ(LineX lineX, LineZ lineZ)
{
    Point3D p3D = {-1, -1, -1};
    if (lineX.y == lineZ.y)
    {
        if (liesWithin(lineX.z, lineZ.zStart, lineZ.zEnd) && liesWithin(lineZ.x, lineX.xStart, lineX.xEnd))
        {

            p3D.x = lineZ.x;
            p3D.y = lineX.y;
            p3D.z = lineX.z;
        }
    }
    return p3D;
}

// Function to find if two ZY lines intersect
Point3D lineIntersectionZY(LineZ lineZ, LineY lineY)
{
    Point3D p3D = {-1, -1, -1};
    if (lineZ.x == lineY.x)
    {
        if (liesWithin(lineZ.y, lineY.yStart, lineY.yEnd) && liesWithin(lineY.z, lineZ.zStart, lineZ.zEnd))
        {
            p3D.x = lineY.x;
            p3D.y = lineZ.y;
            p3D.z = lineY.z;
        }
    }
    return p3D;
}

// Get intersection of lineSet X and lineSet Y
Intersection *lineSetIntersectionXY(LineSetX *lineSetX, LineSetY *lineSetY)
{
    for (int i = lineSetX->size - 1; i >= 0; i--)
    {
        for (int j = lineSetY->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionXY(lineSetX->array[i], lineSetY->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = &lineSetX->array[i];
                interS->lineY = &lineSetY->array[j];
                interS->lineZ = NULL;
                return interS;
            }
        }
    }
    return NULL;
}

// Get intersection of lineSet X and lineSet Z
Intersection *lineSetIntersectionXZ(LineSetX *lineSetX, LineSetZ *lineSetZ)
{
    for (int i = lineSetX->size - 1; i >= 0; i--)
    {
        for (int j = lineSetZ->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionXZ(lineSetX->array[i], lineSetZ->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = &lineSetX->array[i];
                interS->lineY = NULL;
                interS->lineZ = &lineSetZ->array[j];
                return interS;
            }
        }
    }
    return NULL;
}

// Get intersection of lineSet Z and lineSet Y
Intersection *lineSetIntersectionZY(LineSetZ *lineSetZ, LineSetY *lineSetY)
{
    for (int i = lineSetZ->size - 1; i >= 0; i--)
    {
        for (int j = lineSetY->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionZY(lineSetZ->array[i], lineSetY->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = NULL;
                interS->lineY = &lineSetY->array[j];
                interS->lineZ = &lineSetZ->array[i];
                return interS;
            }
        }
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Methods to backtrace and store the path //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Back trace from the point of intersection on X
void backTraceX(LineX *lineX, PathFromIntersection *path)
{
    path->array[path->size][0] = lineX->x;
    path->array[path->size][1] = lineX->y;
    path->array[path->size][2] = lineX->z;
    path->size++;

    if (lineX->parentLineY != NULL)
    {
        backTraceY(lineX->parentLineY, path);
    }
    else if (lineX->parentLineZ != NULL)
    {
        backTraceZ(lineX->parentLineZ, path);
    }
    else
    {
        printf("Src or dst reached while backtrackking!\n");
    }
}

// Back trace from the point of intersection on Y
void backTraceY(LineY *lineY, PathFromIntersection *path)
{
    path->array[path->size][0] = lineY->x;
    path->array[path->size][1] = lineY->y;
    path->array[path->size][2] = lineY->z;
    path->size++;

    if (lineY->parentLineX != NULL)
    {
        backTraceX(lineY->parentLineX, path);
    }
    else if (lineY->parentLineZ != NULL)
    {
        backTraceZ(lineY->parentLineZ, path);
    }
    else
    {
        printf("Src or dst reached while backtrackking!\n");
    }
}

// Back trace from the point of intersection on Z
void backTraceZ(LineZ *lineZ, PathFromIntersection *path)
{
    path->array[path->size][0] = lineZ->x;
    path->array[path->size][1] = lineZ->y;
    path->array[path->size][2] = lineZ->z;
    path->size++;

    if (lineZ->parentLineY != NULL)
    {
        backTraceY(lineZ->parentLineY, path);
    }
    else if (lineZ->parentLineX != NULL)
    {
        backTraceX(lineZ->parentLineX, path);
    }
    else
    {
        printf("Src or dst reached while backtrackking!\n");
    }
}

void setPath(PathFromIntersection *pathToSrc, Point3D interP, PathFromIntersection *pathToDst, Path *path)
{
    // Add the path to src in reverse
    for (int i = pathToSrc->size - 1; i >= 0; i--)
    {
        path->array[path->path_len][0] = pathToSrc->array[i][0];
        path->array[path->path_len][1] = pathToSrc->array[i][1];
        path->array[path->path_len][2] = pathToSrc->array[i][2];
        path->path_len++;
    }
    // Add the intersection point
    path->array[path->path_len][0] = interP.x;
    path->array[path->path_len][1] = interP.y;
    path->array[path->path_len][2] = interP.z;
    path->path_len++;

    // Add the path to dst
    for (int i = 0; i < pathToDst->size; i++)
    {
        path->array[path->path_len][0] = pathToDst->array[i][0];
        path->array[path->path_len][1] = pathToDst->array[i][1];
        path->array[path->path_len][2] = pathToDst->array[i][2];
        path->path_len++;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Generate lines for each level /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LineX createLineX(int x, int y, int z, ObstacleArray *obsArray)
{
    LineX lineX;
    lineX.x = x;
    lineX.y = y;
    lineX.z = z;
    lineX.xStart = 0;
    lineX.xEnd = HORIZON_LEN - 1;
    trimLineX(&lineX, obsArray);
    return lineX;
}

LineY createLineY(int x, int y, int z, ObstacleArray *obsArray)
{
    LineY lineY;
    lineY.x = x;
    lineY.y = y;
    lineY.z = z;
    lineY.yStart = 0;
    lineY.yEnd = HORIZON_LEN - 1;
    trimLineY(&lineY, obsArray);
    return lineY;
}

LineZ createLineZ(int x, int y, int z, ObstacleArray *obsArray)
{
    LineZ lineZ;
    lineZ.x = x;
    lineZ.y = y;
    lineZ.z = z;
    lineZ.zStart = 0;
    lineZ.zEnd = HORIZON_LEN - 1;
    trimLineZ(&lineZ, obsArray);
    return lineZ;
}

void spawnLines(LineSetX *lineSetX, LineSetY *lineSetY, LineSetZ *lineSetZ, ObstacleArray *obsArray)
{
    int lineSetXSize = lineSetX->size;
    int lineSetYSize = lineSetY->size;
    int lineSetZSize = lineSetZ->size;

    for (int i = lineSetX->prevSize; i < lineSetXSize; i++)
    {
        LineX lineX = lineSetX->array[i];
        for (int j = lineX.xStart; j <= lineX.xEnd; j++)
        {
            if (j != lineX.x)
            {
                LineY lineY = createLineY(j, lineX.y, lineX.z, obsArray);
                lineY.parentLineX = &lineSetX->array[i];
                lineY.parentLineZ = NULL;
                lineSetY->array[lineSetY->size] = lineY;
                lineSetY->size++;

                LineZ lineZ = createLineZ(j, lineX.y, lineX.z, obsArray);
                lineZ.parentLineX = &lineSetX->array[i];
                lineZ.parentLineY = NULL;
                lineSetZ->array[lineSetZ->size] = lineZ;
                lineSetZ->size++;
            }
        }
    }

    for (int i = lineSetY->prevSize; i < lineSetYSize; i++)
    {
        LineY lineY = lineSetY->array[i];
        for (int j = lineY.yStart; j <= lineY.yEnd; j++)
        {
            if (j != lineY.y)
            {
                LineX lineX = createLineX(lineY.x, j, lineY.z, obsArray);
                lineX.parentLineY = &lineSetY->array[i];
                lineX.parentLineZ = NULL;
                lineSetX->array[lineSetX->size] = lineX;
                lineSetX->size++;

                LineZ lineZ = createLineZ(lineY.x, j, lineY.z, obsArray);
                lineZ.parentLineY = &lineSetY->array[i];
                lineZ.parentLineX = NULL;
                lineSetZ->array[lineSetZ->size] = lineZ;
                lineSetZ->size++;
            }
        }
    }

    for (int i = lineSetZ->prevSize; i < lineSetZSize; i++)
    {
        LineZ lineZ = lineSetZ->array[i];
        for (int j = lineZ.zStart; j <= lineZ.zEnd; j++)
        {
            if (j != lineZ.z)
            {
                LineY lineY = createLineY(lineZ.x, lineZ.y, j, obsArray);
                lineY.parentLineZ = &lineSetZ->array[i];
                lineY.parentLineX = NULL;
                lineSetY->array[lineSetY->size] = lineY;
                lineSetY->size++;

                LineX lineX = createLineX(lineZ.x, lineZ.y, j, obsArray);
                lineX.parentLineZ = &lineSetZ->array[i];
                lineX.parentLineY = NULL;
                lineSetX->array[lineSetX->size] = lineX;
                lineSetX->size++;
            }
        }
    }

    lineSetX->prevSize = lineSetXSize;
    lineSetY->prevSize = lineSetYSize;
    lineSetZ->prevSize = lineSetZSize;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Main method ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mikami Tabuchi algorithm implementation
void *mikamiTabuchi(Point3D start, Point3D dst, Path *path, OccupancyGrid *occGrid)
{
    if (!isValidPoint(start) || !isValidPoint(dst))
    {
        printf("Invalid start or dst point\n");
        return NULL;
    }

    if (!isPassable(occGrid, start) || !isPassable(occGrid, dst))
    {
        printf("Start or dst point is an obstacle\n");
        return NULL;
    }

    // Get obstacle array
    ObstacleArray *obsArray = generateObstacleSet(occGrid);

    printf("Generating initial lineSets\n");
    // Initializes the data structure and adds L0 lines
    LineSetX *srcLineSetX = createLineSetX(start, obsArray);
    LineSetX *dstLineSetX = createLineSetX(dst, obsArray);

    LineSetY *srcLineSetY = createLineSetY(start, obsArray);
    LineSetY *dstLineSetY = createLineSetY(dst, obsArray);

    LineSetZ *srcLineSetZ = createLineSetZ(start, obsArray);
    LineSetZ *dstLineSetZ = createLineSetZ(dst, obsArray);

    // Local data structures to store the path
    Intersection *interS = NULL;
    PathFromIntersection *pathToSrc = (PathFromIntersection *)malloc(sizeof(PathFromIntersection));
    pathToSrc->size = 0;
    PathFromIntersection *pathToDst = (PathFromIntersection *)malloc(sizeof(PathFromIntersection));
    pathToDst->size = 0;

    for (int i = 0; i < 5; i++)
    {
        int level = i;
        printf("Starting level %d\n", level);

        // Find intersections with the current set of X lines from src
        interS = lineSetIntersectionXY(srcLineSetX, dstLineSetY);
        if (interS != NULL)
        {
            printf("Found intersection in XY\n");
            backTraceX(interS->lineX, pathToSrc);
            backTraceY(interS->lineY, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        interS = lineSetIntersectionXZ(srcLineSetX, dstLineSetZ);
        if (interS != NULL)
        {
            printf("Found intersection in XZ\n");
            backTraceX(interS->lineX, pathToSrc);
            backTraceZ(interS->lineZ, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        // Find intersections with the current set of Y lines from src
        interS = lineSetIntersectionXY(dstLineSetX, srcLineSetY);
        if (interS != NULL)
        {
            printf("Found intersection in XY\n");
            backTraceX(interS->lineX, pathToDst);
            backTraceY(interS->lineY, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        interS = lineSetIntersectionZY(dstLineSetZ, srcLineSetY);
        if (interS != NULL)
        {
            printf("Found intersection in ZY\n");
            backTraceY(interS->lineY, pathToSrc);
            backTraceZ(interS->lineZ, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        // Find intersections with the current set of Z lines from src
        interS = lineSetIntersectionXZ(dstLineSetX, srcLineSetZ);
        if (interS != NULL)
        {
            printf("Found intersection in XZ\n");
            backTraceX(interS->lineX, pathToDst);
            backTraceZ(interS->lineZ, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }
        interS = lineSetIntersectionZY(srcLineSetZ, dstLineSetY);
        if (interS != NULL)
        {
            printf("Found intersection in ZY\n");
            backTraceY(interS->lineY, pathToDst);
            backTraceZ(interS->lineZ, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        // If no intersection is found, generate new points and continue the search
        printf("No intersection found in level %d!\n", i);
        if (interS == NULL)
        {
            spawnLines(srcLineSetX, srcLineSetY, srcLineSetZ, obsArray);
            spawnLines(dstLineSetX, dstLineSetY, dstLineSetZ, obsArray);
        }
    }

    if (interS == NULL)
    {
        printf("No path found.\n");
    }
    freeMemory(occGrid, obsArray, srcLineSetX, dstLineSetX, srcLineSetY, dstLineSetY, srcLineSetZ, dstLineSetZ, pathToSrc, pathToDst, interS);
    return NULL;
}

void freeMemory(OccupancyGrid *occGrid, ObstacleArray *obsArray, LineSetX *srcLineSetX, LineSetX *dstLineSetX, LineSetY *srcLineSetY, LineSetY *dstLineSetY, LineSetZ *srcLineSetZ, LineSetZ *dstLineSetZ, PathFromIntersection *pathToSrc, PathFromIntersection *pathToDst, Intersection *interS)
{
    // free(occGrid); // Uncomment while running isolated C code.
    free(obsArray);
    free(srcLineSetX);
    free(dstLineSetX);
    free(srcLineSetY);
    free(dstLineSetY);
    free(srcLineSetZ);
    free(dstLineSetZ);
    free(pathToSrc);
    free(pathToDst);
    free(interS);
}

// For debugging within C
int main()
{
    // Define start and end points
    Point3D start_3d = {0.0, 0.0, 0.0};
    Point3D end_3d = {5.0, 6.0, 5.0};
    Path *path = (Path *)malloc(sizeof(Path));

    OccupancyGrid *occGrid = (OccupancyGrid *)malloc(sizeof(OccupancyGrid));
    for (int i = 0; i < HORIZON_LEN; i++)
    {
        for (int j = 0; j < HORIZON_LEN; j++)
        {
            for (int k = 0; k < HORIZON_LEN; k++)
            {

                occGrid->array[i][j][k] = 0;
                if (i == j && j == k && i != 0)
                {
                    occGrid->array[i][j][k] = 1;
                }
            }
        }
    }
    // Calculate straight line path
    mikamiTabuchi(start_3d, end_3d, path, occGrid);
}

// Called from Python
extern void planner(float start[3], float end[3], Path *path, OccupancyGrid *occGrid)
{
    // Define start and end points
    Point3D start_3d = {start[0], start[1], start[2]};
    Point3D end_3d = {end[0], end[1], end[2]};

    // Calculate straight line path
    mikamiTabuchi(start_3d, end_3d, path, occGrid);
}
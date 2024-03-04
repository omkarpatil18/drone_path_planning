#include "/home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/mikami/structs_opt.h"

void trimLineX(LineX *, ObstacleArray *);
void trimLineY(LineY *, ObstacleArray *);
void trimLineZ(LineZ *, ObstacleArray *);
void backTraceX(LineX *, PathFromIntersection *);
void backTraceY(LineY *, PathFromIntersection *);
void backTraceZ(LineZ *, PathFromIntersection *);
void freeMemory(OccupancyGrid *, ObstacleArray *, LinePointersX *, LinePointersX *, LinePointersY *, LinePointersY *, LinePointersZ *, LinePointersZ *, PathFromIntersection *, PathFromIntersection *, Intersection *);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Create initial linesets from src/dst ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to create a new lineSet for X
LinePointersX *createLinePointersX(Point3D p3D, ObstacleArray *obsArray)
{
    LinePointersX *lineSet = (LinePointersX *)calloc(1, sizeof(LinePointersX));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].xStart = 0;
    lineSet->array[0].xEnd = HORIZON_LEN - 1;
    lineSet->array[0].parentLineY = NULL;
    lineSet->array[0].parentLineZ = NULL;
    trimLineX(&lineSet->array[0], obsArray);
    for (int i = lineSet->array[0].xStart; i <= lineSet->array[0].xEnd; i++)
    {
        lineSet->included[i][(int)p3D.y][(int)p3D.z] = true;
    }
    return lineSet;
}

// Function to create a new lineSet for Y
LinePointersY *createLinePointersY(Point3D p3D, ObstacleArray *obsArray)
{
    LinePointersY *lineSet = (LinePointersY *)calloc(1, sizeof(LinePointersY));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].yStart = 0;
    lineSet->array[0].yEnd = HORIZON_LEN - 1;
    lineSet->array[0].parentLineX = NULL;
    lineSet->array[0].parentLineZ = NULL;
    trimLineY(&lineSet->array[0], obsArray);
    for (int i = lineSet->array[0].yStart; i <= lineSet->array[0].yEnd; i++)
    {
        lineSet->included[(int)p3D.x][i][(int)p3D.z] = true;
    }
    return lineSet;
}

// Function to create a new lineSet for Z
LinePointersZ *createLinePointersZ(Point3D p3D, ObstacleArray *obsArray)
{
    LinePointersZ *lineSet = (LinePointersZ *)calloc(1, sizeof(LinePointersZ));
    lineSet->prevSize = 0;
    lineSet->size = 1;

    lineSet->array[0].x = p3D.x;
    lineSet->array[0].y = p3D.y;
    lineSet->array[0].z = p3D.z;
    lineSet->array[0].zStart = 0;
    lineSet->array[0].zEnd = HORIZON_LEN - 1;
    lineSet->array[0].parentLineY = NULL;
    lineSet->array[0].parentLineX = NULL;
    trimLineZ(&lineSet->array[0], obsArray);
    for (int i = lineSet->array[0].zStart; i <= lineSet->array[0].zEnd; i++)
    {
        lineSet->included[(int)p3D.x][(int)p3D.y][i] = true;
    }
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
Intersection *lineSetIntersectionXY(LinePointersX *linePointersX, LinePointersY *linePointersY)
{
    for (int i = linePointersX->size - 1; i >= 0; i--)
    {
        for (int j = linePointersY->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionXY(linePointersX->array[i], linePointersY->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = &linePointersX->array[i];
                interS->lineY = &linePointersY->array[j];
                interS->lineZ = NULL;
                return interS;
            }
        }
    }
    return NULL;
}

// Get intersection of lineSet X and lineSet Z
Intersection *lineSetIntersectionXZ(LinePointersX *linePointersX, LinePointersZ *linePointersZ)
{
    for (int i = linePointersX->size - 1; i >= 0; i--)
    {
        for (int j = linePointersZ->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionXZ(linePointersX->array[i], linePointersZ->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = &linePointersX->array[i];
                interS->lineY = NULL;
                interS->lineZ = &linePointersZ->array[j];
                return interS;
            }
        }
    }
    return NULL;
}

// Get intersection of lineSet Z and lineSet Y
Intersection *lineSetIntersectionZY(LinePointersZ *linePointersZ, LinePointersY *linePointersY)
{
    for (int i = linePointersZ->size - 1; i >= 0; i--)
    {
        for (int j = linePointersY->size - 1; j >= 0; j--)
        {
            Point3D p3D = lineIntersectionZY(linePointersZ->array[i], linePointersY->array[j]);
            if (p3D.x != -1 && p3D.y != -1 && p3D.z != -1)
            {
                Intersection *interS = (Intersection *)malloc(sizeof(Intersection));
                interS->p3D = p3D;
                interS->lineX = NULL;
                interS->lineY = &linePointersY->array[j];
                interS->lineZ = &linePointersZ->array[i];
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

LineX createLineX(int x, int y, int z, ObstacleArray *obsArray, LinePointersX *linePointersX)
{
    LineX lineX;
    lineX.x = x;
    lineX.y = y;
    lineX.z = z;
    lineX.xStart = 0;
    lineX.xEnd = HORIZON_LEN - 1;
    trimLineX(&lineX, obsArray);
    for (int i = lineX.xStart; i <= lineX.xEnd; i++)
    {
        linePointersX->included[i][lineX.y][lineX.z] = true;
    }
    return lineX;
}

LineY createLineY(int x, int y, int z, ObstacleArray *obsArray, LinePointersY *linePointersY)
{
    LineY lineY;
    lineY.x = x;
    lineY.y = y;
    lineY.z = z;
    lineY.yStart = 0;
    lineY.yEnd = HORIZON_LEN - 1;
    trimLineY(&lineY, obsArray);
    for (int i = lineY.yStart; i <= lineY.yEnd; i++)
    {
        linePointersY->included[lineY.x][i][lineY.z] = true;
    }
    return lineY;
}

LineZ createLineZ(int x, int y, int z, ObstacleArray *obsArray, LinePointersZ *linePointersZ)
{
    LineZ lineZ;
    lineZ.x = x;
    lineZ.y = y;
    lineZ.z = z;
    lineZ.zStart = 0;
    lineZ.zEnd = HORIZON_LEN - 1;
    trimLineZ(&lineZ, obsArray);
    for (int i = lineZ.zStart; i <= lineZ.zEnd; i++)
    {
        linePointersZ->included[lineZ.x][lineZ.y][i] = true;
    }
    return lineZ;
}

void spawnLines(LinePointersX *linePointersX, LinePointersY *linePointersY, LinePointersZ *linePointersZ, ObstacleArray *obsArray)
{
    int linePointersXSize = linePointersX->size;
    int linePointersYSize = linePointersY->size;
    int linePointersZSize = linePointersZ->size;

    for (int i = linePointersX->prevSize; i < linePointersXSize; i++)
    {
        LineX lineX = linePointersX->array[i];
        for (int j = lineX.xStart; j <= lineX.xEnd; j++)
        {
            if (!linePointersY->included[j][lineX.y][lineX.z])
            {
                LineY lineY = createLineY(j, lineX.y, lineX.z, obsArray, linePointersY);
                lineY.parentLineX = &(linePointersX->array[i]);
                lineY.parentLineZ = NULL;
                linePointersY->array[linePointersY->size] = lineY;
                linePointersY->size++;
            }
            if (!linePointersZ->included[j][lineX.y][lineX.z])
            {
                LineZ lineZ = createLineZ(j, lineX.y, lineX.z, obsArray, linePointersZ);
                lineZ.parentLineX = &(linePointersX->array[i]);
                lineZ.parentLineY = NULL;
                linePointersZ->array[linePointersZ->size] = lineZ;
                linePointersZ->size++;
            }
        }
    }

    for (int i = linePointersY->prevSize; i < linePointersYSize; i++)
    {
        LineY lineY = linePointersY->array[i];
        for (int j = lineY.yStart; j <= lineY.yEnd; j++)
        {
            if (!linePointersX->included[(int)lineY.x][j][(int)lineY.z])
            {
                LineX lineX = createLineX(lineY.x, j, lineY.z, obsArray, linePointersX);
                lineX.parentLineY = &(linePointersY->array[i]);
                lineX.parentLineZ = NULL;
                linePointersX->array[linePointersX->size] = lineX;
                linePointersX->size++;
            }
            if (!linePointersZ->included[(int)lineY.x][j][(int)lineY.z])
            {
                LineZ lineZ = createLineZ(lineY.x, j, lineY.z, obsArray, linePointersZ);
                lineZ.parentLineY = &(linePointersY->array[i]);
                lineZ.parentLineX = NULL;
                linePointersZ->array[linePointersZ->size] = lineZ;
                linePointersZ->size++;
            }
        }
    }

    for (int i = linePointersZ->prevSize; i < linePointersZSize; i++)
    {
        LineZ lineZ = linePointersZ->array[i];
        for (int j = lineZ.zStart; j <= lineZ.zEnd; j++)
        {
            if (!linePointersY->included[(int)lineZ.x][(int)lineZ.y][j])
            {
                LineY lineY = createLineY(lineZ.x, lineZ.y, j, obsArray, linePointersY);
                lineY.parentLineZ = &(linePointersZ->array[i]);
                lineY.parentLineX = NULL;
                linePointersY->array[linePointersY->size] = lineY;
                linePointersY->size++;
            }
            if (!linePointersX->included[(int)lineZ.x][(int)lineZ.y][j])
            {
                LineX lineX = createLineX(lineZ.x, lineZ.y, j, obsArray, linePointersX);
                lineX.parentLineZ = &(linePointersZ->array[i]);
                lineX.parentLineY = NULL;
                linePointersX->array[linePointersX->size] = lineX;
                linePointersX->size++;
            }
        }
    }

    linePointersX->prevSize = linePointersXSize;
    linePointersY->prevSize = linePointersYSize;
    linePointersZ->prevSize = linePointersZSize;
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
    LinePointersX *srcLinePointersX = createLinePointersX(start, obsArray);
    LinePointersX *dstLinePointersX = createLinePointersX(dst, obsArray);

    LinePointersY *srcLinePointersY = createLinePointersY(start, obsArray);
    LinePointersY *dstLinePointersY = createLinePointersY(dst, obsArray);

    LinePointersZ *srcLinePointersZ = createLinePointersZ(start, obsArray);
    LinePointersZ *dstLinePointersZ = createLinePointersZ(dst, obsArray);

    // Local data structures to store the path
    Intersection *interS = NULL;
    PathFromIntersection *pathToSrc = (PathFromIntersection *)malloc(sizeof(PathFromIntersection));
    pathToSrc->size = 0;
    PathFromIntersection *pathToDst = (PathFromIntersection *)malloc(sizeof(PathFromIntersection));
    pathToDst->size = 0;
    path->path_len = 0;

    for (int i = 0; i < 7; i++)
    {
        int level = i;
        printf("Starting level %d\n", level);

        // Find intersections with the current set of X & Y lines from src
        interS = lineSetIntersectionXY(srcLinePointersX, dstLinePointersY);
        if (interS != NULL)
        {
            printf("Found intersection in XY\n");
            backTraceX(interS->lineX, pathToSrc);
            backTraceY(interS->lineY, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        interS = lineSetIntersectionXY(dstLinePointersX, srcLinePointersY);
        if (interS != NULL)
        {
            printf("Found intersection in XY\n");
            backTraceX(interS->lineX, pathToDst);
            backTraceY(interS->lineY, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        interS = lineSetIntersectionXZ(srcLinePointersX, dstLinePointersZ);
        if (interS != NULL)
        {
            printf("Found intersection in XZ\n");
            backTraceX(interS->lineX, pathToSrc);
            backTraceZ(interS->lineZ, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        interS = lineSetIntersectionZY(dstLinePointersZ, srcLinePointersY);
        if (interS != NULL)
        {
            printf("Found intersection in ZY\n");
            backTraceY(interS->lineY, pathToSrc);
            backTraceZ(interS->lineZ, pathToDst);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        // Find intersections with the current set of Z lines from src
        interS = lineSetIntersectionXZ(dstLinePointersX, srcLinePointersZ);
        if (interS != NULL)
        {
            printf("Found intersection in XZ\n");
            backTraceX(interS->lineX, pathToDst);
            backTraceZ(interS->lineZ, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }
        interS = lineSetIntersectionZY(srcLinePointersZ, dstLinePointersY);
        if (interS != NULL)
        {
            printf("Found intersection in ZY\n");
            backTraceY(interS->lineY, pathToDst);
            backTraceZ(interS->lineZ, pathToSrc);
            setPath(pathToSrc, interS->p3D, pathToDst, path);
            freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
            return NULL;
        }

        // If no intersection is found, generate new points and continue the search
        printf("No intersection found in level %d!\n", i);
        if (interS == NULL)
        {
            spawnLines(srcLinePointersX, srcLinePointersY, srcLinePointersZ, obsArray);
            spawnLines(dstLinePointersX, dstLinePointersY, dstLinePointersZ, obsArray);
        }
    }

    if (interS == NULL)
    {
        printf("No path found.\n");
    }
    else
    {
        free(interS);
    }
    freeMemory(occGrid, obsArray, srcLinePointersX, dstLinePointersX, srcLinePointersY, dstLinePointersY, srcLinePointersZ, dstLinePointersZ, pathToSrc, pathToDst, interS);
    return NULL;
}

void freeMemory(OccupancyGrid *occGrid, ObstacleArray *obsArray, LinePointersX *srcLinePointersX, LinePointersX *dstLinePointersX, LinePointersY *srcLinePointersY, LinePointersY *dstLinePointersY, LinePointersZ *srcLinePointersZ, LinePointersZ *dstLinePointersZ, PathFromIntersection *pathToSrc, PathFromIntersection *pathToDst, Intersection *interS)
{
    // free(occGrid); // Uncomment while running isolated C code.
    free(obsArray);
    free(srcLinePointersX);
    free(dstLinePointersX);
    free(srcLinePointersY);
    free(dstLinePointersY);
    free(srcLinePointersZ);
    free(dstLinePointersZ);
    free(pathToSrc);
    free(pathToDst);
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
extern double planner(float start[3], float end[3], Path *path, OccupancyGrid *occGrid)
{
    // Define start and end points
    Point3D start_3d = {start[0], start[1], start[2]};
    Point3D end_3d = {end[0], end[1], end[2]};

    // Calculate straight line path
    clock_t beginT = clock();
    mikamiTabuchi(start_3d, end_3d, path, occGrid);
    clock_t endT = clock();
    double time_spent = (double)(endT - beginT) / CLOCKS_PER_SEC;
    return time_spent;
}
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include </home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/header.h>

// Step structure for hopping on lines
typedef struct
{
    int del_x;
    int del_y;
    int del_z;
} Step;

// Node structure for Mikami algorithm
typedef struct
{
    Point3D point;
    int level; // Level of search in Mikami
    struct Node *parent;
} Node;

// X Line implementation
typedef struct
{
    LineY *parentLine;
    LineZ *parentLine;
    int x, y, z;
    int xStart, xEnd;
} LineX;

// Y Line implementation
typedef struct
{
    LineX *parentLine;
    LineZ *parentLine;
    int x, y, z;
    int yStart, yEnd;
} LineY;

// Z Line implementation
typedef struct
{
    LineX *parentLine;
    LineY *parentLine;
    int x, y, z;
    int zStart, zEnd;
} LineZ;

// X LineSet implementation
typedef struct
{
    LineX **array;
    int capacity;
    int size;
} LineSetX;

// Y LineSet implementation
typedef struct
{
    LineY **array;
    int capacity;
    int size;
} LineSetY;

// Z LineSet implementation
typedef struct
{
    LineZ **array;
    int capacity;
    int size;
} LineSetZ;

// Struct for storing the set of obstacles
typedef struct
{
    Point3D *array;
    int size;
} ObstacleArray;

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

// Function to create a new lineSet for X
LineSetX *createLineSetX(int capacity, Point3D p3D, ObstacleArray *obsArray)
{
    LineSetX *lineSet = (LineSetX *)malloc(sizeof(LineSetX));
    lineSet->capacity = capacity;
    lineSet->size = 1;
    lineSet->array = (LineX **)malloc(capacity * sizeof(LineX *));

    LineX *lineX = lineSet->array[0];
    lineX->x = p3D.x;
    lineX->y = p3D.y;
    lineX->z = p3D.z;
    lineX->xStart = 0;
    lineX->xEnd = HORIZON_LEN - 1;
    trimLineX(lineX, obsArray);
    return lineSet;
}

// Function to create a new lineSet for Y
LineSetY *createLineSetY(int capacity, Point3D p3D, ObstacleArray *obsArray)
{
    LineSetY *lineSet = (LineSetY *)malloc(sizeof(LineSetY));
    lineSet->capacity = capacity;
    lineSet->size = 0;
    lineSet->array = (LineY **)malloc(capacity * sizeof(LineY *));

    LineY *lineY = lineSet->array[0];
    lineY->x = p3D.x;
    lineY->y = p3D.y;
    lineY->z = p3D.z;
    lineY->yStart = 0;
    lineY->yEnd = HORIZON_LEN - 1;
    trimLineY(lineY, obsArray);
    return lineSet;
}

// Function to create a new lineSet for Z
LineSetZ *createLineSetZ(int capacity, Point3D p3D, ObstacleArray *obsArray)
{
    LineSetZ *lineSet = (LineSetZ *)malloc(sizeof(LineSetZ));
    lineSet->capacity = capacity;
    lineSet->size = 0;
    lineSet->array = (LineZ **)malloc(capacity * sizeof(LineZ *));

    LineZ *lineZ = lineSet->array[0];
    lineZ->x = p3D.x;
    lineZ->y = p3D.y;
    lineZ->z = p3D.z;
    lineZ->zStart = 0;
    lineZ->zEnd = HORIZON_LEN - 1;
    trimLineZ(lineZ, obsArray);
    return lineSet;
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

// Function to check if a point is the goal
bool isGoal(Point3D point, Point3D goal)
{
    return (point.x == goal.x && point.y == goal.y && point.z == goal.z);
}

// Generate the set of obstacles from the occupancy grid
ObstacleArray *generateObstacleSet(OccupancyGrid *occGrid)
{
    ObstacleArray *obsArray = (ObstacleArray *)malloc(sizeof(ObstacleArray));
    obsArray->size = 0;
    obsArray->array = (Point3D *)malloc(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN * sizeof(Point3D *));
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
Point3D *lineIntersectionXY(LineX *lineX, LineY *lineY)
{
    if (lineX->z == lineY->z)
    {
        if (liesWithin(lineX->y, lineY->yStart, lineY->yEnd) && liesWithin(lineY->x, lineX->xStart, lineX->xEnd))
        {
            Point3D p3D = {lineY->x, lineX->y, lineX->z};
            return &p3D;
        }
    }
    return NULL;
}

// Function to find if two XZ lines intersect
Point3D *lineIntersectionXZ(LineX *lineX, LineZ *lineZ)
{
    if (lineX->y == lineZ->y)
    {
        if (liesWithin(lineX->z, lineZ->zStart, lineZ->zEnd) && liesWithin(lineZ->x, lineX->xStart, lineX->xEnd))
        {
            Point3D p3D = {lineZ->x, lineX->y, lineX->z};
            return &p3D;
        }
    }
    return NULL;
}

// Function to find if two ZY lines intersect
Point3D *lineIntersectionZY(LineZ *lineZ, LineY *lineY)
{
    if (lineZ->x == lineY->x)
    {
        if (liesWithin(lineZ->y, lineY->yStart, lineY->yEnd) && liesWithin(lineY->z, lineZ->zStart, lineZ->zEnd))
        {
            Point3D p3D = {lineY->x, lineZ->y, lineY->z};
            return &p3D;
        }
    }
    return NULL;
}

// Function to generate a line along X
LineX *getLineX(Node *node, OccupancyGrid *occGrid, int level, Step step)
{
    Line *line = createLine(HORIZON_LEN);
    for (int i = 0; i < HORIZON_LEN; i++)
    {
        Point3D nextPoint = {node->point.x + step.del_x, node->point.y + step.del_y, node->point.z + step.del_z};
        if (isValidPoint(nextPoint) && isPassable(occGrid, nextPoint))
        {
            Node *nextNode = insertPoint(line, node, nextPoint, level);
            node = nextNode;
        }
        else
        {
            break;
        }
    }
    return line;
}

// Function to perform one level of line search for two given points
Node *twoPointSearch(Node *src, Node *dst, OccupancyGrid *occGrid, int level)
{
    Step xStep = {1, 0, 0};
    Step yStep = {0, 1, 0};
    Step zStep = {0, 0, 1};

    // Get lines from src and dst
    Line *srcLineX = getLine(src, occGrid, level, xStep);
    Line *dstLineX = getLine(dst, occGrid, level, xStep);

    Line *srcLineY = getLine(src, occGrid, level, yStep);
    Line *dstLineY = getLine(dst, occGrid, level, yStep);

    Line *srcLineZ = getLine(src, occGrid, level, zStep);
    Line *dstLineZ = getLine(dst, occGrid, level, zStep);

    // Get intersection between each pair of lines (ugly code)
    Node *intersection = lineIntersection(srcLineX, dstLineY);
    if (!intersection)
    {
        Node *intersection = lineIntersection(srcLineX, dstLineZ);
    }
    if (!intersection)
    {
        Node *intersection = lineIntersection(srcLineY, dstLineX);
    }
    if (!intersection)
    {
        Node *intersection = lineIntersection(srcLineY, dstLineZ);
    }
    if (!intersection)
    {
        Node *intersection = lineIntersection(srcLineZ, dstLineX);
    }
    if (!intersection)
    {
        Node *intersection = lineIntersection(srcLineZ, dstLineY);
    }

    return intersection;
}

// Create initial set of lines from the points
void createInitialLineSet(LineSetX *linSetX, LineSetY *lineSetY, LineSetZ *lineSetZ, Point3D start, Point3D goal)
{
}

// Mikami Tabuchi algorithm implementation
Node *mikamiTabuchi(Point3D start, Point3D goal, Path *path, OccupancyGrid *occGrid)
{
    if (!isValidPoint(start) || !isValidPoint(goal))
    {
        printf("Invalid start or goal point\n");
        return NULL;
    }

    if (!isPassable(occGrid, start) || !isPassable(occGrid, goal))
    {
        printf("Start or goal point is an obstacle\n");
        return NULL;
    }

    // Initializes the data structure and adds L0 lines
    LineSetX *srcLineSetX = createLineSetX(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);
    LineSetX *dstLineSetX = createLineSetX(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);

    LineSetY *srcLineSetY = createLineSetY(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);
    LineSetY *dstLineSetY = createLineSetY(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);

    LineSetZ *srcLineSetZ = createLineSetZ(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);
    LineSetZ *dstLineSetZ = createLineSetZ(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);

    bool intersectionFound = false;
    for (int i = 0; i < 3; i++)
    {
        int level = i;

        if (intersectionFound)
        {
            break;
        }
    }

    Line *openList = createBinaryHeap(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);
    float gScore[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
    path->path_len = 0;

    for (int i = 0; i < HORIZON_LEN; i++)
    {
        for (int j = 0; j < HORIZON_LEN; j++)
        {
            for (int k = 0; k < HORIZON_LEN; k++)
            {
                gScore[i][j][k] = INFINITY;
            }
        }
    }
    gScore[(int)start.x][(int)start.y][(int)start.z] = 0;

    Node *startNode = (Node *)malloc(sizeof(Node));
    startNode->point = start;
    startNode->g = 0;
    startNode->h = euclideanDistance(start, goal);
    startNode->f = startNode->g + startNode->h;
    startNode->parent = NULL;
    insert(openList, startNode);
    bool goalReached = false;
    while (openList->size > 0)
    {
        Node *currentNode = extractMin(openList);

        if (isGoal(currentNode->point, goal))
        {
            goalReached = true;
            printf("Path found: ");
            reconstructPath(currentNode, path);
            printf("\n");
            return NULL;
        }

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    Point3D neighborPoint = {currentNode->point.x + dx, currentNode->point.y + dy, currentNode->point.z + dz};

                    if (isValidPoint(neighborPoint) && isPassable(occGrid, neighborPoint))
                    {
                        float tentativeG = currentNode->g + euclideanDistance(currentNode->point, neighborPoint);

                        if (tentativeG < gScore[(int)neighborPoint.x][(int)neighborPoint.y][(int)neighborPoint.z])
                        {
                            gScore[(int)neighborPoint.x][(int)neighborPoint.y][(int)neighborPoint.z] = tentativeG;
                            Node *neighborNode = (Node *)malloc(sizeof(Node));
                            neighborNode->point = neighborPoint;
                            neighborNode->g = tentativeG;
                            neighborNode->h = euclideanDistance(neighborPoint, goal);
                            neighborNode->f = neighborNode->g + neighborNode->h;
                            neighborNode->parent = currentNode;
                            insert(openList, neighborNode);
                        }
                    }
                }
            }
        }
    }
    if (!goalReached)
    {
        printf("No path found.\n");
    }
    return NULL;
}

extern void planner(float start[3], float end[3], Path *path, OccupancyGrid *occGrid)
{
    // Define start and end points
    Point3D start_3d = {start[0], start[1], start[2]};
    Point3D end_3d = {end[0], end[1], end[2]};

    // Calculate straight line path
    mikamiTabuchi(start_3d, end_3d, path, occGrid);
}
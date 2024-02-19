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

// Line implementation
typedef struct
{
    Node **array;
    int capacity;
    int size;
} Line;

// LineSet implementation
typedef struct
{
    Line **array;
    int capacity;
    int size;
} LineSet;

// Function to create a new line
Line *createLine(int capacity)
{
    Line *line = (Line *)malloc(sizeof(Line));
    line->capacity = capacity;
    line->size = 0;
    line->array = (Node **)malloc(capacity * sizeof(Node *));
    return line;
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

// Function to reconstruct the path from the goal node to the start node
void reconstructPath(Node *currentNode, Path *path)
{
    if (currentNode != NULL)
    {
        // Add point to path
        path->array[path->path_len][0] = currentNode->point.x;
        path->array[path->path_len][1] = currentNode->point.y;
        path->array[path->path_len][2] = currentNode->point.z;
        path->path_len++;
        reconstructPath(currentNode->parent, path);
        printf("(%d, %d, %d) -> ", (int)currentNode->point.x, (int)currentNode->point.y, (int)currentNode->point.z);
    }
}

// Function to find if the grid lines generated from two points intersect
Node *lineIntersection(Line *lineOne, Line *lineTwo)
{
    for (int i = 0; i < lineTwo->size; i++)
    {
        for (int j = 0; j < lineOne->size; j++)
        {
            if (lineTwo->array[i] == lineOne->array[j])
            {
                return lineTwo->array[i];
            }
        }
    }
    return NULL;
}

// Function to insert a point into a line
Node *insertPoint(Line *line, Node *parentNode, Point3D p, int level)
{
    if (line->size == line->capacity)
    {
        printf("Line full %d\n", (int)line->capacity);
        return;
    }

    Node *newNode = (Node *)malloc(sizeof(Node));
    newNode->level = level;
    newNode->parent = parentNode;
    newNode->point = p;

    int index = line->size;
    line->array[index] = newNode;
    line->size++;
    return newNode;
}

// Function to generate a line along the given step
Line *getLine(Node *node, OccupancyGrid *occGrid, int level, Step step)
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
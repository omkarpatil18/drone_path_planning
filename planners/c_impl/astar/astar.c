/* !! Inefficient implementation !! */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include </home/local/ASUAD/opatil3/src/drone_path_planning/planners/c_impl/header.h>

// Node structure for A* algorithm
typedef struct Node
{
    Point3D point;
    float f; // Total cost (g + h)
    float g; // Cost from start to current node
    float h; // Heuristic cost from current node to goal
    struct Node *parent;
} Node;

// Binary Heap implementation
typedef struct
{
    Node **array;
    int capacity;
    int size;
} BinaryHeap;

// Function to create a new binary heap
BinaryHeap *createBinaryHeap(int capacity)
{
    BinaryHeap *heap = (BinaryHeap *)malloc(sizeof(BinaryHeap));
    heap->capacity = capacity;
    heap->size = 0;
    heap->array = (Node **)malloc(capacity * sizeof(Node *));
    return heap;
}

// Function to swap two nodes in a binary heap
void swapNodes(Node **a, Node **b)
{
    Node *temp = *a;
    *a = *b;
    *b = temp;
}

// Function to get the index of the parent node
int getParentIndex(int index)
{
    return (index - 1) / 2;
}

// Function to get the index of the left child node
int getLeftChildIndex(int index)
{
    return 2 * index + 1;
}

// Function to get the index of the right child node
int getRightChildIndex(int index)
{
    return 2 * index + 2;
}

// Function to insert a node into a binary heap
void insert(BinaryHeap *heap, Node *node)
{
    if (heap->size == heap->capacity)
    {
        printf("Heap overflow %d\n", (int)heap->capacity);
        return;
    }

    int index = heap->size;
    heap->array[index] = node;
    heap->size++;

    // Heapify up
    while (index > 0 && heap->array[index]->f < heap->array[getParentIndex(index)]->f)
    {
        swapNodes(&heap->array[index], &heap->array[getParentIndex(index)]);
        index = getParentIndex(index);
    }
}

// Function to heapify down (used in extractMin operation)
void heapifyDown(BinaryHeap *heap, int index)
{
    int smallest = index;
    int left = getLeftChildIndex(index);
    int right = getRightChildIndex(index);

    if (left < heap->size && heap->array[left]->f < heap->array[smallest]->f)
    {
        smallest = left;
    }

    if (right < heap->size && heap->array[right]->f < heap->array[smallest]->f)
    {
        smallest = right;
    }

    if (smallest != index)
    {
        swapNodes(&heap->array[index], &heap->array[smallest]);
        heapifyDown(heap, smallest);
    }
}

// Function to extract the node with the minimum f value from a binary heap
Node *extractMin(BinaryHeap *heap)
{
    if (heap->size == 0)
    {
        printf("Heap underflow\n");
        return NULL;
    }

    Node *minNode = heap->array[0];
    heap->array[0] = heap->array[heap->size - 1];
    heap->size--;
    heapifyDown(heap, 0);
    return minNode;
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

// Function to calculate constrained Euclidean distance between two points
float constrainedEuclideanDistance(Point3D a, Point3D b, OccupancyGrid *occGrid)
{
    int del_x = b.x - a.x;
    int del_y = b.y - a.y;
    int del_z = b.z - a.z;
    Point3D ax = {a.x + del_x, a.y, a.z};
    Point3D ay = {a.x, a.y + del_y, a.z};
    Point3D az = {a.x, a.y, a.z + del_z};
    if (del_x > 0)
    {
        if (del_y > 0)
        {
            if (del_z > 0)
            {
                if (!isPassable(occGrid, ax) && !isPassable(occGrid, ay) && !isPassable(occGrid, az))
                {
                    return INFINITY;
                }
            }
            else
            {
                if (!isPassable(occGrid, ax) && !isPassable(occGrid, ay))
                {
                    return INFINITY;
                }
            }
        }
        if (del_z > 0)
        {
            if (!isPassable(occGrid, ax) && !isPassable(occGrid, az))
            {
                return INFINITY;
            }
        }
    }
    else if (del_y > 0)
    {
        if (del_z > 0)
        {
            if (!isPassable(occGrid, ay) && !isPassable(occGrid, az))
            {
                return INFINITY;
            }
        }
    }

    // Evaluates to 1
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
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

// A* algorithm implementation
Node *aStar(Point3D start, Point3D goal, Path *path, OccupancyGrid *occGrid)
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

    BinaryHeap *openList = createBinaryHeap(HORIZON_LEN * HORIZON_LEN * HORIZON_LEN);
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
                        float tentativeG = currentNode->g + constrainedEuclideanDistance(currentNode->point, neighborPoint, occGrid);

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
    aStar(start_3d, end_3d, path, occGrid);
}

extern void planner(float start[3], float end[3], Path *path, OccupancyGrid *occGrid)
{
    // Define start and end points
    Point3D start_3d = {start[0], start[1], start[2]};
    Point3D end_3d = {end[0], end[1], end[2]};

    // Calculate straight line path
    aStar(start_3d, end_3d, path, occGrid);
}
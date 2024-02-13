#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define GRID_SIZE_X 5
#define GRID_SIZE_Y 5
#define GRID_SIZE_Z 5

// Structure to represent a point in 3D space
typedef struct
{
    int x;
    int y;
    int z;
} Point3D;

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
        printf("Heap overflow\n");
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

// Function to calculate Euclidean distance between two points
float euclideanDistance(Point3D a, Point3D b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// Function to check if a point is within the grid bounds
bool isValidPoint(Point3D point)
{
    return (point.x >= 0 && point.x < GRID_SIZE_X &&
            point.y >= 0 && point.y < GRID_SIZE_Y &&
            point.z >= 0 && point.z < GRID_SIZE_Z);
}

// Function to check if a point is passable (not an obstacle)
bool isPassable(int grid[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z], Point3D point)
{
    return (isValidPoint(point) && grid[point.x][point.y][point.z] == 0);
}

// Function to check if a point is the goal
bool isGoal(Point3D point, Point3D goal)
{
    return (point.x == goal.x && point.y == goal.y && point.z == goal.z);
}

// Function to reconstruct the path from the goal node to the start node
void reconstructPath(Node *currentNode)
{
    if (currentNode != NULL)
    {
        reconstructPath(currentNode->parent);
        printf("(%d, %d, %d) -> ", currentNode->point.x, currentNode->point.y, currentNode->point.z);
    }
}

// A* algorithm implementation
Node *aStar(int grid[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z], Point3D start, Point3D goal)
{
    if (!isValidPoint(start) || !isValidPoint(goal))
    {
        printf("Invalid start or goal point\n");
        return NULL;
    }

    if (!isPassable(grid, start) || !isPassable(grid, goal))
    {
        printf("Start or goal point is an obstacle\n");
        return NULL;
    }

    BinaryHeap *openList = createBinaryHeap(GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z);
    bool closedList[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z];

    for (int i = 0; i < GRID_SIZE_X; i++)
    {
        for (int j = 0; j < GRID_SIZE_Y; j++)
        {
            for (int k = 0; k < GRID_SIZE_Z; k++)
            {
                closedList[i][j][k] = false;
            }
        }
    }

    Node *startNode = (Node *)malloc(sizeof(Node));
    startNode->point = start;
    startNode->g = 0;
    startNode->h = euclideanDistance(start, goal);
    startNode->f = startNode->g + startNode->h;
    startNode->parent = NULL;

    insert(openList, startNode);

    while (openList->size > 0)
    {
        Node *currentNode = extractMin(openList);

        if (isGoal(currentNode->point, goal))
        {
            printf("Path found: ");
            reconstructPath(currentNode);
            printf("\n");
            return currentNode;
        }

        closedList[currentNode->point.x][currentNode->point.y][currentNode->point.z] = true;

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

                    if (isValidPoint(neighborPoint) && isPassable(grid, neighborPoint))
                    {
                        float tentativeG = currentNode->g + euclideanDistance(currentNode->point, neighborPoint);

                        Node *neighborNode = (Node *)malloc(sizeof(Node));
                        neighborNode->point = neighborPoint;
                        neighborNode->g = tentativeG;
                        neighborNode->h = euclideanDistance(neighborPoint, goal);
                        neighborNode->f = neighborNode->g + neighborNode->h;
                        neighborNode->parent = currentNode;

                        if (!closedList[neighborPoint.x][neighborPoint.y][neighborPoint.z])
                        {
                            insert(openList, neighborNode);
                        }
                    }
                }
            }
        }
    }

    printf("No path found.\n");
    return NULL;
}

int main()
{
    int grid[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z] = {
        {{0, 0, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 1, 1, 1, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 0, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 1, 0, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0}, {0, 0, 1, 1, 0}, {0, 1, 1, 1, 0}, {0, 1, 0, 1, 0}, {0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0}, {0, 0, 1, 1, 1}, {0, 1, 0, 0, 0}, {0, 1, 1, 0, 0}, {0, 0, 0, 0, 0}}};

    Point3D start = {0, 0, 0};
    Point3D goal = {4, 4, 4};

    Node *result = aStar(grid, start, goal);

    if (result != NULL)
    {
        // Free memory allocated for the path
        while (result != NULL)
        {
            Node *temp = result;
            result = result->parent;
            free(temp);
        }
    }

    return 0;
}

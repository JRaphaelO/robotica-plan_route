#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

enum PointType {
    UNKNOWN = 0,
    FREE = 1,
    OBSTACLE = 3,
    ROBOT = 2,
    DESTINATION = 4,
    PATH = 5
};

typedef struct {
    int gridPositionX;
    int gridPositionY;
    int worldPositionX;
    int worldPositionY;

    PointType type;
} Point;

typedef struct NodeAstar{
    Point point;
    int gCost;
    int hCost;
    int fCost;

    NodeAstar *parent;
} Node;


Point GRID_DATA[11][11];
double robotPositionX = floor(4.520105346468555);
double roobotPositionY = floor(2.3943042799960246);
int grid[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 0, 3, 3, 3, 3, 3, 3, 3, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 2, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3};

void createGrid()
{
    for (int i = 0; i < 11; i++)
        for (int j = 0; j < 11; j++) {
            GRID_DATA[i][j].gridPositionX = i;
            GRID_DATA[i][j].gridPositionY = j;
            GRID_DATA[i][j].worldPositionX = robotPositionX - i;
            GRID_DATA[i][j].worldPositionY = roobotPositionY - j;
            GRID_DATA[i][j].type = (PointType)grid[i * 11 + j];
        }
}

void displayGrid() {
    for (int i = 10; i >= 0; i--) {
        for (int j = 0; j < 11; j++)
            cout << "(" << GRID_DATA[i][j].worldPositionX << ", " << GRID_DATA[i][j].worldPositionY << ", " << GRID_DATA[i][j].type << ") ";

        cout << endl;
    }
}

void displayNode(Node *node) {
    cout << "(" << node->point.gridPositionX << ", " << node->point.gridPositionY << ", " << node->point.worldPositionX << ", " << node->point.worldPositionY << ", " << node->point.type << ", " << node->fCost << ", " << node->gCost << ", " << node->hCost << ") ";
    if (node->parent) {
        cout << " Parent: ";
        displayNode(node->parent);
    }
}

vector<Node*> aStar() {
    Node *startNode = new Node();
    startNode->point = GRID_DATA[5][5];
    startNode->gCost = 0;
    startNode->hCost = 0;
    startNode->fCost = 0;
    startNode->parent = NULL;

    Node *destinationNode = new Node();
    destinationNode->point.worldPositionX = 3;
    destinationNode->point.worldPositionY = 3;
    destinationNode->point.worldPositionX = robotPositionX - 3;
    destinationNode->point.worldPositionY = roobotPositionY - 3;
    destinationNode->point.type = DESTINATION;
    destinationNode->gCost = 0;
    destinationNode->hCost = 0;
    destinationNode->fCost = 0;
    destinationNode->parent = NULL;

    vector<Node*> openList;
    vector<Node*> closedList;
    vector<Node*> path;

    openList.push_back(startNode);

    while (openList.size() > 0) {
        Node *currentNode = openList[0];

        displayNode(currentNode);
        int currentIndex = 0;

        cout << "Size list: " << openList.size() << endl;
        for (int i = 0; i < openList.size(); i++) {
            if (openList[i]->fCost < currentNode->fCost) {
                currentNode = openList[i];
                currentIndex = i;
                displayNode(currentNode);
                cout << "Swap current node" << endl;
            }
        }

        openList.erase(openList.begin() + currentIndex);
        closedList.push_back(currentNode);
        cout << "Size Open List: " << openList.size() << endl;
        cout << "Size Closed List: " << closedList.size() << endl;

        if (currentNode->point.worldPositionX == destinationNode->point.worldPositionX && currentNode->point.worldPositionY == destinationNode->point.worldPositionY) {
            cout << "Found!" << endl;

            path = vector<Node*>();
            Node *current = currentNode;

            while (current != NULL) {
                path.push_back(current);
                current = current->parent;
            }

            return path;
        }

        vector<Node*> children;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++)
            {
                if ((i == 0 && j == 0) || (i == -1 && j == -1) || (i == -1 && j == 1) || (i == 1 && j == -1) || (i == 1 && j == 1))
                    continue;

                int x = currentNode->point.gridPositionX + i;
                int y = currentNode->point.gridPositionY + j;

                if (x < 0 || x >= 11 || y < 0 || y >= 11)
                    continue;

                if (GRID_DATA[x][y].type == OBSTACLE) {
                    cout << "Obstacle" << endl;
                    continue;
                }

                cout << "Free" << endl;
                Node *newNode = new Node();
                newNode->point = GRID_DATA[x][y];
                newNode->parent = currentNode;
                children.push_back(newNode);
            }
        }

        for (int i = 0; i < children.size(); i++) {
            bool skip = false;
            displayNode(children[i]);
            for (int j = 0; j < closedList.size(); j++) {
                if (children[i]->point.worldPositionX == closedList[j]->point.worldPositionX && children[i]->point.worldPositionY == closedList[j]->point.worldPositionY) {
                    skip = true;
                    cout << "In closed list" << endl;
                    break;
                }
            }

            if (skip)
                continue;
            
            children[i]->gCost = currentNode->gCost + 1;
            children[i]->hCost = sqrt(pow(children[i]->point.worldPositionX - destinationNode->point.worldPositionX, 2) + pow(children[i]->point.worldPositionY - destinationNode->point.worldPositionY, 2));
            children[i]->fCost = children[i]->gCost + children[i]->hCost;

            for (int j = 0; j < openList.size(); j++) {
                if (children[i]->point.worldPositionX == openList[j]->point.worldPositionX && children[i]->point.worldPositionY == openList[j]->point.worldPositionY && children[i]->gCost > openList[j]->gCost) {
                    skip = true;
                    cout << "In open list" << endl;
                    break;
                }
            }

            if (skip)
                continue;

            cout << " Add to open list: ";
            displayNode(children[i]);
            cout << endl;
            openList.push_back(children[i]);
        }
    }

    return path;
}

int main()
{
    cout << "Hello World!" << endl;
    createGrid();
    displayGrid();
    vector<Node*> path = aStar();

    cout << endl << "Path: ";
    for (int i = path.size() - 1; i >= 0 ; i--) {
        cout << i + 1 << " -> ";
        displayNode(path[i]);
        cout << endl;
    }
    return 0;
}
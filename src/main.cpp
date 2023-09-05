#include <iostream>
#include <cmath>
#include <vector>

// Ros includes
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

enum PointType {
    FREE = 0,
    OBSTACLE = 1,
    ROBOT = 2,
    DESTINATION = 3,
    PATH = 4
};

typedef struct {
    int gridPositionX;
    int gridPositionY;

    PointType type;
} Point;

typedef struct NodeAstar{
    Point point;
    int gCost;
    int hCost;
    int fCost;

    NodeAstar *parent;
} Node;

Point GRID_DATA[33][33];
double robotGridPositionX = 17;
double robotGridPositionY = 17;
double robotPositionX;
double robotPositionY;
bool robotGridReceived = false;
bool robotPositionReceived = false;
bool findPath = false;
Node *destinationNode;

void displayGrid()
{
    for (int i = 0; i < 33; i++) {
        for (int j = 0; j < 33; j++)
            cout << GRID_DATA[i][j].type << " ";

        cout << endl;
    }
}


void displayNode(Node *node) {
    cout << "(" << node->point.gridPositionX << ", " << node->point.gridPositionY << ", " << node->point.type << ", " << node->fCost << ", " << node->gCost << ", " << node->hCost << ") ";
    if (node->parent) {
        cout << " Parent: ";
        displayNode(node->parent);
    }
}


vector<Node*> aStar() {
    Node *startNode = new Node();
    startNode->point = GRID_DATA[17][17];
    startNode->gCost = 0;
    startNode->hCost = 0;
    startNode->fCost = 0;
    startNode->parent = NULL;

    if (!findPath)
        return vector<Node*>();
    
    vector<Node*> openList;
    vector<Node*> closedList;
    vector<Node*> path;

    openList.push_back(startNode);

    while (openList.size() > 0) {
        Node *currentNode = openList[0];

        int currentIndex = 0;

        for (int i = 0; i < openList.size(); i++) {
            if (openList[i]->fCost < currentNode->fCost) {
                currentNode = openList[i];
                currentIndex = i;
            }
        }

        openList.erase(openList.begin() + currentIndex);
        closedList.push_back(currentNode);

        if (currentNode->point.gridPositionX == destinationNode->point.gridPositionX && currentNode->point.gridPositionY == destinationNode->point.gridPositionY) {
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
            for (int j = -1; j <= 1; j++) {
                if ((i == 0 && j == 0) || (i == -1 && j == -1) || (i == -1 && j == 1) || (i == 1 && j == -1) || (i == 1 && j == 1))
                    continue;
                
                int x = currentNode->point.gridPositionX + i;
                int y = currentNode->point.gridPositionY + j;

                if (x < 0 || x >= 33 || y < 0 || y >= 33)
                    continue;
                
                if (GRID_DATA[x][y].type == OBSTACLE)
                    continue;
                
                Node *newNode = new Node();
                newNode->point = GRID_DATA[x][y];
                newNode->parent = currentNode;
                children.push_back(newNode);
            }
        }

        for (int i = 0; i < children.size(); i++) {
            bool skip = false;
            for (int j = 0; j < closedList.size(); j++) {
                if (children[i]->point.gridPositionX == closedList[j]->point.gridPositionX && children[i]->point.gridPositionY == closedList[j]->point.gridPositionY) {
                    skip = true;
                    break;
                }
            }

            if (skip)
                continue;

            children[i]->gCost = currentNode->gCost + 1;
            children[i]->hCost = abs(children[i]->point.gridPositionX - destinationNode->point.gridPositionX) + abs(children[i]->point.gridPositionY - destinationNode->point.gridPositionY);
            children[i]->fCost = children[i]->gCost + children[i]->hCost;

            for (int j = 0; j < openList.size(); j++) {
                if (children[i]->point.gridPositionX == openList[j]->point.gridPositionX && children[i]->point.gridPositionY == openList[j]->point.gridPositionY) {
                    if (children[i]->gCost > openList[j]->gCost) {
                        skip = true;
                        break;
                    }
                }
            }

            if (skip)
                continue;
            
            openList.push_back(children[i]);
        }
    }
    
    return path;
}

void robotGridCallback(std_msgs::Int32MultiArray msg)
{
    ROS_INFO("Robot grid received");
    int grid[33 * 33];
    for (int i = 0; i < 33 * 33; i++)
        grid[i] = msg.data[i];

    for (int i = 0; i < 33; i++)
        for (int j = 0; j < 33; j++) {
            GRID_DATA[i][j].gridPositionX = i;
            GRID_DATA[i][j].gridPositionY = j;
            GRID_DATA[i][j].type = (PointType)grid[i * 33 + j];
        }
    
    robotGridReceived = true;
}

void setRobotPosition(nav_msgs::Odometry msg)
{
    robotPositionX = msg.pose.pose.position.x;
    robotPositionY = msg.pose.pose.position.y;


    robotPositionReceived = true;
}

void setGoal(geometry_msgs::PoseStamped msg)
{
    ROS_INFO("Goal reveived from topic move_base_simple/goal");
    cout << "Robot grid received: " << robotGridReceived << endl;
    cout << "Robot position received: " << robotPositionReceived << endl;
    if (!robotGridReceived || !robotPositionReceived)
        return;

    if (isnan(msg.pose.position.x) || isnan(msg.pose.position.y))
        return;

    ROS_INFO("Goal set to: (%f, %f)", msg.pose.position.x, msg.pose.position.y);
    double range = sqrt(pow(robotPositionX - msg.pose.position.x, 2) + pow(robotPositionY - msg.pose.position.y, 2));
    ROS_INFO("Range: %f", range);
    if (range < 1)
        return;

    double angle = atan2(msg.pose.position.y - robotPositionY, msg.pose.position.x - robotPositionX);

    int gridX = floor(robotGridPositionX + range / 0.5 * cos(angle));
    int gridY = floor(robotGridPositionY + range / 0.5 * sin(angle));

    if (gridX >= 0 && gridX < 33 && gridY >= 0 && gridY < 33 && GRID_DATA[gridX][gridY].type == FREE)
        GRID_DATA[gridX][gridY].type = DESTINATION;

    if (gridX < 0) gridX = 0;
    else if (gridX >= 33) gridX = 32;

    if (gridY < 0) gridY = 0;
    else if (gridY >= 33) gridY = 32;

    if (GRID_DATA[gridX][gridY].type == FREE) {
        GRID_DATA[gridX][gridY].type = DESTINATION;
        return;
    }

    int initialGridX = gridX;
    int initialGridY = gridY;

    while (GRID_DATA[gridX][gridY].type != FREE) {
        gridY ++;
        if (gridY >= 33) gridX = 0;

        if (GRID_DATA[gridX][gridY].type == FREE) {
            GRID_DATA[gridX][gridY].type = DESTINATION;
            break;
        }

        if (gridY == initialGridY) 
            gridX++;

        if (gridX >= 33) gridX = 0;

        if (gridX == initialGridX && gridY == initialGridY)
            break;
    }

    findPath = true;

    destinationNode = new Node();
    destinationNode->point = GRID_DATA[gridX][gridY];
    destinationNode->gCost = 0;
    destinationNode->hCost = 0;
    destinationNode->fCost = 0;
    destinationNode->parent = NULL;

    displayGrid();

    vector<Node*> path = aStar();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_route");
    ros::NodeHandle nodeHandle;

    ros::Subscriber robotGridSubscriber = nodeHandle.subscribe("robot_grid", 1, robotGridCallback);
    ros::Subscriber sub = nodeHandle.subscribe("move_base_simple/goal", 1, setGoal);
    ros::Subscriber odomSub = nodeHandle.subscribe("odom", 1, setRobotPosition);

    ros::spin();
    return 0;
}
#include <path_planning/sp_rrt.h>
#include <math.h>
#include <cstddef>
#include <iostream>

using namespace sp_rrt;

/**
* default constructor for sp_RRT class
* initializes source to 0,0,0
* adds sorce to sp_rrtTree
*/
sp_RRT::sp_RRT()
{
    sp_RRT::sp_rrtNode newNode;
    newNode.posX = 0;
    newNode.posY = 0;
    newNode.posZ = 0;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    newNode.layer = 0;
    sp_rrtTree.push_back(newNode);
}

/**
* default constructor for sp_RRT class
* initializes source to input X,Y,Z
* adds sorce to sp_rrtTree
*/
sp_RRT::sp_RRT(double input_PosX, double input_PosY, double input_PosZ)
{
    sp_RRT::sp_rrtNode newNode;
    newNode.posX = input_PosX;
    newNode.posY = input_PosY;
    newNode.posZ = input_PosZ;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    newNode.layer = 0;
    sp_rrtTree.push_back(newNode);
}

/**
* Returns the current sp_RRT tree
* @return sp_RRT Tree
*/
vector<sp_RRT::sp_rrtNode> sp_RRT::getTree()
{
    return sp_rrtTree;
}

/**
* For setting the sp_rrtTree to the inputTree
* @param sp_rrtTree
*/
void sp_RRT::setTree(vector<sp_RRT::sp_rrtNode> input_sp_rrtTree)
{
    sp_rrtTree = input_sp_rrtTree;
}

/**
* to get the number of nodes in the sp_rrt Tree
* @return tree size
*/
int sp_RRT::getTreeSize()
{
    return sp_rrtTree.size();
}

/**
* adding a new node to the sp_rrt Tree
*/
void sp_RRT::addNewNode(sp_RRT::sp_rrtNode node)
{
    sp_rrtTree.push_back(node);
}

/**
* removing a node from the sp_RRT Tree
* @return the removed tree
*/
sp_RRT::sp_rrtNode sp_RRT::removeNode(int id)
{
    sp_RRT::sp_rrtNode tempNode = sp_rrtTree[id];
    sp_rrtTree.erase(sp_rrtTree.begin()+id);
    return tempNode;
}

/**
* getting a specific node
* @param node id for the required node
* @return node in the sp_rrtNode structure
*/
sp_RRT::sp_rrtNode sp_RRT::getNode(int id)
{
    return sp_rrtTree[id];
}

/**
* return a node from the sp_rrt tree nearest to the given point
* @param X position in X cordinate
* @param Y position in Y cordinate
* @param Z position in Z cordinate
* @return nodeID of the nearest Node
*/
int sp_RRT::getNearestNodeID(double X, double Y, double Z)
{
    int i, returnID;
    double distance = 9999, tempDistance;
    for(i=0; i<this->getTreeSize(); i++)
    {
        tempDistance = getEuclideanDistance(X, Y, Z, getPosX(i), getPosY(i), getPosZ(i));
        if (tempDistance < distance)
        {
            distance = tempDistance;
            returnID = i;
        }
    }
    return returnID;
}

/**
* returns X coordinate of the given node
*/
double sp_RRT::getPosX(int nodeID)
{
    return sp_rrtTree[nodeID].posX;
}

/**
* returns Y coordinate of the given node
*/
double sp_RRT::getPosY(int nodeID)
{
    return sp_rrtTree[nodeID].posY;
}

/**
* returns Z coordinate of the given node
*/
double sp_RRT::getPosZ(int nodeID)
{
    return sp_rrtTree[nodeID].posZ;
}

/**
* set X coordinate of the given node
*/
void sp_RRT::setPosX(int nodeID, double input_PosX)
{
    sp_rrtTree[nodeID].posX = input_PosX;
}

/**
* set Y coordinate of the given node
*/
void sp_RRT::setPosY(int nodeID, double input_PosY)
{
    sp_rrtTree[nodeID].posY = input_PosY;
}

/**
* set Z coordinate of the given node
*/
void sp_RRT::setPosZ(int nodeID, double input_PosZ)
{
    sp_rrtTree[nodeID].posZ = input_PosZ;
}

/**
* returns parentID of the given node
*/
sp_RRT::sp_rrtNode sp_RRT::getParent(int id)
{
    return sp_rrtTree[sp_rrtTree[id].parentID];
}

/**
* set parentID of the given node
*/
void sp_RRT::setParentID(int nodeID, int parentID)
{
    sp_rrtTree[nodeID].parentID = parentID;
}

/**
* add a new childID to the children list of the given node
*/
void sp_RRT::addChildID(int nodeID, int childID)
{
    sp_rrtTree[nodeID].children.push_back(childID);
}

/**
* returns the children list of the given node
*/
vector<int> sp_RRT::getChildren(int id)
{
    return sp_rrtTree[id].children;
}

/**
* returns number of children of a given node
*/
int sp_RRT::getChildrenSize(int nodeID)
{
    return sp_rrtTree[nodeID].children.size();
}

/**
* returns euclidean distance between two set of X,Y,Z coordinates
*/
double sp_RRT::getEuclideanDistance(double sourceX, double sourceY, double sourceZ, double destinationX, double destinationY, double destinationZ)
{
    return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2) + pow(destinationZ - sourceZ,2));
}

/**
* returns path from root to end node
* @param endNodeID of the end node
* @return path containing ID of member nodes in the vector form
*/
vector<int> sp_RRT::getRootToEndPath(int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(sp_rrtTree[path.front()].nodeID != 0)
    {
        //std::cout<<sp_rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),sp_rrtTree[path.front()].parentID);
    }
    return path;
}

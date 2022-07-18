#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

#define success false
#define running true

using namespace rrt;

bool status = running;

int rrtStepSize = 10;            // 杆件长度

int boundary_x = 100;           //x轴节点界限阈值
int boundary_y = 100;           //y轴节点界限阈值
int boundary_z = 50;            //z轴节点界限阈值

double random_threshold = 0.5;   //进行随机采样的概率
double accuracy_theshold = 5;    //精确度阈值
double link_angle_threshold = 20;//杆件间夹角阈值
int rrtPathLimit = 1;            //得到的轨迹数

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath)
{
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 0.8;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles()
{
    obstacles obst;
    return obst.getObstacleArray();
}

void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

geometry_msgs::Point point;

point.x = tempNode.posX;
point.y = tempNode.posY;
point.z = tempNode.posZ;
rrtTreeMarker.points.push_back(point);

RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

point.x = parentNode.posX;
point.y = parentNode.posY;
point.z = parentNode.posZ;

rrtTreeMarker.points.push_back(point);
}

bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0 || tempNode.posZ < 0 || tempNode.posX > boundary_x || tempNode.posY > boundary_y || tempNode.posZ > boundary_z) 
        return false;
    else return true;
}

bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
         //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}

void generateTempPoint(RRT::rrtNode &tempNode, int goalX, int goalY, int goalZ)
{    
    double seed = rand() % 256;
    double random_rate = seed / 256;

    //std::cout<<"random_rate:"<<random_rate<<endl;

    if(random_rate < random_threshold)
    {
        int x = rand() % boundary_x + 1;
        int y = rand() % boundary_y + 1;
        int z = rand() % boundary_z + 1;
        //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
        
        tempNode.posX = x;
        tempNode.posY = y;
        tempNode.posZ = z;
    }
    else 
    {
        tempNode.posX = goalX;
        tempNode.posY = goalY;
        tempNode.posZ = goalZ;
    }
    
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX, tempNode.posY, tempNode.posZ);
    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    // 取nearest与采样点的方向上的点
    double offset_x = tempNode.posX - nearestNode.posX;
    double offset_y = tempNode.posY - nearestNode.posY;
    double offset_z = tempNode.posZ - nearestNode.posZ;
    double norm = sqrt(pow(offset_x, 2) + pow(offset_y, 2) + pow(offset_z, 2));

    // 检查nearest与采样点方向是否符合杆件夹角阈值限制
    RRT::rrtNode parent_nearestNode = myRRT.getParent(nearestNodeID);
    if(parent_nearestNode.nodeID != nearestNodeID)
    {
        double old_link_x = nearestNode.posX - parent_nearestNode.posX;
        double old_link_y = nearestNode.posY - parent_nearestNode.posY;
        double old_link_z = nearestNode.posZ - parent_nearestNode.posZ;

        //内积求相邻杆夹角
        double cos_theta = (old_link_x * offset_x + old_link_y * offset_y + old_link_z * offset_z) / norm / rrtStepSize;

        if (cos_theta < cos(link_angle_threshold))
            return false;
    }

    tempNode.posX = nearestNode.posX + (rrtStepSize * offset_x / norm);
    tempNode.posY = nearestNode.posY + (rrtStepSize * offset_y / norm);
    tempNode.posZ = nearestNode.posZ + (rrtStepSize * offset_z / norm);

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode))
    {
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}

bool checkNodetoGoal(int X, int Y, int Z, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2) + pow(Y-tempNode.posY,2) + pow(Z-tempNode.posZ,2));
    if(distance < accuracy_theshold)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY, int goalZ)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = tempNode.posZ;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    point.z = goalZ;
    finalpath.points.push_back(point);
}

int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

    //setting source and goal
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;
    sourcePoint.pose.position.z = 0;

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;
    goalPoint.pose.position.z = 30;

    rrt_publisher.publish(sourcePoint);
    rrt_publisher.publish(goalPoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL));
    //initialize rrt specific variables

    //initializing rrtTree
    RRT myRRT(2.0,2.0, 0.0);
    int goalX, goalY, goalZ;
    goalX = goalY = 95; goalZ= 30;


    vector< vector<int> > rrtPaths;      
    vector<int> path;

    int shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();

    bool addNodeResult = false, nodeToGoal = false;

    while(ros::ok() && status)
    {
        if(rrtPaths.size() < rrtPathLimit)
        {
            generateTempPoint(tempNode, goalX, goalY, goalZ);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList);
            if(addNodeResult)
            {
               // std::cout<<"tempnode accepted"<<endl;
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
               // std::cout<<"tempnode printed"<<endl;
                nodeToGoal = checkNodetoGoal(goalX, goalY, goalZ, tempNode);
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY, goalZ);
            rrt_publisher.publish(finalPath);
        }


        rrt_publisher.publish(sourcePoint);
        rrt_publisher.publish(goalPoint);
        rrt_publisher.publish(rrtTreeMarker);
        //rrt_publisher.publish(finalPath);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	return 1;
}

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/sp_rrt.h>
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

using namespace sp_rrt;

bool status = running;

int sp_linkLength = 10;            // 杆件长度

int boundary_x = 70;           //x轴节点界限阈值
int boundary_y = 70;           //y轴节点界限阈值
int boundary_z = 50;            //z轴节点界限阈值

double random_threshold = 0.1;   //进行随机采样的概率
double accuracy_threshold = 2;    //精确度阈值
double link_angle_threshold = 20;//杆件间夹角阈值
int link_numbers = 8;            //机械臂杆件个数
int sp_rrtPathLimit = 1;            //得到的轨迹数


void initializeMarkers(vector<visualization_msgs::Marker> &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &sp_rrtTreeMarker,
    node pos_goal,
    vector<node> pos_source,
    ros::Publisher sp_rrt_publisher)
{
    //init headers
	goalPoint.header.frame_id    = randomPoint.header.frame_id    = sp_rrtTreeMarker.header.frame_id    = "path_planner";
	goalPoint.header.stamp       = randomPoint.header.stamp       = sp_rrtTreeMarker.header.stamp       = ros::Time::now();
	goalPoint.ns                 = randomPoint.ns                 = sp_rrtTreeMarker.ns                 = "path_planner";
	goalPoint.action             = randomPoint.action             = sp_rrtTreeMarker.action             = visualization_msgs::Marker::ADD;
	goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = sp_rrtTreeMarker.pose.orientation.w = 1.0;

    //setting id for each marker
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	sp_rrtTreeMarker.id  = 3;

	//defining types
	sp_rrtTreeMarker.type                                 = visualization_msgs::Marker::LINE_LIST;
	goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	sp_rrtTreeMarker.scale.x = 0.1;
	goalPoint.scale.x = randomPoint.scale.x = 1;
    goalPoint.scale.y = randomPoint.scale.y = 1;
    goalPoint.scale.z = randomPoint.scale.z = 1;

    //setting position of source and goal points
    goalPoint.pose.position.x = pos_goal.posX;
    goalPoint.pose.position.y = pos_goal.posY;
    goalPoint.pose.position.z = pos_goal.posZ;

    for(int i = 0; i < pos_source.size(); i++)
    {
        visualization_msgs::Marker tmpsource;
        tmpsource.id    = 4 + i;
        tmpsource.header.frame_id = "path_planner";
        tmpsource.header.stamp = ros::Time::now();
        tmpsource.ns = "path_planner";
        tmpsource.action = visualization_msgs::Marker::ADD;
        tmpsource.pose.orientation.w = 1.0;
        tmpsource.pose.position.x = pos_source[i].posX;
        tmpsource.pose.position.y = pos_source[i].posY;
        tmpsource.pose.position.z = pos_source[i].posZ;
        tmpsource.color.r = 1.0f;
        tmpsource.color.a = 1.0f;
        tmpsource.scale.x = 1;
        tmpsource.scale.y = 1;
        tmpsource.scale.z = 1;
        sourcePoint.push_back(tmpsource);
    }

    //assigning colors
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	sp_rrtTreeMarker.color.r = 0.8f;
	sp_rrtTreeMarker.color.g = 0.4f;

	goalPoint.color.a = randomPoint.color.a = sp_rrtTreeMarker.color.a = 1.0f;

    ros::spinOnce();
    ros::Duration(0.01).sleep();
}

vector< vector<geometry_msgs::Point> > getObstacles()
{
    obstacles obst;
    return obst.getObstacleArray();
}

void addBranchtosp_RRTTree(visualization_msgs::Marker &sp_rrtTreeMarker, sp_RRT::sp_rrtNode &tempNode, sp_RRT &mysp_RRT)
{

geometry_msgs::Point point;

point.x = tempNode.posX;
point.y = tempNode.posY;
point.z = tempNode.posZ;
sp_rrtTreeMarker.points.push_back(point);

sp_RRT::sp_rrtNode parentNode = mysp_RRT.getParent(tempNode.nodeID);

point.x = parentNode.posX;
point.y = parentNode.posY;
point.z = parentNode.posZ;

sp_rrtTreeMarker.points.push_back(point);
}

bool checkIfInsideBoundary(sp_RRT::sp_rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0 || tempNode.posZ < 0 || tempNode.posX > boundary_x || tempNode.posY > boundary_y || tempNode.posZ > boundary_z) 
        return false;
    else return true;
}

bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, sp_RRT::sp_rrtNode &tempNode)
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

void generateTempPoint(sp_RRT::sp_rrtNode &tempNode, vector<node>pos_source)
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
        //随机取souce点
        node rand_source = pos_source[rand() % pos_source.size()];
        tempNode.posX = rand_source.posX;
        tempNode.posY = rand_source.posY;
        tempNode.posZ = rand_source.posZ;
    }
    
}

bool addNewPointtosp_RRT(sp_RRT &mysp_RRT, sp_RRT::sp_rrtNode &tempNode, int sp_linkLength, vector< vector<geometry_msgs::Point> > &obstArray)
{
    int nearestNodeID = mysp_RRT.getNearestNodeID(tempNode.posX, tempNode.posY, tempNode.posZ);
    
    sp_RRT::sp_rrtNode nearestNode = mysp_RRT.getNode(nearestNodeID);

    // nearest节点是机械臂起始段，无法连接新杆件
    //std::cout << "nearestNode.layer:"<< nearestNode.layer << endl;
    if( nearestNode.layer >= link_numbers)
        return false;

    // 取nearest与采样点的方向上的点
    double offset_x = tempNode.posX - nearestNode.posX;
    double offset_y = tempNode.posY - nearestNode.posY;
    double offset_z = tempNode.posZ - nearestNode.posZ;
    double norm = sqrt(pow(offset_x, 2) + pow(offset_y, 2) + pow(offset_z, 2));

    // 检查nearest与采样点方向是否符合杆件夹角阈值限制
    sp_RRT::sp_rrtNode parent_nearestNode = mysp_RRT.getParent(nearestNodeID);
    if(parent_nearestNode.nodeID != nearestNodeID)
    {
        double old_link_x = nearestNode.posX - parent_nearestNode.posX;
        double old_link_y = nearestNode.posY - parent_nearestNode.posY;
        double old_link_z = nearestNode.posZ - parent_nearestNode.posZ;

        //内积求相邻杆夹角
        double cos_theta = (old_link_x * offset_x + old_link_y * offset_y + old_link_z * offset_z) / norm / sp_linkLength;

        if (cos_theta < cos(link_angle_threshold))
            return false;
    }

    tempNode.posX = nearestNode.posX + (sp_linkLength * offset_x / norm);
    tempNode.posY = nearestNode.posY + (sp_linkLength * offset_y / norm);
    tempNode.posZ = nearestNode.posZ + (sp_linkLength * offset_z / norm);

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode))
    {
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = mysp_RRT.getTreeSize();
        tempNode.layer = nearestNode.layer + 1;
        mysp_RRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}

int checkNodetoSource(vector<node> pos_source, sp_RRT::sp_rrtNode &tempNode)
{
    for(int i = 0; i < pos_source.size(); i++)
    {
        double sourceX = pos_source[i].posX;
        double sourceY = pos_source[i].posY;
        double sourceZ = pos_source[i].posZ;
        double distance = sqrt(pow(sourceX-tempNode.posX,2) + pow(sourceY-tempNode.posY,2) + pow(sourceZ-tempNode.posZ,2));
        if(distance < accuracy_threshold)
        {
            std::cout << "distance error:" << distance << endl;
            return i;
        }            
    }
    return pos_source.size();
}

void setFinalPathData(vector<int> &path, sp_RRT &mysp_RRT, int current_path_size, node source, ros::Publisher sp_rrt_publisher)
{
    sp_RRT::sp_rrtNode tempNode;
    geometry_msgs::Point point;
    visualization_msgs::Marker finalPath;

	finalPath.header.frame_id    = "path_planner";
	finalPath.header.stamp       = ros::Time::now();
	finalPath.ns                 = "path_planner";
	finalPath.action             = visualization_msgs::Marker::ADD;
	finalPath.pose.orientation.w = 1.0;
    finalPath.id                 = 200 + current_path_size;
    finalPath.type               = visualization_msgs::Marker::LINE_STRIP;
    finalPath.scale.x = 0.5;
    finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;
    finalPath.color.a = 1.0f;

    std::cout << "path_joint:" << endl;
    for(int j = 0; j < path.size(); j++)
    {
        tempNode = mysp_RRT.getNode(path[j]);
        std: cout << tempNode.posX << " " << tempNode.posY << " " << tempNode.posZ << endl;
        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = tempNode.posZ;

        finalPath.points.push_back(point);
    }
    std::cout<< "link_num:" << path.size() << endl;
    sp_rrt_publisher.publish(finalPath);
}

int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"sp_rrt_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher sp_rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	//defining markers
    vector<visualization_msgs::Marker> sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker sp_rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    int goalX = 45;
    int goalY = 55;
    int goalZ = 30;
    node pos_goal = {45, 55, 30};
    vector<node> pos_source;
    //node sourcepoint1 = {20, 30, 10};
    node sourcepoint2 = {10, 10, 30};
    //pos_source.push_back(sourcepoint1);
    pos_source.push_back(sourcepoint2);

    initializeMarkers(sourcePoint, goalPoint, randomPoint, sp_rrtTreeMarker, pos_goal, pos_source, sp_rrt_publisher);

    srand (time(NULL));
    //initialize sp_rrt specific variables

    //initializing sp_rrtTree
    sp_RRT mysp_RRT(goalX, goalY, goalZ);

    vector< vector<int> > sp_rrtPaths;
    vector<int> path;

    sp_RRT::sp_rrtNode tempNode;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();

    bool addNodeResult = false, nodeToGoal = false;

    while(ros::ok() && status)
    {
        if(sp_rrtPaths.size() < sp_rrtPathLimit)
        {
            generateTempPoint(tempNode, pos_source);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtosp_RRT(mysp_RRT,tempNode,sp_linkLength,obstacleList);
            if(addNodeResult)
            {
               // std::cout<<"tempnode accepted"<<endl;
                addBranchtosp_RRTTree(sp_rrtTreeMarker,tempNode,mysp_RRT);
                int nodeToSource = checkNodetoSource(pos_source, tempNode);
                if(nodeToSource != pos_source.size())
                {
                    path = mysp_RRT.getRootToEndPath(tempNode.nodeID);
                    setFinalPathData(path, mysp_RRT, sp_rrtPaths.size(), pos_source[nodeToSource], sp_rrt_publisher);
                    sp_rrtPaths.push_back(path);
                    std::cout<<"New Path Found for source " << nodeToSource + 1 << endl;
                    std::cout <<"Total paths :" << sp_rrtPaths.size()<<endl;
                    std::cout << "-------------------------------------"<< endl;
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(sp_rrtPaths.size() >= sp_rrtPathLimit)
        {
            status = success;
            std::cout<<"The RRT search ends."<<endl;
        }

        sp_rrt_publisher.publish(sp_rrtTreeMarker);
        sp_rrt_publisher.publish(goalPoint);
        for(int i = 0; i < pos_source.size(); i++)
            sp_rrt_publisher.publish(sourcePoint[i]);
            
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
	return 1;
}

#ifndef sp_rrt_h
#define sp_rrt_h

#include <vector>
using namespace std;
namespace sp_rrt {
    struct node{
                double posX;
                double posY;
                double posZ;
            };

	class sp_RRT{

        public:

            sp_RRT();
            sp_RRT(double input_PosX, double input_PosY, double input_PosZ);

            struct sp_rrtNode{
                int nodeID;
                double posX;
                double posY;
                double posZ;
                int parentID;
                int layer;
                vector<int> children;
            };

            vector<sp_rrtNode> getTree();
            void setTree(vector<sp_rrtNode> input_sp_rrtTree);
            int getTreeSize();

            void addNewNode(sp_rrtNode node);
            sp_rrtNode removeNode(int nodeID);
            sp_rrtNode getNode(int nodeID);

            double getPosX(int nodeID);
            double getPosY(int nodeID);
            double getPosZ(int nodeID);
            void setPosX(int nodeID, double input_PosX);
            void setPosY(int nodeID, double input_PosY);
            void setPosZ(int nodeID, double input_PosZ);


            sp_rrtNode getParent(int nodeID);
            void setParentID(int nodeID, int parentID);

            void addChildID(int nodeID, int childID);
            vector<int> getChildren(int nodeID);
            int getChildrenSize(int nodeID);

            int getNearestNodeID(double X, double Y, double Z);
            vector<int> getRootToEndPath(int endNodeID);

        private:
            vector<sp_rrtNode> sp_rrtTree;
            double getEuclideanDistance(double sourceX, double sourceY, double sourceZ, double destinationX, double destinationY, double destinationZ);
	};
};

#endif

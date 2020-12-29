#include "node_c/cluster_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clusterNode");

    ros::NodeHandle node;
    Cluster cluster(node);

    ros::spin();
    return 0;
}
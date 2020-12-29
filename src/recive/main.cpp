#include "node_c/recive_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_repub");
    ros::NodeHandle node;

    Reciver reciver(node);

    ros::spin();

    return 0;
}
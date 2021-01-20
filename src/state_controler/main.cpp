#include "node_c/state_controler.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state");

    ros::NodeHandle node;

    State state(node);

    return 0;
}
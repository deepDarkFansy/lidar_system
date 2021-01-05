#include "node_c/tcp_info.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_app");

    ros::NodeHandle node;

    TcpInfo tcp_info(node);

    return 0;
}
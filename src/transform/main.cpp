#include "node_c/transform_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tranform_data");

    ros::NodeHandle node;
    
    Transformer tran(node);

    ros::spin();

    return 0;

}
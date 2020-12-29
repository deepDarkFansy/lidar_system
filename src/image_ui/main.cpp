#include "node_c/Image_ui.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_app");

    ros::NodeHandle node;

    Image_ui my_ui(node);

    ros::spin();
}
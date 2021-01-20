#include "node_c/Image_ui.h"
#include <X11/Xlib.h>


int main(int argc, char **argv)
{
    XInitThreads();
    ros::init(argc, argv, "opencv_app");

    ros::NodeHandle node;

    Image_ui my_ui(node);

    ros::spin();
}
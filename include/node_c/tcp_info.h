#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int32.h>
#include "loam_horizon/common.h"


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <errno.h>

class TcpInfo
{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
public:
    TcpInfo(ros::NodeHandle &node);
    ~TcpInfo(){}

    void initPublisher();
    

    void run();
};

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



class State
{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Subscriber sub;
    state_my state;
public:
    State(ros::NodeHandle &node);
    ~State(){}

    void initPublisher();

    void initSubscriber();
    void changeStateCbk(const std_msgs::Int32ConstPtr &msgs);
    
    void run();
};
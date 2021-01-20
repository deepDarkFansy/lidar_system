#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include "livox_ros_driver/CustomMsg.h"
#include "loam_horizon/common.h"

class Reciver
{
private:
    ros::Publisher pub_pcl_out;
    ros::Subscriber sub_enable;
    ros::Subscriber sub_livox_msg;
    uint64_t TO_MERGE_CNT = 1;
    bool b_dbg_line = false;
    std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
    ros::NodeHandle node;
public:
    Reciver(ros::NodeHandle &node);
    ~Reciver(){};

    void initPublisher();
    void initSubscriber();
    void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in);
    void EnableCbk(const std_msgs::Int32ConstPtr& msgs);
};
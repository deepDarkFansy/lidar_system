#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "loam_horizon/common.h"
#include <tf/transform_datatypes.h>

class Transformer
{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Publisher pub_check;
    ros::Subscriber sub;
    tf::Transform transform;

public:
    Transformer(ros::NodeHandle &node);
    ~Transformer(){}

    void computTransform();
    void initPublisher();
    void initSubscriber();
    void transCbk(const sensor_msgs::PointCloud2ConstPtr &msg);

};


void RGBpointAssociateToMap(PointType const *const pi,
                            pcl::PointXYZRGB *const po);\

bool isInCheckPlace(double x, double y, double z);
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "loam_horizon/common.h"
#include "tool/DBSCANCluster.h"
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/octree/octree.h>
#include <sstream>

#define TO_MERGE_CNT 1

class Cluster
{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Publisher pub_1;
    ros::Subscriber sub;
    std::vector<sensor_msgs::PointCloud2ConstPtr> cloud_data;
    visualization_msgs::Marker my_line_list;

public:
    Cluster(ros::NodeHandle &node);
    ~Cluster(){}

    void initPublisher();
    void initSubscriber();

    void clusterCbk(const sensor_msgs::PointCloud2ConstPtr &msg);

};


visualization_msgs::Marker get_boundry(pcl::PointXYZRGB p_ld, pcl::PointXYZRGB p_ru);


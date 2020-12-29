#include "node_c/recive_data.h"

Reciver::Reciver(ros::NodeHandle &node){
    this->node = node;
    initPublisher();
    initSubscriber();
    
}

void Reciver::initPublisher(){
    pub_pcl_out = node.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);
}


void Reciver::initSubscriber(){
    sub_livox_msg = node.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, &Reciver::LivoxMsgCbk, this);
}


void Reciver::LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in){
    livox_data.push_back(livox_msg_in);
    if (livox_data.size() < TO_MERGE_CNT) return;


    pcl::PointCloud<PointType> pcl_in;

    for (size_t j = 0; j < livox_data.size(); j++) {
        auto& livox_msg = livox_data[j];
        auto time_end = livox_msg->points.back().offset_time;
        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            PointType pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            
            float s = livox_msg->points[i].offset_time / (float)time_end;
            
            pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
            
            pt.curvature = livox_msg->points[i].reflectivity * 0.1;
            
            pcl_in.push_back(pt);
        }
    }

    /// timebase 5ms ~ 50000000, so 10 ~ 1ns
    unsigned long timebase_ns = livox_data[0]->timebase;
    ros::Time timestamp;
    timestamp.fromNSec(timebase_ns);

    //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
    //   pcl_in.size(),
    //           timestamp.toSec(), livox_data.size());
    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
    pcl_ros_msg.header.frame_id = "/rec";
    pub_pcl_out.publish(pcl_ros_msg);
    livox_data.clear();
}


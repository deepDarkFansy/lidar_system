#include "node_c/transform_data.h"


void RGBpointAssociateToMap(PointType const *const pi,
                            pcl::PointXYZRGB *const po) {
  po->x = pi->x;
  po->y = pi->y;
  po->z = pi->z;
  int reflection_map = pi->curvature * 10;
  if (reflection_map < 30) {
    int green = (reflection_map * 255 / 30);
    po->r = 0;
    po->g = green & 0xff;
    po->b = 0xff;
  } else if (reflection_map < 90) {
    int blue = (((90 - reflection_map) * 255) / 60);
    po->r = 0x0;
    po->g = 0xff;
    po->b = blue & 0xff;
  } else if (reflection_map < 150) {
    int red = ((reflection_map - 90) * 255 / 60);
    po->r = red & 0xff;
    po->g = 0xff;
    po->b = 0x0;
  } else {
    int green = (((255 - reflection_map) * 255) / (255 - 150));
    po->r = 0xff;
    po->g = green & 0xff;
    po->b = 0;
  }
}



bool isInCheckPlace(double x, double y, double z)
{
  if(x>=-2 && x<=15 && y>=-1.1 && y<= 1.4 && z>=0.1 && z<= 2)
    return true;
  return false;
}

Transformer::Transformer(ros::NodeHandle &node)
{
    this->node = node;
    computTransform();
    initSubscriber();
    initPublisher();
}


void Transformer::computTransform()
{
    transform.setOrigin(tf::Vector3(-7.1, 0.211, 1.293));
    tf::Quaternion q;
    tfScalar roll = deg2rad(-0.94);
    tfScalar pitch = deg2rad(2.68);
    tfScalar yaw = deg2rad(-5.0);
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
}

void Transformer::initPublisher()
{
    pub = node.advertise<sensor_msgs::PointCloud2>(
        "/transformed", 100
    );
    pub_check = node.advertise<sensor_msgs::PointCloud2>(
        "/checked", 100
    );
}



void Transformer::initSubscriber()
{
    sub = node.subscribe<sensor_msgs::PointCloud2>(
        "/livox_pcl0", 100, &Transformer::transCbk, this
    );
}




void Transformer::transCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(*msg, cloud);
    size_t num = cloud.points.size();

    auto colored_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto colored_cloud_check = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    for(size_t i=0; i<num; i++){
        tf::Vector3 temp(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        temp = transform*temp;
        cloud.points[i].x = temp[0];
        cloud.points[i].y = temp[1];
        cloud.points[i].z = temp[2];
        pcl::PointXYZRGB colored_point;
        RGBpointAssociateToMap(&cloud.points[i], &colored_point);
        if(isInCheckPlace(temp[0], temp[1], temp[2])){
          colored_cloud_check->push_back(colored_point);
        }else{
          colored_cloud->push_back(colored_point);
        }
    }
    

    sensor_msgs::PointCloud2 tt;
    pcl::toROSMsg(*colored_cloud, tt);

    tt.header.stamp = msg->header.stamp;
    tt.header.frame_id = "/trans";

    pub.publish(tt); 


    sensor_msgs::PointCloud2 tt1;
    pcl::toROSMsg(*colored_cloud_check, tt1);

    tt1.header.stamp = msg->header.stamp;
    tt1.header.frame_id = "/trans";
    
    pub_check.publish(tt1);
     
}






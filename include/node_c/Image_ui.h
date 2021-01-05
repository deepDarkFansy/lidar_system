#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include "loam_horizon/common.h"

class Image_ui
{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub1;
    cv::Mat img;
    double per;
    int flag_l;
    int flag_r;
    std::string info;
public:
    Image_ui(ros::NodeHandle &node);
    ~Image_ui(){}

    void initPublisher();
    void initSubscriber();

    void infoCbk(const std_msgs::Int32ConstPtr &msgs);
    void drawItCbk(const sensor_msgs::PointCloud2ConstPtr &msgs);

};


void show_text(cv::Mat &img, std::string text);
void show_rectangle(cv::Mat &img, float per);
void show_left_side(cv::Mat &img, int flag);
void show_right_side(cv::Mat &img, int flag);
void refrash(cv::Mat &img);
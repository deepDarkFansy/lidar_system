#include "node_c/Image_ui.h"


#define FONT_SIZE 5
#define NOR_SIZE 3
#define HEIGHT_P 500

void show_text(cv::Mat &img, std::string text)
{
    cv::putText(img, text, cv::Point(100, 100), cv::QT_FONT_NORMAL, 2, cv::Scalar(0, 0, 200), 3);
}

void show_rectangle(cv::Mat &img, float per)
{
    int height = int(1080-700*per);
    cv::rectangle(img, cv::Point(850, height), cv::Point(1150, 1080), cv::Scalar(0, 200, 200), -1);
}

void show_left_side(cv::Mat &img, int flag)
{
    if(flag == 1){
        show_text(img, "left");
        // cv::circle(img, cv::Point(150, 400), 85, cv::Scalar(219, 112, 147), -1);
        cv::putText(img, ">", cv::Point(100, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
        cv::putText(img, ">", cv::Point(300, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);   
    }else if(flag == 2){
        show_text(img, "too left");
        // cv::circle(img, cv::Point(150, 400), 85, cv::Scalar(219, 112, 147), -1);
        // cv::circle(img, cv::Point(150, 600), 85, cv::Scalar(219, 112, 147), -1);
        cv::putText(img, ">", cv::Point(100, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
        cv::putText(img, ">", cv::Point(300, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
    }else{
        cv::putText(img, ">", cv::Point(100, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
        cv::putText(img, ">", cv::Point(300, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
    }
}

void show_right_side(cv::Mat &img, int flag)
{
    if(flag == 1){
        show_text(img, "right");
        // cv::circle(img, cv::Point(1770, 400), 85, cv::Scalar(219, 112, 147), -1);
        cv::putText(img, "<", cv::Point(1520, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
        cv::putText(img, "<", cv::Point(1720, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
    }else if(flag == 2){
        show_text(img, "too right");
        // cv::circle(img, cv::Point(1770, 400), 85, cv::Scalar(219, 112, 147), -1);
        // cv::circle(img, cv::Point(1770, 600), 85, cv::Scalar(219, 112, 147), -1);
        cv::putText(img, "<", cv::Point(1520, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
        cv::putText(img, "<", cv::Point(1720, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
    }else{
        cv::putText(img, "<", cv::Point(1520, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
        cv::putText(img, "<", cv::Point(1720, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
    }
}

void refrash(cv::Mat &img)
{
    cv::rectangle(img, cv::Point(0, 0), cv::Point(1920, 150), cv::Scalar(28, 28, 28), -1);
    cv::rectangle(img, cv::Point(0, 150), cv::Point(500, 1080), cv::Scalar(79, 79, 79), -1);
    cv::rectangle(img, cv::Point(1420, 150), cv::Point(1920, 1080), cv::Scalar(79, 79, 79), -1);
    cv::rectangle(img, cv::Point(500, 150), cv::Point(1420, 1080), cv::Scalar(108, 108, 108), -1);
}


Image_ui::Image_ui(ros::NodeHandle &node)
{
    this->node = node;
    img.create(1080, 1920, CV_8UC3);
    per = 1.0;
    flag_l = 0;
    flag_r = 0;
    // initPublisher();
    initSubscriber();
}

void Image_ui::initPublisher()
{

}

void Image_ui::initSubscriber()
{
    sub = node.subscribe<sensor_msgs::PointCloud2>(
        "/centers", 100, &Image_ui::drawItCbk, this
    );
}

void Image_ui::drawItCbk(const sensor_msgs::PointCloud2ConstPtr &msgs)
{
    pcl::PointCloud<pcl::PointXYZ> rec;
    pcl::fromROSMsg(*msgs, rec);

    cv::namedWindow("UI", CV_WINDOW_NORMAL);
    cv::setWindowProperty("UI", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    
    if(rec.points.size() > 1){
        auto point = rec.points[0];
        if(point.x >=10)
        {
            per = 1;
        }else
        {
            per = (point.x+2)/12;
        }

        if(point.y <= -0.9){
            flag_r = 2;
            flag_l = 0;
        }else if(point.y <= -0.7){
            flag_r = 1;
            flag_l = 0;
        }else if(point.y <= 1){
            flag_r = 0;
            flag_l = 0;
        }else if(point.y <= 1.2){
            flag_r = 0;
            flag_l = 1;
        }else{
            flag_r = 0;
            flag_l = 2;
        }   
    }else
    {
        per = 0;
        flag_l = 0;
        flag_r = 0;
    }
    
    refrash(img);
    show_rectangle(img, per);
    show_left_side(img, flag_l);
    show_right_side(img, flag_r);
    // ROS_INFO("per: %f, flag_l: %d, flag_r: %d\n", per, flag_l, flag_r);

    cv::imshow("UI", img);
    cv::waitKey(30);
}
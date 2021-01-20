#include "node_c/Image_ui.h"


#define FONT_SIZE 7
#define NOR_SIZE 5
#define HEIGHT_P 500


Image_ui::Image_ui(ros::NodeHandle &node)
{
    this->node = node;
    per = 1.0;
    flag_l = 0;
    flag_r = 0;
    img.create(1080, 1920, CV_8UC3);
    proc1 = std::thread(&Image_ui::drawFunc, this);

    initPublisher();
    initSubscriber();
 
    cv::namedWindow("UI", CV_WINDOW_NORMAL);
    cv::setWindowProperty("UI", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    
}

void Image_ui::initPublisher()
{
    pub = node.advertise<std_msgs::Int32>(
        "/command", 100
    );
}

void Image_ui::initSubscriber()
{
    sub1 = node.subscribe<std_msgs::Int32>(
        "/state", 100, &Image_ui::stateCbk, this
    );
    sub = node.subscribe<sensor_msgs::PointCloud2>(
        "/centers", 100, &Image_ui::drawItCbk, this
    );
}


void Image_ui::drawItCbk(const sensor_msgs::PointCloud2ConstPtr &msgs)
{
    pcl::PointCloud<pcl::PointXYZ> rec;
    pcl::fromROSMsg(*msgs, rec);
    
    if(rec.points.size() > 1){
        auto point = rec.points[0];
        if(point.x >=10)
        {
            per = 1;
        }else
        {
            per = (point.x+2)/12;
        }

        if(point.x >= -1 && point.x <=1){
            text_flag = 1;
        }else if(point.x >1 && point.x <= 4){
            text_flag = 2;
        }else{
            text_flag = 0;
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
}

void Image_ui::stateCbk(const std_msgs::Int32ConstPtr &msgs)
{
    state = state_my (msgs->data);
}

void Image_ui::drawFunc()
{
    while(true){
        refrash(img);
        switch(state){
            case ST_WAIT:
                show_text(img, "   WAIT  ");
                break;
            case ST_ACTIVE:
                show_rectangle(img, per);
                show_left_side(img, flag_l);
                show_right_side(img, flag_r);
                if(text_flag == 2){
                    show_text(img, "SLOW DOWN");
                }else if(text_flag == 1){
                    show_text(img, "   STOP  ");
                    std_msgs::Int32 temp;
                    temp.data = SI_STOP;
                    pub.publish(temp);
                }
                break;
            case ST_STOP:
                show_text(img, "   STOP  ");
                break;
        }

        cv::imshow("PARKING SYSTEM", img);
        cv::waitKey(30);
    }
}



void show_text(cv::Mat &img, std::string text)
{
    cv::putText(img, text, cv::Point(700, 125), cv::QT_FONT_NORMAL, 3, cv::Scalar(0, 0, 200), 3);
}

void show_rectangle(cv::Mat &img, float per)
{
    int height = int(1080-700*per);
    cv::rectangle(img, cv::Point(850, height), cv::Point(1150, 1080), cv::Scalar(0, 200, 200), -1);
}

void show_left_side(cv::Mat &img, int flag)
{
    if(flag == 1){
        cv::putText(img, ">", cv::Point(300, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);   
    }else if(flag == 2){
        cv::putText(img, ">", cv::Point(100, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
        cv::putText(img, ">", cv::Point(300, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
    }else{
        
    }
}

void show_right_side(cv::Mat &img, int flag)
{
    if(flag == 1){
         cv::putText(img, "<", cv::Point(1520, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
    }else if(flag == 2){
        cv::putText(img, "<", cv::Point(1520, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 200, 200), FONT_SIZE);
        cv::putText(img, "<", cv::Point(1720, HEIGHT_P), cv::QT_FONT_NORMAL, NOR_SIZE, cv::Scalar(0, 0, 200), FONT_SIZE);
    }else{
    }
}

void refrash(cv::Mat &img)
{
    cv::rectangle(img, cv::Point(0, 0), cv::Point(1920, 200), cv::Scalar(28, 28, 28), -1);
    cv::rectangle(img, cv::Point(0, 200), cv::Point(500, 1080), cv::Scalar(79, 79, 79), -1);
    cv::rectangle(img, cv::Point(1420, 200), cv::Point(1920, 1080), cv::Scalar(79, 79, 79), -1);
    cv::rectangle(img, cv::Point(500, 200), cv::Point(1420, 1080), cv::Scalar(108, 108, 108), -1);
}
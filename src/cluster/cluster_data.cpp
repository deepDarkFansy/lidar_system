#include "node_c/cluster_data.h"

Cluster::Cluster(ros::NodeHandle &node)
{
    this->node = node;
    initPublisher();
    initSubscriber();
}
    

void Cluster::initPublisher()
{
    pub = node.advertise<visualization_msgs::Marker>(
        "/cluster", 100
    );
    pub_1 = node.advertise<sensor_msgs::PointCloud2>(
        "/centers", 100
    );
}

void Cluster::initSubscriber()
{
    sub = node.subscribe<sensor_msgs::PointCloud2>(
        "/checked", 100, &Cluster::clusterCbk, this
    );
}

visualization_msgs::Marker get_boundry(pcl::PointXYZRGB p_ld, pcl::PointXYZRGB p_ru, int id) 
{
    visualization_msgs::Marker line_list;
    line_list.ns = "lines";
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.color.a = 1.0;
    line_list.color.r = 0.933;
    line_list.color.g = 0.388;
    line_list.color.b = 0.388;
    line_list.scale.x = 0.1;
    line_list.scale.y = 0.1;
    line_list.scale.z = 0.1;
    double x[2] = {p_ld.x, p_ru.x};
    double y[2] = {p_ld.y, p_ru.y};
    double z[2] = {p_ld.z, p_ru.z};
    std::vector<geometry_msgs::Point> points;

    
    for(auto x_t:x){
        for(auto y_t:y){
            for(auto z_t:z){
                geometry_msgs::Point temp;
                temp.x = x_t;
                temp.y = y_t;
                temp.z = z_t;
                points.push_back(temp);
            }
        }
    }
    
    while(!points.empty()){
        geometry_msgs::Point temp = points[0];
        points.erase(points.begin());
        for(auto ele:points){
            if(ele.x == temp.x){
                if(ele.y == temp.y || ele.z == temp.z){
                    line_list.points.push_back(ele);
                    line_list.points.push_back(temp);
                }
            }else if(ele.y == temp.y && ele.z == temp.z){
                line_list.points.push_back(ele);
                line_list.points.push_back(temp);
            }
        }
    }

    return line_list;

}


std::string fun_keep(float x, int n){
    std::ostringstream out;
    out.precision(n);
    out<< std::fixed<<x;
    return out.str();
}

visualization_msgs::Marker get_text(pcl::PointXYZRGB p_ld, pcl::PointXYZRGB p_ru, int id, std::string info)
{
    visualization_msgs::Marker text;
    text.ns = "text";
    text.id = id;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.color.a = 1;
    text.color.r = 1;
    text.color.g = 0;
    text.color.b = 1;
    text.scale.z = 0.2;
    text.pose.position.x = p_ru.x;
    text.pose.position.y = p_ru.y;
    text.pose.position.z = p_ru.z;
    text.text = info;

    return text;

}




void Cluster::clusterCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    cloud_data.push_back(msg);
    if(cloud_data.size() < TO_MERGE_CNT){
        my_line_list.header.stamp = msg->header.stamp;
        my_line_list.header.frame_id = "trans";
        // my_line_list.points.clear();
        pub.publish(my_line_list);
        return;
    }

    

    auto cloud_t = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t1(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<cloud_data.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB> temp;
        pcl::fromROSMsg(*(cloud_data[i]), temp);
        for(int j=0; j<temp.points.size(); j++){
            cloud_t->push_back(temp.points[j]);
        }
    }


    


    if(cloud_t->points.size() < 80){
        my_line_list.header.stamp = msg->header.stamp;
        my_line_list.header.frame_id = "trans";
        my_line_list.color.a = 0;
        my_line_list.action = visualization_msgs::Marker::DELETEALL;
        pub.publish(my_line_list);

        pcl::PointCloud<pcl::PointXYZ> center;
        pcl::PointXYZ cents_op;
        center.push_back(cents_op);

        sensor_msgs::PointCloud2 center_t;
        pcl::toROSMsg(center, center_t);
        center_t.header.frame_id = "trans";
        center_t.header.stamp = msg->header.stamp;
        pub_1.publish(center_t);
        cloud_data.clear();
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud_t);
    filter.setLeafSize(0.02f, 0.02f, 0.02f);
    filter.filter(*cloud_t);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_t);
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANSimpleCluster<pcl::PointXYZRGB> ec;

    ec.setCorePointMinPts(20);
    ec.setClusterTolerance(1);
    ec.setMinClusterSize(40);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_t);
    ec.extract(cluster_indices);

    pcl::PointXYZRGB min_t, max_t;

    if(cluster_indices.size() == 0){
        ROS_INFO("points: %d", cloud_t->points.size());
        my_line_list.header.stamp = msg->header.stamp;
        my_line_list.header.frame_id = "trans";
        my_line_list.color.a = 0;
        my_line_list.action = visualization_msgs::Marker::DELETEALL;
        pub.publish(my_line_list);

        pcl::PointCloud<pcl::PointXYZ> center;
        pcl::PointXYZ cents_op;
        center.push_back(cents_op);

        sensor_msgs::PointCloud2 center_t;
        pcl::toROSMsg(center, center_t);
        center_t.header.frame_id = "trans";
        center_t.header.stamp = msg->header.stamp;
        pub_1.publish(center_t);
        cloud_data.clear();
        return;
    }
    

    int id = 0;
    pcl::PointCloud<pcl::PointXYZ> center;

    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); it++, id++){
        pcl::PointCloud<pcl::PointXYZRGB> tt;
        for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++){
            tt.push_back(cloud_t->points[*pit]);
        }
        pcl::getMinMax3D(tt, min_t, max_t);
        
        auto marker = get_boundry(min_t, max_t, id);
        marker.header.frame_id = "trans";
        marker.header.stamp = msg->header.stamp;

        float height = max_t.z - min_t.z;
        float width = max_t.y - min_t.y;
        float length = max_t.x - min_t.x;

        float position_x = (max_t.x+min_t.x) / 2.0;
        float position_y = (max_t.y+min_t.y) / 2.0;
        float position_z = (max_t.z+min_t.z) / 2.0;

        pcl::PointXYZ cents;
        cents.x = position_x;
        cents.y = position_y;
        cents.z = position_z;
        center.push_back(cents);


        std::string info;
        info = "height:" + fun_keep(height, 2) + "m, width:" + fun_keep(width, 2) + "m, length:" + fun_keep(length, 2);
        info += "m\nx:" + fun_keep(position_x, 2) + "m, y:" + fun_keep(position_y, 2) + "m, z:" + fun_keep(position_z, 2) + "m\n";

        auto my_text = get_text(min_t, max_t, id, info);
        my_text.header.frame_id = "trans";
        my_text.header.stamp = msg->header.stamp;

        pub.publish(my_text);
        pub.publish(marker);
    }

    pcl::PointXYZ cents_op;
    center.push_back(cents_op);

    sensor_msgs::PointCloud2 center_t;
    pcl::toROSMsg(center, center_t);
    center_t.header.frame_id = "trans";
    center_t.header.stamp = msg->header.stamp;
    pub_1.publish(center_t);

    cloud_data.clear();


}
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class Lister_Tf
{
public:
    Lister_Tf();
    void get_tf();
    void print_tf();
private:
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;
    ros::NodeHandle *pnh;
    std::string source_link, target_link;
    geometry_msgs::TransformStamped tf_st_;
};
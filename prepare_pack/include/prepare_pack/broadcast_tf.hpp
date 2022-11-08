#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class Tf_broadcast
{
public:
    Tf_broadcast();
    void operate_tf();
    void static_tf_broadcast();
    void tf_broadcast();
private:
    ros::NodeHandle nh_;
    tf2_ros::TransformBroadcaster tf_;
    tf2_ros::StaticTransformBroadcaster static_tf_;
    int counter_;
};
#pragma once
#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class camera_node
{
public:
    camera_node();
    void parameter_set();
    void operate();
    void callback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr);
private:
    ros::Publisher operate_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle *pnh_;
    std::string camera_topic_name_, output_topic_name_, image_topic_name_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_type;
    message_filters::Synchronizer<Sync_type> *sensor_sync_;
};
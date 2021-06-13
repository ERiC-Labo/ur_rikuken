#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

ros::Publisher gray_pub;

void callback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs) 
{
    ROS_INFO_STREAM(cam_msgs->header.frame_id);
    ROS_INFO_STREAM("width: " << cam_msgs->width << "  height: " << cam_msgs->height);
    for (std::size_t i = 0; i < cam_msgs->K.size() - 3; i+=3)
    {
        ROS_INFO_STREAM(cam_msgs->K[i] << " " << cam_msgs->K[i+1] << " " << cam_msgs->K[i+2]);
    }

    cv_bridge::CvImagePtr input_bridge, cv_3;
    input_bridge = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::BGR8);
    cv_3 = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::MONO8);

    cv::Mat img, out;
    //input_bridge->image;
    cv::cvtColor(input_bridge->image, img, cv::COLOR_BGR2GRAY);
    cv::threshold(img, cv_3->image, 130, 255, cv::THRESH_BINARY);
   
    gray_pub.publish(cv_3->toImageMsg());
    cv::imshow("windo", cv_3->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_main");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string output_topic_name, camera_topic_name, image_topic_name;
    pnh.getParam("output_topic_name", output_topic_name);
    pnh.getParam("image_topic_name", image_topic_name);
    pnh.getParam("camera_topic_name", camera_topic_name);
    gray_pub = nh.advertise<sensor_msgs::Image>(output_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_sub(nh, camera_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic_name, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_type;
    message_filters::Synchronizer<Sync_type> sensor_sync(Sync_type(10), cam_sub, image_sub);
    sensor_sync.registerCallback(boost::bind(callback, _1, _2));

    ros::spin();
    return 0;
}
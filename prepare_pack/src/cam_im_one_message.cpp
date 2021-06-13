#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

template <typename Ty>
void get_one_message(std::string topic_name, ros::NodeHandle nh, int timeout, Ty &message)
{
    boost::shared_ptr<const Ty> share;
    share = ros::topic::waitForMessage<Ty>(topic_name, nh, ros::Duration(timeout));
    if (share != NULL)
    {
        message = *share;
    }
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
    sensor_msgs::CameraInfo cam_msgs;
    sensor_msgs::Image image_msg;
    ros::Publisher gray_pub;
    gray_pub = nh.advertise<sensor_msgs::Image>(output_topic_name, 10);

    ros::Rate loop(10);
    int count = 0;
    while (count++ <= 10)
    {
        get_one_message<sensor_msgs::CameraInfo>(camera_topic_name, nh, 10, cam_msgs);
        get_one_message<sensor_msgs::Image>(image_topic_name, nh, 10, image_msg);

        ROS_INFO_STREAM(cam_msgs.header.frame_id);
        ROS_INFO_STREAM("width: " << cam_msgs.width << "  height: " << cam_msgs.height);
        for (std::size_t i = 0; i < cam_msgs.K.size() - 3; i+=3)
        {
            ROS_INFO_STREAM(cam_msgs.K[i] << " " << cam_msgs.K[i+1] << " " << cam_msgs.K[i+2]);
        }

        cv_bridge::CvImagePtr input_bridge, cv_3;
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv_3 = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat img, out;
        input_bridge->image;
        cv::cvtColor(input_bridge->image, img, cv::COLOR_BGR2GRAY);
        cv::threshold(img, cv_3->image, 130, 255, cv::THRESH_BINARY);
        gray_pub.publish(cv_3->toImageMsg());
        cv::imshow("window", cv_3->image);
        cv::waitKey(1);
        loop.sleep();
    }
    return 0;
}
#include <prepare_pack/camera_and_image.hpp>
#include <image_transport/image_transport.h>
camera_and_image::camera_and_image()
{
    pnh_ = new ros::NodeHandle("~");
    it_ = new image_transport::ImageTransport(nh_);
    parameter_set();
    operate();
}

void camera_and_image::parameter_set()
{
    pnh_->getParam("camera_topic_name", camera_topic_name_);
    pnh_->getParam("image_topic_name", image_topic_name_);
    pnh_->getParam("output_topic_name", output_topic_name_);
}

void camera_and_image::operate()
{
    gray_pub_ = it_->advertise(output_topic_name_, 10);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_type>(Sync_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&camera_and_image::callback, this, _1, _2));
}

void camera_and_image::callback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
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
    input_bridge->image;
    cv::cvtColor(input_bridge->image, img, cv::COLOR_BGR2GRAY);
    cv::threshold(img, cv_3->image, 130, 255, cv::THRESH_BINARY);
    //sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(image_msgs->header, "bgr8", out).toImageMsg();
    //input_bridge->image = out;
    //input_bridge->encoding = "bgr8";
    gray_pub_.publish(cv_3->toImageMsg());
    cv::imshow("windo", cv_3->image);
    cv::waitKey(1);
}
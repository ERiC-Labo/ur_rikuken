#include <camera_node.hpp>
#include <image_transport/image_transport.h>

camera_node::camera_node()
{
    pnh_ = new ros::NodeHandle("~");
    parameter_set();
    operate();
}

void camera_node::parameter_set()
{
    pnh_->getParam("camera_topic_name", camera_topic_name_);
    pnh_->getParam("image_topic_name", image_topic_name_);
    pnh_->getParam("output_topic_name", output_topic_name_);
}

void camera_node::operate()
{
    operate_pub_ =nh_.advertise<sensor_msgs::Image>(output_topic_name_,10);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_type>(Sync_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&camera_node::callback, this, _1, _2));
}

void camera_node::callback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
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

    cv::Mat img, out, gaussian, red, mask;
    input_bridge->image;
    // cv::cvtColor(input_bridge->image, img, cv::COLOR_BGR2GRAY);
    camera_node::detect_red(input_bridge->image, mask);
    // cv::threshold(red, cv_3->image, 130, 255, cv::THRESH_BINARY);
    camera_node::centroid(mask);
    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(image_msgs->header, "bgr8", out).toImageMsg();
    input_bridge->image = out;
    input_bridge->encoding = "bgr8";
    operate_pub_.publish(cv_3->toImageMsg());
    cv::imshow("windo", mask);
    cv::waitKey();
}

void camera_node::detect_red(cv::Mat img, cv::Mat &mask)
{
    cv::Mat mask_1;
    cv::Mat mask_2;
    cv::Mat hsv;

    //change img from bgr to hsv
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    //define hsv limit for mask_1
    std::vector<int> hsv_min;
    hsv_min = {0, 127, 0};
    std::vector<int> hsv_max;
    hsv_max = {30, 255, 255};
    cv::inRange(hsv, hsv_min, hsv_max, mask_1);

    //define hsv lint for mask_2
    hsv_min = {150, 127, 0};
    hsv_max = {179, 255, 255};
    cv::inRange(hsv, hsv_min, hsv_max, mask_2);

    mask = mask_1 + mask_2;
}

void camera_node::centroid(cv::Mat mask)
{
    cv::Moments mu = cv::moments(mask, true);
    cv::Point2f mc = cv::Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
    cv::circle(mask, mc, 4, cv::Scalar(100), 2, 4);
}
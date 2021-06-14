#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ListenTest
{
public:
ListenTest() : tfBuffer_(new tf2_ros::Buffer())
{
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);
    pnh = new ros::NodeHandle("~");
    pnh->getParam("source_link", source_link);
    pnh->getParam("target_link", target_link);
}

void get_tf()
{
    try
    {
        tf_st_ = tfBuffer_->lookupTransform(source_link, target_link, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
}

void print_tf()
{
    ROS_INFO_STREAM("parent link: " << tf_st_.header.frame_id);
    ROS_INFO_STREAM("child _link: " << tf_st_.child_frame_id);
    ROS_INFO_STREAM("x: " << tf_st_.transform.translation.x);
    ROS_INFO_STREAM("y: " << tf_st_.transform.translation.y);
    ROS_INFO_STREAM("z: " << tf_st_.transform.translation.z);
    ROS_INFO_STREAM("q.x: " << tf_st_.transform.rotation.x);
    ROS_INFO_STREAM("q.y: " << tf_st_.transform.rotation.y);
    ROS_INFO_STREAM("q.z: " << tf_st_.transform.rotation.z);
    ROS_INFO_STREAM("q.w: " << tf_st_.transform.rotation.w);
}

tf2_ros::Buffer *tfBuffer_;
tf2_ros::TransformListener *tfListener_;
ros::NodeHandle *pnh;
std::string source_link, target_link;
geometry_msgs::TransformStamped tf_st_;
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "tf2_listener");
    ListenTest listen_test;
    ros::Rate loop(10);

    while(ros::ok())
    {
       listen_test.get_tf();
       listen_test.print_tf();
       loop.sleep();
    }
    ros::spin();
    return 0;
}
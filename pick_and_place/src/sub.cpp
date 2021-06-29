#include <ros/ros.h>
#include <geometry_msgs/Point.h>

void callback(geometry_msgs::Point &box_point)
{
    ROS_INFO_STREAM("subscribe:" << "x:" << box_point.x << "y:" << box_point.y << "z:" << box_point.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("box_point", 10, callback);

    ros::spin();
    return 0;
}
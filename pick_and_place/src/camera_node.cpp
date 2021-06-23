#include <camera_node.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "int");
    camera_node cn;
    ros::spin();
    return 0;
}
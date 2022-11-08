#include <prepare_pack/camera_and_image.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "int");
    camera_and_image cam_i;
    ros::spin();
    return 0;
}
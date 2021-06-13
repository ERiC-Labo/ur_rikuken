#include <prepare_pack/listener_tf.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inti");
    Lister_Tf lister;

    ros::Rate loop(1);
    while (ros::ok())
    {
        lister.get_tf();
        lister.print_tf();
        loop.sleep();
    }
    return 0;
}
#include <prepare_pack/broadcast_tf.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init");
    Tf_broadcast tf_br;
    ros::Rate loop(10);
    while (ros::ok)
    {
        tf_br.operate_tf();
        loop.sleep();
    }
    return 0;
}
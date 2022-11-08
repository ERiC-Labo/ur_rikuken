#include <prepare_pack/listener_tf.hpp>

Lister_Tf::Lister_Tf()
    : tfBuffer_(new tf2_ros::Buffer())
{
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);
    pnh = new ros::NodeHandle("~");
    pnh->getParam("source_link", source_link);
    pnh->getParam("target_link", target_link);

}

void Lister_Tf::get_tf()
{
    try
    {
        tf_st_ = tfBuffer_->lookupTransform(source_link, target_link, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}

void Lister_Tf::print_tf()
{
    ROS_INFO_STREAM("parent link: " << tf_st_.header.frame_id);
    ROS_INFO_STREAM("child_link: " << tf_st_.child_frame_id);
    ROS_INFO_STREAM("x: " << tf_st_.transform.translation.x);
    ROS_INFO_STREAM("y: " << tf_st_.transform.translation.y);
    ROS_INFO_STREAM("z: " << tf_st_.transform.translation.z);
    ROS_INFO_STREAM("q.x: " << tf_st_.transform.rotation.x);
    ROS_INFO_STREAM("q.y: " << tf_st_.transform.rotation.y);
    ROS_INFO_STREAM("q.z: " << tf_st_.transform.rotation.z);
    ROS_INFO_STREAM("q.w: " << tf_st_.transform.rotation.w);

}
#include <prepare_pack/broadcast_tf.hpp>
#include <tf2/LinearMath/Quaternion.h>

Tf_broadcast::Tf_broadcast()
    : counter_(0)
{

}

void Tf_broadcast::static_tf_broadcast()
{
    geometry_msgs::TransformStamped sta_tf_;
    sta_tf_.header.stamp = ros::Time::now();
    sta_tf_.header.frame_id = "base_link";
    sta_tf_.child_frame_id = "rikuken_tf";
    sta_tf_.transform.translation.x = 0.2;
    sta_tf_.transform.translation.y = 0;
    sta_tf_.transform.translation.z = 0.6;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 3.14/2, 0);
    sta_tf_.transform.rotation.x = quat.x();
    sta_tf_.transform.rotation.y = quat.y();
    sta_tf_.transform.rotation.z = quat.z();
    sta_tf_.transform.rotation.w = quat.w();
    static_tf_.sendTransform(sta_tf_);
}

void Tf_broadcast::tf_broadcast()
{
    geometry_msgs::TransformStamped dynamic_tf_;
    dynamic_tf_.header.stamp = ros::Time::now();
    dynamic_tf_.header.frame_id = "rikuken_tf";
    dynamic_tf_.child_frame_id = "rotation_tf";
    dynamic_tf_.transform.translation.x = 0.3*cos(counter_ * 0.1);
    dynamic_tf_.transform.translation.y = 0.3 * sin(counter_*0.1);
    dynamic_tf_.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0, 0);
    dynamic_tf_.transform.rotation.x = quat.x();
    dynamic_tf_.transform.rotation.y = quat.y();
    dynamic_tf_.transform.rotation.z = quat.z();
    dynamic_tf_.transform.rotation.w = quat.w();
    tf_.sendTransform(dynamic_tf_);
}

void Tf_broadcast::operate_tf()
{
    counter_++;
    static_tf_broadcast();
    tf_broadcast();
}
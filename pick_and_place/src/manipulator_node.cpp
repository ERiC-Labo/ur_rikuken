#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <manipulator_node.hpp>

manipulator_node::manipulator_node(std::string group_name)
{
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
    move_group->setPlanningTime(0.050);
    move_group->setPlannerId("RRTConnect");
}

void manipulator_node::move_point(geometry_msgs::Point &image_point)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = image_point.x;
    pose.pose.position.y = image_point.y;
    pose.pose.position.z = image_point.z;
    pose.pose.orientation.w = 1.0;

    moveit::planning_interface::MoveItErrorCode ret;

    move_group->setPoseTarget(pose);
    ret = move_group->move();
    if (!ret) {
        ROS_WARN_STREAM("Fail:" << ret.val);
    }
    ros::Duration(0.5).sleep();

    ros::shutdown();
}
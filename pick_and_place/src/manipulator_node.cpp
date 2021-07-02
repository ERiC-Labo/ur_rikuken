#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <manipulator_node.hpp>
#include <geometry_msgs/Point.h>

manipulator_node::manipulator_node(std::string group_name)
{
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
}

void manipulator_node::move_point(geometry_msgs::Point &image_point)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.x = image_point.x;
    wpose.position.y = image_point.y;
    wpose.position.z = image_point.z;
    wpose.orientation.w = 0.5;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    move_group->execute(trajectory);
}
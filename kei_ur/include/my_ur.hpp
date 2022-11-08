#ifndef _MY_UR_H_
#define _MY_UR_H_

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
class MYUR
{
public:
    void print_vector(std::vector<double> joint);
    void end_pose();
    void bit_move_x(double x_value);
    void bit_move_y(double y_value);
    void bit_move_z(double z_value);
    void move_point(double x_value, double y_value, double z_value, double q_x, double q_y, double q_z, double q_w);
    MYUR(std::string);
    moveit::planning_interface::MoveGroupInterface *move_group;
};  
#endif
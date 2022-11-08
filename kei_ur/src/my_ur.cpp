#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <my_ur.hpp>


MYUR::MYUR(std::string group_name)
{
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
}
void MYUR::print_vector(std::vector<double> joint){
    std::cout << "value is";
    for(int i = 0; i < joint.size(); i++){
        std::cout << joint[i] << "    ";
    }
    std::cout << std::endl;
}

void MYUR::end_pose(){
    geometry_msgs::Pose end_pose;
    end_pose = move_group->getCurrentPose().pose;
    ROS_INFO_STREAM("Endeffector pose x: " << end_pose.position.x << "   y: " << end_pose.position.y << "   z: " << end_pose.position.z);
}

void MYUR::bit_move_x(double x_value){
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.x += x_value;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group->execute(trajectory);
    MYUR::end_pose();
}

void MYUR::bit_move_y(double y_value){
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.y += y_value;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group->execute(trajectory);
}

void MYUR::bit_move_z(double z_value){
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.z += z_value;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group->execute(trajectory);
}

void MYUR::move_point(double x_value, double y_value, double z_value, double q_x, double q_y, double q_z, double q_w){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.2;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.707106;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.707106;
    move_group->setPoseTarget(pose);
    move_group->move();
   
}
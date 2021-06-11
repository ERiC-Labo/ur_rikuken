#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <my_ur.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "move_direction");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MYUR myur;

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    myur.print_vector(move_group.getCurrentJointValues());
    
    myur.end_pose();

    myur.bit_move_x(-0.15);
    
}
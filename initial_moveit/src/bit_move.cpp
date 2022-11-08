#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>


void print_vector(std::vector<double> joint)
{
    std::cout << "value is ";
    for (int i = 0; i < joint.size(); i++)
    {
        std::cout << joint[i] << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_moveit");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    print_vector(move_group.getCurrentJointValues());

    geometry_msgs::Pose end_pose;
    end_pose = move_group.getCurrentPose().pose;
    ROS_INFO_STREAM("Endeffector pose  x: " << end_pose.position.x << "    y: " << end_pose.position.y
                            << "    z: " << end_pose.position.z);
    ROS_INFO_STREAM("Q.x: " << end_pose.orientation.x << "  Q.y: " << end_pose.orientation.y
                        << "  Q.z: " << end_pose.orientation.z << "   Q.w: " << end_pose.orientation.w);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group.getCurrentPose().pose;
    std::cout << "The z value is ";
    double z_value;
    std::cin >> z_value;
    wpose.position.z += z_value;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.execute(trajectory);


}
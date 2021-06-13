#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define PI 3.1415

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
    trajectory_msgs::JointTrajectoryPoint joint_value;
    joint_value.positions.push_back(-PI/2);
    joint_value.positions.push_back(-PI/2);
    joint_value.positions.push_back(-PI/5);
    joint_value.positions.push_back(-2*PI/3);
    joint_value.positions.push_back(PI/2);
    joint_value.positions.push_back(-PI/2);

    trajectory_msgs::JointTrajectory joint;
    joint.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
   // trajectory_msgs::JointTrajectoryPoint j(1);
   // joint.points
   // joint.points[0].time_from_start(1.0);
    //joint.points[0].positions = joint_value;
    joint.header.stamp = ros::Time::now();

    ros::Rate loop(10);
    while (ros::ok()) {
        pub.publish(joint);
        loop.sleep();
    }
    

    return 0;
    

}
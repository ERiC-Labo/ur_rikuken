#ifndef _MY_UR_H_
#define _MY_UR_H_

#include <ros/ros.h>

class MYUR
{
public:
    void print_vector(std::vector<double> joint);
    void end_pose();
    void bit_move_x(double x_value);
    void bit_move_y(double y_value);
    void bit_move_z(double z_value);
    MYUR(std::string);
    moveit::planning_interface::MoveGroupInterface *move_group;
};  
#endif
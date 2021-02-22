#ifndef MOVEIT_JOINT_H
#define MOVEIT_JOINT_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

using namespace std;

class MoveitIk
{

private:
    std::vector<double> joint_group_positions;
    moveit::planning_interface::MoveGroupInterface armgroup;

public:
    MoveitIk(); //初始化机械臂的参数

    void goSW(); //去某点

    void goHome(); //回到初始位置

    void initMove();
};

#endif

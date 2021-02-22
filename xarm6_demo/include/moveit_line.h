#ifndef MOVEIT_LINE_H
#define MOVEIT_LINE_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>

class MoveitLine
{

private:
    moveit::planning_interface::MoveGroupInterface armgroup;
    geometry_msgs::PoseStamped target_pose;

public:
    MoveitLine();

    void goSW(); //去某点

    void goHome(); //回到初始位置

    void initMove();
};

#endif

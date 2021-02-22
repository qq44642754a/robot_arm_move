#ifndef DOOR_OPEN_H
#define DOOR_OPEN_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include "ar_track_alvar/Marker.h"
#include <actionlib/client/simple_action_client.h>
#include <xarm_gripper/MoveAction.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

namespace RobotArm
{
    class Handle_1
    {

    private:
        moveit::planning_interface::MoveGroupInterface armgroup;
        geometry_msgs::PoseStamped current_pose, Marker_pose, Marker_id;
        geometry_msgs::Pose target_pose;
        std::vector<double> joint_group_positions, rpy;
        std::string end_effector_link, reference_frame;
        ros::NodeHandle nh_;
        ros::Subscriber pose_sub;
        int get_pose, arm_move;
        double handle_length, open_angle;

    public:
        Handle_1(vector<double> orientation, vector<double> start_joint, double handle_length, double open_angle);

        ~Handle_1();

        void gripper_open(); //gripper open

        void gripper_close(); //gripper close

        void goSP(); //go to starting point

        void goHandle(); //go to the point of Handle

        void turn_handle(); //turn the handle

        void base_move(); // base move

        void restore_handle(); // handle restore

        void goHome(); // go to the home pose

        void initMove();

        void poseCallback(const visualization_msgs::Marker &marker_tmp);

        int open_finish;
    };
}
#endif

#include "moveit_ik.h"
#include <iostream>

using namespace std;

void MoveitIk ::armInit() 
{


}
void MoveitIk ::goSW()
{
    moveit::planning_interface::MoveGroupInterface armgroup("xarm6");
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = 0;
    joint_group_positions[1] = -0.57;
    joint_group_positions[2] = 0;
    joint_group_positions[3] = 0;
    joint_group_positions[4] = 0;
    joint_group_positions[5] = 0;

    armgroup.setJointValueTarget(joint_group_positions);
    armgroup.move();
}

void MoveitIk ::goHome()
{
    moveit::planning_interface::MoveGroupInterface armgroup("xarm6");
    armgroup.setNamedTarget("home");
    armgroup.move();
    armgroup.move();

}

void MoveitIk ::initMove()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(3.0).sleep();
    armInit();
    goSW();
    sleep(1);
    goHome();
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "moveit_ik");
    moveit::planning_interface::MoveGroupInterface armgroup("xarm6");
    //获取终端link的名称
    std::string end_effector_link = armgroup.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "link_base";
    armgroup.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    armgroup.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    armgroup.setGoalPositionTolerance(0.001);
    armgroup.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    armgroup.setMaxAccelerationScalingFactor(0.1);
    armgroup.setMaxVelocityScalingFactor(0.1);
    MoveitIk move;
    while (ros::ok())
        {
	 ros::spinOnce();
    	 move.initMove();
        }


}

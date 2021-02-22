#include "moveit_line.h"
#include <iostream>

using namespace std;

MoveitLine::MoveitLine(): armgroup("xarm6")
{
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

}


void MoveitLine::goSW()
{
    //moveit::planning_interface::MoveGroupInterface armgroup("xarm6");
    target_pose = armgroup.getCurrentPose();
    target_pose.pose.position.z += 0.3;
    cout << target_pose << endl;
    armgroup.setPoseTarget(target_pose);    
    armgroup.move();
}

void MoveitLine::goHome()
{
    //moveit::planning_interface::MoveGroupInterface armgroup("xarm6");
    armgroup.setNamedTarget("home");
    armgroup.move();
}

void MoveitLine::initMove()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    goSW();
    sleep(1);
    goHome();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "moveit_lint");
    MoveitLine move;
    while (ros::ok())
        {
	 ros::spinOnce();
    	 move.initMove();
        }


}

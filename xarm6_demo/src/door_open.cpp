/*The demo of the robot opening the door, which uses ARMarker to locate the door handle.

 Author：Zheng, Zhitao
 Emali: ztzheng@fitgreat.cn

*/

#include "door_open.h"
namespace RobotArm
{

    Handle_1::Handle_1(vector<double> orientation, vector<double> start_joint, double _handle_length, double _open_angle) : armgroup("xarm6"), joint_group_positions(6), rpy(3), get_pose(0), open_finish(0), arm_move(0)
    {
        //获取终端link的名称
        end_effector_link = armgroup.getEndEffectorLink();

        //设置目标位置所使用的参考坐标系
        reference_frame = "link_base";
        armgroup.setPoseReferenceFrame(reference_frame);

        //当运动规划失败后，允许重新规划
        armgroup.allowReplanning(true);

        //设置位置(单位：米)和姿态（单位：弧度）的允许误差
        armgroup.setGoalPositionTolerance(0.001);
        armgroup.setGoalOrientationTolerance(0.01);

        //设置允许的最大速度和加速度
        armgroup.setMaxAccelerationScalingFactor(0.1);
        armgroup.setMaxVelocityScalingFactor(0.1);

        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::WallDuration(5.0).sleep();

        target_pose.orientation.x = orientation[0];
        target_pose.orientation.y = orientation[1];
        target_pose.orientation.z = orientation[2];
        target_pose.orientation.w = orientation[3];

        joint_group_positions[0] = start_joint[0] / 180.0 * M_PI;
        joint_group_positions[1] = start_joint[1] / 180.0 * M_PI;
        joint_group_positions[2] = start_joint[2] / 180.0 * M_PI;
        joint_group_positions[3] = start_joint[3] / 180.0 * M_PI;
        joint_group_positions[4] = start_joint[4] / 180.0 * M_PI;
        joint_group_positions[5] = start_joint[5] / 180.0 * M_PI;

        handle_length = _handle_length;
        open_angle = _open_angle;

        pose_sub = nh_.subscribe("/visualization_marker", 10, &Handle_1::poseCallback, this);
    }

    Handle_1::~Handle_1()
    {
        cout << "Delete the class" << endl;
    }

    void Handle_1::gripper_open()
    {
        actionlib::SimpleActionClient<xarm_gripper::MoveAction> ac("xarm/gripper_move", true);
        ac.waitForServer();
        xarm_gripper::MoveGoal open;
        open.target_pulse = 800;
        open.pulse_speed = 1500;
        ac.sendGoal(open);
    }

    void Handle_1::gripper_close()
    {
        actionlib::SimpleActionClient<xarm_gripper::MoveAction> ac("xarm/gripper_move", true);
        ac.waitForServer();
        xarm_gripper::MoveGoal open;
        open.target_pulse = 0;
        open.pulse_speed = 1500;
        ac.sendGoal(open);
    }

    void Handle_1::poseCallback(const visualization_msgs::Marker &marker_tmp)
    {
        if (arm_move == 0)
        {
            Marker_pose.header = marker_tmp.header;
            Marker_pose.pose.position = marker_tmp.pose.position;
            Marker_id.pose.position.x = marker_tmp.id;
            if (Marker_pose.pose.position.x != 0)
            {
                get_pose = 1;
            }

            //计算物体在机械臂坐标系下的位置(tf转化)
            Eigen::Matrix<double, 4, 4> Base_H_Cam;
            Base_H_Cam << 0.0107, 0.1326, 0.9911, -0.1808,
                -0.9999, 0.0049, 0.0102, -0.1944,
                -0.0035, -0.9912, 0.1327, 0.4541,
                0.0, 0.0, 0.0, 1.0;

            Eigen::Matrix<double, 4, 1> Cam_H_Obj;
            Cam_H_Obj << Marker_pose.pose.position.x, Marker_pose.pose.position.y, Marker_pose.pose.position.z, 1.0;
            Eigen::Matrix<double, 4, 1> Base_H_Obj;
            Base_H_Obj = Base_H_Cam * Cam_H_Obj; //得到标签的位置

            target_pose.position.x = Base_H_Obj[0];
            target_pose.position.y = Base_H_Obj[1];
            target_pose.position.z = Base_H_Obj[2];
            arm_move = 1;
        }
    }
    void Handle_1::goSP() //去预备状态
    {
        armgroup.setJointValueTarget(joint_group_positions);
        armgroup.move();
    }

    void Handle_1::goHandle() //去门把手的位置
    {
        armgroup.setPoseTarget(target_pose);
        armgroup.move();
        current_pose = armgroup.getCurrentPose(); //夹爪关闭时的位姿
    }

    void Handle_1::turn_handle()
    {
        rpy = armgroup.getCurrentRPY();
        rpy[1] = open_angle / 180.0 * M_PI; //门把手转动的角度
        Eigen::Quaterniond quaternion3;
        quaternion3 = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());

        //门把手坐标的变化值
        target_pose.position.x = current_pose.pose.position.x;
        target_pose.position.y = current_pose.pose.position.y - handle_length * (1.0 - cos(open_angle / 180.0 * M_PI));
        target_pose.position.z = current_pose.pose.position.z - handle_length * sin(open_angle / 180.0 * M_PI);
        target_pose.orientation.x = quaternion3.x();
        target_pose.orientation.y = quaternion3.y();
        target_pose.orientation.z = quaternion3.z();
        target_pose.orientation.w = quaternion3.w();
        armgroup.setPoseTarget(target_pose);
        armgroup.move();
    }

    void Handle_1::base_move()
    {
        //底盘的运动
    }

    void Handle_1::restore_handle()
    {
        armgroup.setPoseTarget(current_pose);
        armgroup.move();
    }

    void Handle_1::goHome()
    {
        armgroup.setJointValueTarget(joint_group_positions);
        armgroup.move();
        armgroup.setNamedTarget("home");
        armgroup.move();
    }

    void Handle_1::initMove()
    {
        if (get_pose == 1)
        {
            ros::AsyncSpinner spinner(1);
            spinner.start();
            goSP();
            cout << "arrive at starting point..." << endl;
            gripper_open();
            sleep(3);
            goHandle();
            gripper_close();
            cout << "attain the handle..." << endl;
            sleep(3);
            turn_handle();
            cout << "the door handle was turned..." << endl;
            sleep(1);
            base_move();
            cout << "the door was opened..." << endl;
            sleep(1);
            restore_handle();
            gripper_open();
            sleep(3);
            cout << "the handle was restored..." << endl;
            goHome();
            gripper_close();
            cout << "opening complete..." << endl;
            open_finish = 1;
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_open");
    std::vector<double> orientation(4);
    orientation[0] = orientation[1] = orientation[2] = orientation[3] = 0.5;
    std::vector<double> start_joint(6);
    start_joint[0] = -90.0;
    start_joint[1] = -12.4;
    start_joint[2] = -19.3;
    start_joint[3] = -76.4;
    start_joint[4] = -81.7;
    start_joint[5] = -58.9;
    double handle_length = 0.082;
    double open_angle = 40.0;
    RobotArm::Handle_1 handle_1(orientation, start_joint, handle_length, open_angle);
    while (ros::ok())
    {
        ros::spinOnce();
        handle_1.initMove();
        if (handle_1.open_finish == 1)
        {
            ros::shutdown();
        }
    }
}



# robot_arm_move

xarm6 是机械臂原型

下载步骤
cd catkin_ws/src
git clone https://github.com/xArm-Developer/xarm_ros.git
cd ..
catkin_make


1. 运行 roslaunch xarm6_gripper_moveit_config demo.launch   //在rviz中操作机械臂

xarm_demo 是机械臂运动的相关程序

moveit_ik.cpp 是机械臂的关节运动

move_line.cpp 是机械臂的直线运动

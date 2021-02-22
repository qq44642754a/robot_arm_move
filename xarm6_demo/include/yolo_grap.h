#ifndef YOLO_GRAP_H
#define YOLO_GRAP_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace cv;
using namespace std;
class ImageConverter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_color; //接收彩色图像
    image_transport::Subscriber image_sub_depth; //接收深度图像
    ros::Subscriber pose_sub;
    ros::Subscriber Object_sub;
    ros::Subscriber camera_info_sub_; //接收深度图像对应的相机参数话题
    ros::Publisher arm_point_pub_;    //发布一个三维坐标点，可用于可视化

    sensor_msgs::CameraInfo camera_info;
    geometry_msgs::PointStamped output_point;

    /* Mat depthImage,colorImage; */
    Mat colorImage;
    Mat depthImage = Mat::zeros(480, 640, CV_16UC1); //注意这里要修改为你接收的深度图像尺寸
    Point mousepos = Point(0, 0);                    /* mousepoint to be map */

    //variant of Robotarm
    moveit::planning_interface::MoveGroupInterface armgroup;
    geometry_msgs::PoseStamped target_pose;
    int get_pose;      //判断是否已经得到了物体的坐标
    int grasp_running; //判断机械臂是否在工作

public:
    ImageConverter();
    ~ImageConverter();
    void ObjectCallback(const darknet_ros_msgs::BoundingBoxes &object_tmp);
    void cameraInfoCb(const sensor_msgs::CameraInfo &msg);
    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg);
    void imageColorCb(const sensor_msgs::ImageConstPtr &msg);
    void goStart();
    void goAttach();
    void initMove();
};

#endif
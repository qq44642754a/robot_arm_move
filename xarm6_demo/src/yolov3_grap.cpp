/**********************
coordinate_map.cpp
author: wxw and Zhitao zheng 
email: qq44642754@gmail.com
time: 2021.02.07
**********************/
#include "yolo_grap.h"

ImageConverter::ImageConverter() : it_(nh_), armgroup("xarm6")
{
	//topic sub:

	//获取鼠标的坐标，通过param指针传出到类成员Point mousepos
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::WallDuration(5.0).sleep();
	Object_sub = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::ObjectCallback, this);

	image_sub_depth = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ImageConverter::imageDepthCb, this);
	image_sub_color = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageColorCb, this);
	camera_info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &ImageConverter::cameraInfoCb, this);

	//topic pub:
	arm_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/mouse_point", 10);

	cv::namedWindow("colorImage");

	// robot initialization:

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

	get_pose = 0;
	grasp_running = 0;
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow("colorImage");
}

void ImageConverter::ObjectCallback(const darknet_ros_msgs::BoundingBoxes &object_tmp)
{
	mousepos.x = (object_tmp.bounding_boxes[0].xmin + object_tmp.bounding_boxes[0].xmax) / 2;
	mousepos.y = (object_tmp.bounding_boxes[0].ymin + object_tmp.bounding_boxes[0].ymax) / 2;
}

void ImageConverter::cameraInfoCb(const sensor_msgs::CameraInfo &msg)
{
	camera_info = msg;
}

void ImageConverter::imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		depthImage = cv_ptr->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void ImageConverter::imageColorCb(const sensor_msgs::ImageConstPtr &msg)
{

	if (grasp_running == 0) //当机械臂在运动时，不去识别物体.(降低内存)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			colorImage = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		if (mousepos.x != 0 && mousepos.y != 0) //接收到topic的信息时，getpose = 1
		{
			get_pose = 1;
		}
		//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
		float real_z = 0.001 * depthImage.at<u_int16_t>(mousepos.y, mousepos.x);
		float real_x =
			(mousepos.x - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
		float real_y =
			(mousepos.y - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;

		char tam[100];
		sprintf(tam, "(%0.2fm,%0.2fm,%0.2fm)", real_x, real_y, real_z);
		putText(colorImage, tam, mousepos, FONT_HERSHEY_SIMPLEX, 0.6,
				cvScalar(0, 0, 255), 1); //打印到屏幕上
		circle(colorImage, mousepos, 2, Scalar(255, 0, 0));
		output_point.header.frame_id = "/camera_depth_optical_frame";
		output_point.header.stamp = ros::Time::now();
		output_point.point.x = real_x;
		output_point.point.y = real_y;
		output_point.point.z = real_z;
		arm_point_pub_.publish(output_point);
		cv::imshow("colorImage", colorImage);
		cv::waitKey(1);
		if (get_pose == 1)
		{
			grasp_running = 1; //当得到识别的物体坐标后，机械臂开始运动
		}
	}
}

void ImageConverter::goStart()
{
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

void ImageConverter::goAttach()
{
	target_pose = armgroup.getCurrentPose();
	target_pose.pose.position.z += 0.1;
	armgroup.setPoseTarget(target_pose);
	armgroup.move();
}

void ImageConverter::initMove()
{
	if (get_pose == 1) //当得到识别的物体坐标后，机械臂开始运动
	{
		ros::AsyncSpinner spinner(1);
		spinner.start();
		goStart();
		cout << "arrive at start point" << endl;
		sleep(1);
		goAttach();
		sleep(3);
		grasp_running = 0;
		mousepos.x = 0;
		mousepos.y = 0;
		get_pose = 0;
		sleep(1);
		cout << "arrive at attach point" << endl;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "yolov3_grap");
	ImageConverter imageconverter;
	while (ros::ok())
	{
		ros::spinOnce();
		imageconverter.initMove();
	}
	return (0);
}

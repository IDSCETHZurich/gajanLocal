#ifndef CAMERA_CALIBRATION_HPP_
#define CAMERA_CALIBRATION_HPP_

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include "quaternions.hpp"

const string IMAGE_WINDOW = "Image Window";

class CalibrationNode
{
private:
	enum return_values
	{
		pose_robot_stored 			= 0,
		pose_robot_not_stored 		= 1,
		checkerboard_found 			= 2,
		checkerboard_not_found 		= 3,
		object_points_stored 		= 4,
		camera_calibrated			= 5,
		camera_not_calibrated		= 6
	};

	ros::NodeHandle ROSNode;

	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber calibrationImageSubscriber;

	ros::Subscriber robotPoseSubscriber;

	geometry_msgs::Pose robotPose;

	cv::Size image_size;
	cv::Size pattern;
	cv::Size pattern_size;

	cv::Mat image;

	cv::vector<cv::vector<cv::Point2f> > image_points;
	cv::vector<cv::vector<cv::Point3f> > object_points;
	cv::vector<geometry_msgs::Pose> robotPoseVector;

	cv::Mat camera_matrix, distortion_coefficients;

	bool readPoseFlag;

//	Eigen::Vector4d pattern_to_base;


public:
	CalibrationNode ();

	CalibrationNode (ros::NodeHandle& n);

	~CalibrationNode ();

	void imgCallback (const sensor_msgs::ImageConstPtr& msg);

	void poseCallback (const geometry_msgs::PoseConstPtr& msg);

	static void mouseCallback (int event, int x, int y, int flags, void* param);

	int storeData ();

//	int getObjectPoints ();

//	cv::vector<cv::vector<cv::Point2f> > getImagePoints ();
//
//	cv::vector<cv::vector<cv::Point3f> > getObjectPoints ();

	int calibrateCamera ();

};

#endif /* CAMERA_CALIBRATION_HPP_ */

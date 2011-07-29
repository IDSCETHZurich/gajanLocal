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
		checkboard_found 			= 2,
		checkboard_not_found 		= 3,
		object_points_stored 		= 4,
		camera_calibrated			= 5,
		camera_not_calibrated		= 6
	};

	ros::NodeHandle ROSNode;

	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber calibration_image_subscriber;

	ros::Subscriber pose_robot_subscriber;

	geometry_msgs::Pose pose_robot;

	cv::Size image_size;
	cv::Size pattern;
	cv::Size pattern_size;

	cv::Mat image;

	cv::vector<cv::vector<cv::Point2f> > image_points;
	cv::vector<cv::vector<cv::Point3f> > object_points;
	cv::vector<geometry_msgs::Pose> pose_robot_vector;

	cv::Mat camera_matrix, distortion_coefficients;

	Eigen::Vector4d pattern_to_base;


public:
	CalibrationNode ();

	CalibrationNode (ros::NodeHandle& n);

	~CalibrationNode ();

	void callback (const sensor_msgs::ImageConstPtr& msg);

	void storePose (const geometry_msgs::PoseConstPtr& msg);

	static void mouseCallback (int event, int x, int y, int flags, void* param);

	int storeCheckboard (CalibrationNode* Calibration);

	int getObjectPoints (CalibrationNode* Calibration);

	cv::vector<cv::vector<cv::Point2f> > getImagePoints ();

	cv::vector<cv::vector<cv::Point3f> > getObjectPoints ();


	int calibrateCamera (CalibrationNode* Calibration);
};

#endif /* CAMERA_CALIBRATION_HPP_ */

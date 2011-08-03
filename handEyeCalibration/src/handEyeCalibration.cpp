#include "handEyeCalibration.hpp"

CalibrationNode::CalibrationNode(ros::NodeHandle& n):
	ROSNode (n),
	imageTransport (n)
{
	int width, height;
	this->ROSNode.getParam ("handEyeCalibration/width", width);
	this->ROSNode.getParam ("handEyeCalibration/height", height);
	image_size = cv::Size_<int> (width, height);

	int points_per_column, points_per_row;
	this->ROSNode.getParam ("handEyeCalibration/points_per_column", points_per_column);
	this->ROSNode.getParam ("handEyeCalibration/points_per_row", points_per_row);
	pattern = cv::Size_<int> (points_per_column, points_per_row);

	double pattern_width, pattern_height;
	this->ROSNode.getParam ("handEyeCalibration/pattern_width", pattern_width);
	this->ROSNode.getParam ("handEyeCalibration/pattern_height", pattern_height);
	pattern_size = cv::Size_<double> (pattern_width, pattern_height);

//	pattern_to_base = Eigen::Vector4d::Zero ();
//	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/x", pattern_to_base(0));
//	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/y", pattern_to_base(1));
//	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/z", pattern_to_base(2));
//	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/w", pattern_to_base(3));

	calibrationImageSubscriber = imageTransport.subscribe("/camera/image_color", 1, &CalibrationNode::imgCallback, this);
	robotPoseSubscriber = ROSNode.subscribe ("robotPose", 1, &CalibrationNode::poseCallback, this);
	cameraInfoSubscriber =ROSNode.subscribe ("/camera/camera_info", 1, &CalibrationNode::cameraInfoCallback, this);

	cameraMatrix = CreateMat(3, 3, CV_64FC1);
	distortionCoefficients = CreateMat(5, 1, CV_64FC1);

	cv::namedWindow (IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback (IMAGE_WINDOW, &CalibrationNode::mouseCallback, this);

	readPoseFlag = false;
}

CalibrationNode::~CalibrationNode ()
{
	cv::destroyWindow (IMAGE_WINDOW);
}

void CalibrationNode::imgCallback (const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr img_ptr;
	try
	{
		img_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR ("cv_bridge exeption: %s", e.what ());
	}

	image = img_ptr->image;
	cv::imshow (IMAGE_WINDOW, image);
	cv::waitKey (10);
}

void CalibrationNode::poseCallback (const geometry_msgs::PoseConstPtr& msg)
{
	if (readPoseFlag)
	{
		std::cout << "robot POSE read" << std::endl;
		readPoseFlag = false;
	}

}

void CalibrationNode::mouseCallback (int event, int x, int y, int flags, void* calibrationNode)
{

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		(static_cast<CalibrationNode*> (calibrationNode))->storeData ();

		break;

	case CV_EVENT_RBUTTONDOWN:
		std::cout << "CV_EVENT_RBUTTONDOWN" << std::endl;

		break;
	}
}



void CalibrationNode::cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr& msg){
	std::cout << msg->distortion_model << std::endl;

	// Intrinsic camera matrix for the raw (distorted) images.
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]

	cameraInfoSubscriber.shutdown();
}


int CalibrationNode::storeData ()
{
	cv::vector<cv::Point2f> corners;
	bool patternWasFound = cv::findChessboardCorners (image, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (patternWasFound)
	{
		robotPoseVector.push_back (robotPose);

		cv::Mat gray_image;
		gray_image.create (image_size, CV_8UC1);
		cv::cvtColor (image, gray_image, CV_BGR2GRAY, 0);

		cv::cornerSubPix (gray_image, corners, cv::Size (5, 5), cv::Size (-1, -1),
				cv::TermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners (image, pattern, corners, patternWasFound);
		cv::imshow (IMAGE_WINDOW, image);

		imagePoints.push_back (corners);

		std::cout << "The corner and the object points have been stored!\nPlease press a key to continue!\n";
		cv::waitKey ();

		return checkerboard_found;
	}
	else
	{
		std::cout << "No checkerboard has been found or it is not completely visible" << std::endl;
		cv::waitKey ();

		return checkerboard_not_found;
	}
}


//int CalibrationNode::getObjectPoints ()
//{
//	cv::vector<cv::Point3f> temp_vector;
//
//	for (int i = 0; i < pattern.height; i++)
//	{
//		for (int j = 0; j < pattern.width; j++)
//		{
//			cv::Point3f temp_point = cv::Point3f (j * pattern_size.width, i * pattern_size.height, 0.0);
//			temp_vector.push_back (temp_point);
//		}
//	}
//
//	objectPoints.push_back (temp_vector);
//
//	return objectPoints_stored;
//}

//cv::vector<cv::vector<cv::Point2f> > CalibrationNode::getImagePoints ()
//{
//	return imagePoints;
//}
//cv::vector<cv::vector<cv::Point3f> > CalibrationNode::getObjectPoints ()
//{
//	return objectPoints;
//}


int CalibrationNode::calibrateCamera ()
{
	cv::vector<cv::Mat> rvecs, tvecs;
	cv::calibrateCamera (objectPoints, imagePoints, image_size, camera_matrix, distortion_coefficients, rvecs, tvecs);

	return camera_calibrated;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "handEyeCalibrationNode"); // initialize the CMgreenBallDetector node

	ros::NodeHandle n; // declare a node handle

	CalibrationNode CalibrationObject (n); // start the ball detector node with the node handle n

	ros::spin();

	return 0;
}

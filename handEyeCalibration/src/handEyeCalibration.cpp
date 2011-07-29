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

	pattern_to_base = Eigen::Vector4d::Zero ();
	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/x", pattern_to_base(0));
	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/y", pattern_to_base(1));
	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/z", pattern_to_base(2));
	this->ROSNode.getParam ("handEyeCalibration/pattern_to_base/w", pattern_to_base(3));

	calibration_image_subscriber = imageTransport.subscribe("/camera/image_color", 1, &CalibrationNode::callback, this);

	pose_robot_subscriber = ROSNode.subscribe ("robot_pose", 1, &CalibrationNode::storePose, this);

	cv::namedWindow (IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback (IMAGE_WINDOW, &CalibrationNode::mouseCallback, (void*)this);
}

CalibrationNode::~CalibrationNode ()
{
	cv::destroyWindow (IMAGE_WINDOW);
}

void CalibrationNode::callback (const sensor_msgs::ImageConstPtr& msg)
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

void CalibrationNode::storePose (const geometry_msgs::PoseConstPtr& msg)
{
	if (pose_robot.position.x != msg->position.x
			|| pose_robot.position.y != msg->position.y
			|| pose_robot.position.z != msg->position.z
			|| pose_robot.orientation.x != msg->orientation.x
			|| pose_robot.orientation.y != msg->orientation.y
			|| pose_robot.orientation.z != msg->orientation.z
			|| pose_robot.orientation.w != msg->orientation.w)
	{
		pose_robot.position = msg->position;
		pose_robot.orientation = msg->orientation;

		std::cout << "New robot position: x = " << pose_robot.position.x
				<< " y = " << pose_robot.position.y
				<< " z = " << pose_robot.position.z << std::endl;
		std::cout << "New robot orientation: x = " << pose_robot.orientation.x
				<< " y = " << pose_robot.orientation.y
				<< " z = " << pose_robot.orientation.z
				<< " w = " << pose_robot.orientation.w << std::endl;

		std::cout << pose_robot_stored << ", robot pose stored.\n";
	}

	std::cout << pose_robot_not_stored << ", robot pose not stored.\n";
}

void CalibrationNode::mouseCallback (int event, int x, int y, int flags, void* param)
{
	CalibrationNode* Calibration = static_cast<CalibrationNode*>(param);

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		Calibration->storeCheckboard (Calibration);

		break;

	case CV_EVENT_RBUTTONDOWN:
		Calibration->calibrateCamera (Calibration);

		break;
	}
}

int CalibrationNode::storeCheckboard (CalibrationNode* Calibration)
{
	cv::vector<cv::Point2f> corners;
	bool patternWasFound = cv::findChessboardCorners (Calibration->image, Calibration->pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (patternWasFound)
	{
		geometry_msgs::Pose temp_pose;
		temp_pose.position = Calibration->pose_robot.position;
		temp_pose.orientation = Calibration->pose_robot.orientation;
		Calibration->pose_robot_vector.push_back (temp_pose);

		cv::Mat gray_image;
		gray_image.create (Calibration->image_size, CV_8UC1);
		cv::cvtColor (Calibration->image, gray_image, CV_BGR2GRAY, 0);

		cv::cornerSubPix (gray_image, corners, cv::Size (5, 5), cv::Size (-1, -1),
				cv::TermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners (Calibration->image, Calibration->pattern, corners, patternWasFound);
		cv::imshow (IMAGE_WINDOW, Calibration->image);

		Calibration->image_points.push_back (corners);

		Calibration->getObjectPoints (Calibration);

		std::cout << "The corner and the object points have been stored!\nPlease press a key to continue!\n";
		cv::waitKey ();

		return checkboard_found;
	}
	else
	{
		std::cout << "No checkboard has been found or it is not completely visible (or the key has not been pressed after the checkboard has been found)!\nPlease press a key to continue!\n";
		cv::waitKey ();

		return checkboard_not_found;
	}
}


int CalibrationNode::getObjectPoints (CalibrationNode* Calibration)
{
	cv::vector<cv::Point3f> temp_vector;

	for (int i = 0; i < Calibration->pattern.height; i++)
	{
		for (int j = 0; j < Calibration->pattern.width; j++)
		{
			cv::Point3f temp_point = cv::Point3f (j * Calibration->pattern_size.width, i * Calibration->pattern_size.height, 0.0);
			temp_vector.push_back (temp_point);
		}
	}

	Calibration->object_points.push_back (temp_vector);

	return object_points_stored;
}

cv::vector<cv::vector<cv::Point2f> > CalibrationNode::getImagePoints ()
{
	return image_points;
}
cv::vector<cv::vector<cv::Point3f> > CalibrationNode::getObjectPoints ()
{
	return object_points;
}


int CalibrationNode::calibrateCamera (CalibrationNode* Calibration)
{
	cv::vector<cv::Mat> rvecs, tvecs;
	cv::calibrateCamera (Calibration->getObjectPoints (), Calibration->getImagePoints (), Calibration->image_size,
			Calibration->camera_matrix, Calibration->distortion_coefficients, rvecs, tvecs);



//	cv::vector<cv::Mat> rmats;
//	for (int i = 0; i < (int)(rvecs.size ()); i++)
//	{
//		cv::Mat temp_mat;
//		cv::Rodrigues (rvecs[i], temp_mat);
//	}

	return Calibration->camera_calibrated;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CMgreenBallDetector"); // initialize the CMgreenBallDetector node

	ros::NodeHandle n; // declare a node handle

	CalibrationNode CalibrationObject (n); // start the ball detector node with the node handle n

	ros::spin(); // wait for callbacks until the node is destroyed

	return 0;
}

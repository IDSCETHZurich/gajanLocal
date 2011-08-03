#include "handEyeCalibration.hpp"
#define DEBUG 0

using namespace Eigen;

CalibrationNode::CalibrationNode(ros::NodeHandle& n):
	ROSNode (n),
	imageTransport (n)
{
	int width, height;
	ROSNode.getParam ("handEyeCalibration/width", width);
	ROSNode.getParam ("handEyeCalibration/height", height);
	image_size = cv::Size_<int> (width, height);

	int points_per_column, points_per_row;
	ROSNode.getParam ("handEyeCalibration/points_per_column", points_per_column);
	ROSNode.getParam ("handEyeCalibration/points_per_row", points_per_row);
	pattern = cv::Size_<int> (points_per_column, points_per_row);

	ROSNode.getParam ("handEyeCalibration/pattern_width", patternHeight);
	ROSNode.getParam ("handEyeCalibration/pattern_height", patternWidth);

	calibrationImageSubscriber = imageTransport.subscribe("/camera/image_color", 1, &CalibrationNode::imgCallback, this);
	robotPoseSubscriber = ROSNode.subscribe ("/msrCartPos", 1, &CalibrationNode::poseCallback, this);
	cameraInfoSubscriber =ROSNode.subscribe ("/camera/camera_info", 1, &CalibrationNode::cameraInfoCallback, this);

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
		rotationRB = Eigen::Quaternionf(
				Eigen::Quaternion<float>(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
		translationRB = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);

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

	cameraMatrix = cv::Mat(3, 3, CV_64F);// Intrinsic camera matrix for the raw (distorted) images.
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			cameraMatrix.at<double>(i,j) = msg->K[i*3+j];
	std::cout << "cameraMatrix" << std::endl << cameraMatrix << std::endl;

	distortionCoefficients = cv::Mat(5, 1, CV_64F);
	for(int i=0; i<3; i++)
		distortionCoefficients.at<double>(i,1) = msg->D[i];
	std::cout << "distortionCoefficients" << std::endl << distortionCoefficients << std::endl;
	std::cout << "/camera/camera_info successfully read out!" << std::endl;

	cameraInfoSubscriber.shutdown();
}


int CalibrationNode::storeData ()
{
	cv::vector<cv::Point2f> corners;
	bool patternWasFound = cv::findChessboardCorners (image, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
	if (patternWasFound)
	{
		readPoseFlag = true;
		do{
			ros::Duration(0.2).sleep();
			ROS_INFO("Waiting to read ROBOT pose ...");
		}while(readPoseFlag == false);

		cv::Mat gray_image;
		gray_image.create (image_size, CV_8UC1);
		cv::cvtColor (image, gray_image, CV_BGR2GRAY, 0);

		cv::cornerSubPix (gray_image, corners, cv::Size (5, 5), cv::Size (-1, -1),
				cv::TermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners (image, pattern, corners, patternWasFound);
		cv::imshow (IMAGE_WINDOW, image);

		//fill in imagePoints and objectPoints
		imagePoints = cv::Mat(corners.size(), 2, CV_32F);
		objectPoints = cv::Mat(corners.size(), 3, CV_32F);

		for(int i=0; i < (int)corners.size(); i++){
			imagePoints.at<float>(i,0) = corners[i].x;
			imagePoints.at<float>(i,1) = corners[i].y;
		}

		std::cout << "imagePoints" << std::endl << imagePoints << std::endl;

		for(int i=0; i < pattern.height; i++){
			for(int j=0; j < pattern.width; j++){
				objectPoints.at<float>(i*pattern.width+j, 0) = i*patternHeight;
				objectPoints.at<float>(i*pattern.width+j, 1) = j*patternWidth;
				objectPoints.at<float>(i*pattern.width+j, 2) = 0.0;
			}
		}
		std::cout << "objectPoints" << std::endl << objectPoints << std::endl;

		cv::Mat rvecs, tvecs;

		cv::solvePnP (objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvecs, tvecs);

		std::cout << "rvecs" << std::endl << rvecs << std::endl;
		std::cout << "tvecs" << std::endl << tvecs << std::endl;

		cv::Mat rotMat = cv::Mat(3, 3, CV_32F);
		cv::Rodrigues(rvecs, rotMat);


		for(int i=0; i < 3; i++)
			for(int j=0; j < 3; j++)
				rotationCB(i,j) = rotMat.at<float>(i,j);

		translationCB = Eigen::Vector3f(rvecs.at<float>(0,0), rvecs.at<float>(0,1), rvecs.at<float>(0,2));

		//pushing back data into vectors
		rotationRB_vec.push_back(rotationRB);
		translationRB_vec.push_back(translationRB);
		rotationCB_vec.push_back(rotationCB);
		translationCB_vec.push_back(translationCB);

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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "handEyeCalibrationNode"); // initialize the CMgreenBallDetector node

	ros::NodeHandle n; // declare a node handle

	CalibrationNode CalibrationObject (n); // start the ball detector node with the node handle n

	ros::spin();

	return 0;
}

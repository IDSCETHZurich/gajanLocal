#include "handEyeCalibration.hpp"
#define SIGLE_MEASUREMENT_DEBUG 0
#define ESTIMATION_DEBUG 1

using namespace Eigen;

CalibrationNode::CalibrationNode(ros::NodeHandle& n):
	ROSNode (n),
	imageTransport (n)
{
	int width, height;
	ROSNode.getParam ("handEyeCalibration/width", width);
	ROSNode.getParam ("handEyeCalibration/height", height);
	image_size = cv::Size_<int> (width, height);

	int points_per_column, points_per_row; //8,6
	ROSNode.getParam ("handEyeCalibration/points_per_column", points_per_column);
	ROSNode.getParam ("handEyeCalibration/points_per_row", points_per_row);
	pattern = cv::Size_<int> (points_per_column, points_per_row);

	ROSNode.getParam ("handEyeCalibration/pattern_width", patternHeight);
	ROSNode.getParam ("handEyeCalibration/pattern_height", patternWidth);

	calibrationImageSubscriber = imageTransport.subscribe("/camera/image_rect", 1, &CalibrationNode::imgCallback, this);
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
		ROS_ERROR ("cv_bridge exception: %s", e.what ());
	}

	image = img_ptr->image;
	cv::imshow (IMAGE_WINDOW, image);
	cv::waitKey (10);
}

void CalibrationNode::poseCallback (const geometry_msgs::PoseConstPtr& msg)
{

	robotPose = *msg;

}

void CalibrationNode::mouseCallback (int event, int x, int y, int flags, void* calibrationNode)
{

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		(static_cast<CalibrationNode*> (calibrationNode))->storeData ();
		break;

	case CV_EVENT_RBUTTONDOWN:
		std::cout << "Performing Estimation..." << std::endl;
		(static_cast<CalibrationNode*> (calibrationNode))->performEstimation();
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
//	for(int i=0; i<3; i++)
//		distortionCoefficients.at<double>(i,1) = msg->D[i];
	for(int i=0; i<5; i++)
		distortionCoefficients.at<double>(i,1) = 0.0;

	std::cout << "distortionCoefficients" << std::endl << distortionCoefficients << std::endl;
	std::cout << "/camera/camera_info successfully read out!" << std::endl;

	cameraInfoSubscriber.shutdown();
}


int CalibrationNode::storeData ()
{
	cv::vector<cv::Point2f> corners;
	bool patternWasFound = cv::findChessboardCorners (image, pattern, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
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

#if SIGLE_MEASUREMENT_DEBUG
		std::cout << "imagePoints" << std::endl << imagePoints << std::endl;
#endif

		// cout << "hight & width" << pattern.height << " : " << pattern.width << endl;

		//width - 8 - x
		//height - 6 - y
		//row by row, left to right in every row
		for(int i=0; i < pattern.height; i++){
			for(int j=0; j < pattern.width; j++){
				objectPoints.at<float>(i*pattern.width+j, 0) = j*patternWidth;
				objectPoints.at<float>(i*pattern.width+j, 1) = i*patternHeight;
				objectPoints.at<float>(i*pattern.width+j, 2) = 0.0;
			}
		}

#if SIGLE_MEASUREMENT_DEBUG
		std::cout << "objectPoints" << std::endl << objectPoints << std::endl;
#endif

		cv::Mat rvecs, tvecs;

		cv::solvePnP (objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvecs, tvecs);

#if ESTIMATION_DEBUG
		std::cout << "rvecs" << std::endl << rvecs << std::endl;
		std::cout << "tvecs" << std::endl << tvecs << std::endl;
#endif

		cv::Mat rotMat = cv::Mat(3, 3, CV_64F);
		cv::Rodrigues(rvecs, rotMat);

		// cout << "rotMat" << endl << rotMat << endl;

		for(int i=0; i < 3; i++)
			for(int j=0; j < 3; j++)
				rotationCB(i,j) = rotMat.at<double>(i,j);

		translationCB = Vector3f(tvecs.at<double>(0,0), tvecs.at<double>(0,1), tvecs.at<double>(0,2));

		//robotPose

		//Debug [KDL::Rotation vs Eigen::Matrix3f comparision]
//		KDL::Rotation tmp =
//				KDL::Rotation::Quaternion(robotPose.orientation.x,robotPose.orientation.y,robotPose.orientation.z,robotPose.orientation.w);
//
//		cout << "tmp(rotationRB)" << endl;
//		cout << tmp(0,0) << ", " << tmp(0,1) << ", " << tmp(0,2) << endl;
//		cout << tmp(1,0) << ", " << tmp(1,1) << ", " << tmp(1,2) << endl;
//		cout << tmp(2,0) << ", " << tmp(2,1) << ", " << tmp(2,2) << endl;
//		cout << endl;
		//end of Debug

		rotationRB = Quaternionf(robotPose.orientation.w, robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z);
		translationRB = Vector3f(robotPose.position.x, robotPose.position.y, robotPose.position.z);

		//pushing back data into vectors
		rotationRB_vec.push_back(rotationRB);
		translationRB_vec.push_back(translationRB);
		rotationCB_vec.push_back(rotationCB);
		translationCB_vec.push_back(translationCB);

		std::cout << "Checkerboard found. Measurements Updated." << std::endl;

#if ESTIMATION_DEBUG
	std::cout << "%Adding data #" << rotationRB_vec.size() << std::endl;
	std::cout << "rotRB" << rotationRB_vec.size() << " = [ " << rotationRB << " ]; " <<std::endl;
	std::cout << "transRB" << rotationRB_vec.size() << " = [ " << translationRB << " ]; " << std::endl;
	std::cout << "rotCB" << rotationRB_vec.size() << " = [ " << rotationCB << " ]; " << std::endl;
	std::cout << "transCB" << rotationRB_vec.size() << " = [ " << translationCB << " ]; " << std::endl;
#endif

		cv::waitKey ();

		return checkerboard_found;
	}
	else
	{
		std::cout << "No Checkerboard has been found !" << std::endl;
		cv::waitKey ();

		return checkerboard_not_found;
	}
}

void CalibrationNode::performEstimation(){
	if(rotationRB_vec.size() < 5 ){
		std::cout << "Insufficient data" << std::endl;
		return;
	}

	//perform least squares estimation
	Matrix3f Ai, Bi, M;
	M.setZero();
	Vector3f ai, bi;
	for(int i=0; i < (int)rotationRB_vec.size()-1; i++){
		Ai = rotationRB_vec[i+1].inverse() * rotationRB_vec[i];
		Bi = rotationCB_vec[i+1] * rotationCB_vec[i].inverse();
		ai = getLogTheta(Ai);
		bi = getLogTheta(Bi);
		M += bi*ai.transpose();
	}//end of for(.. i < rotationRB_vec.size()-1; ..)

#if ESTIMATION_DEBUG
	std::cout << "M = [ " << M << " ]; ";
#endif

	EigenSolver<Matrix3f> es(M);
	Matrix3f D = es.eigenvalues().real().asDiagonal();
	Matrix3f V = es.eigenvectors().real();

	Matrix3f Lambda = D.inverse().array().sqrt();

	Matrix3f x_est = V * Lambda * V.inverse() * M.transpose();

	std::cout << "x_est = [ " << x_est  << " ]; ";

}

Vector3f CalibrationNode::getLogTheta(Matrix3f R){
//	theta =  acos((trace(R)-1)/2);
//	logTheta = 0.5*theta/sin(theta)*(R-R');
//	result = [logTheta(3,2);logTheta(1,3);logTheta(2,1)];

	//Assumption R is never an Identity
	float theta = acos((R.trace()-1)/2);
	Matrix3f logTheta = 0.5*theta/sin(theta)*(R-R.transpose());
	return Vector3f(logTheta(2,1), logTheta(0,2), logTheta(1,0));

//	AngleAxis<float> aa;
//	aa.fromRotationMatrix(R);
//	return aa.axis()*aa.angle();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "handEyeCalibrationNode"); // initialize the CMgreenBallDetector node

	ros::NodeHandle n; // declare a node handle

	CalibrationNode CalibrationObject (n); // start the ball detector node with the node handle n

	ros::spin();

	return 0;
}


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace cv;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  vector<Vec3f> circles;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/ballDetectorOut", 1);
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image = ~(cv_ptr->image);
    cv::threshold(cv_ptr->image, cv_ptr->image, 180, 255, cv::THRESH_TOZERO);

//    //perform erosion and dilation
//    int type = cv::MORPH_CROSS;
//    int size = 20;
//    cv::Mat element = cv::getStructuringElement( type,cv::Size( 2*size + 1, 2*size+1 ), cv::Point( size, size ) );
//    cv::erode(cv_ptr->image, cv_ptr->image,element);
//    cv::dilate(cv_ptr->image, cv_ptr->image,element);

//    //Do canny edge detection
    cv::Canny(cv_ptr->image, cv_ptr->image, 40.0, 120.0, 3);
//
    circles.clear();
    HoughCircles(cv_ptr->image, circles, CV_HOUGH_GRADIENT, 2, cv_ptr->image.rows/4, 200, 100, 0,  100 );



    std::cout << "# of circles found: " << circles.size() << std::endl;


    for( size_t i = 0; i < circles.size(); i++ )
	{
		 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		 int radius = cvRound(circles[i][2]);
		 // draw the circle center
		 circle( cv_ptr->image, center, 3, Scalar(255,255,255));
		 // draw the circle outline
		 circle( cv_ptr->image, center, radius, Scalar(255,255,255));
	}



    //cvMoments


    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ballDetector");
  ImageConverter ic;
  ros::spin();
  return 0;
}

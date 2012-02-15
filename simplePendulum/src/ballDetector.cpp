#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Point.h"

namespace enc = sensor_msgs::image_encodings;

using namespace cv;
using namespace std;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  geometry_msgs::Point pendTip;
  ros::Publisher pendTipPub;


  cv::Mat element;
  cv::Moments tmp_moments;
  
  //parameters needed to estimate the pendulumTip

  //camera relative position
  double ox, oy, oz, f, pSize, sWidth, sHeight, l;

  //tmp variables
  double px, py, t;
  double a, b, c;

  std::vector<double> pos_km1, pos_km2, pos_km3, pos_km4, pos_km5;


public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/ballDetectorOut", 1);
    pendTipPub = nh_.advertise<geometry_msgs::Point>("pendPosFromROS", 2, true);

    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);

    int type = cv::MORPH_RECT;
	int size = 9;
	element = cv::getStructuringElement( type,cv::Size( 2*size + 1, 2*size+1 ), cv::Point( size, size ) );

    ox =  0.007;
    oy = -0.168;
    oz =  0.011;

    pSize = 9.9e-06;

    //focal lenght
    f = 504*pSize;

    sWidth = 659;
    sHeight = 493;

	//lenght of the pendulem
    l = 0.96;

    pos_km1 = std::vector<double>(2,0.0);
    pos_km2 = std::vector<double>(2,0.0);
    pos_km3 = std::vector<double>(2,0.0);
    pos_km4 = std::vector<double>(2,0.0);
    pos_km5 = std::vector<double>(2,0.0);

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
    cv::threshold(cv_ptr->image, cv_ptr->image, 195, 255, cv::THRESH_TOZERO);


    //perform erosion and dilation

    cv::erode(cv_ptr->image, cv_ptr->image,element);
    cv::dilate(cv_ptr->image, cv_ptr->image,element);

    //cvMoments
    tmp_moments = cv::moments( cv_ptr->image, 0 );

    //process only if the ball is detected
    if(tmp_moments.m00 > 0.0){
		Point center(tmp_moments.m10/tmp_moments.m00, tmp_moments.m01/tmp_moments.m00);
		circle( cv_ptr->image, center, 10, Scalar(255,255,255));

		//calculate pendTip
		px = pSize*(tmp_moments.m10/tmp_moments.m00 - sWidth/2);
		py = pSize*(tmp_moments.m01/tmp_moments.m00 - sHeight/2);

		a = px*px + py*py + f*f;
		b = px*ox + py*oy + f*oz;
		c = ox*ox + oy*oy + oz*oz - l*l;

		t = (-b + sqrt(b*b - 4*a*c))/(2*a);

		pendTip.x = ox + t * px;
		pendTip.y = oy + t * py;
		pendTip.z = 0.0; // since we care only about the projection;

		//publish pendTip
		pendTipPub.publish(pendTip);

    }else{
    	std::cout << "Could not find the ball" << std::endl;
    }
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

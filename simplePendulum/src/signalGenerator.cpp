#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#define PI 3.14159265


int main(int argc, char **argv)
{

  ros::init(argc, argv, "");

  ros::NodeHandle n;
  ros::Publisher posePub = n.advertise<geometry_msgs::Point>("pendPosFromROS", 2, true);
  ros::Rate loop_rate(89);

  geometry_msgs::Point tmpPoint;
  double f = 2;
  double a = 0.05; // 5 cm
  double na = 0.02; // 3 mm

  while (ros::ok())
  {

	tmpPoint.x = a*sin(2*PI*f*ros::Time::now().toSec()) + na *rand()/(double)RAND_MAX - na/2;
	tmpPoint.y = a*sin(2*PI*f*ros::Time::now().toSec()) + na *rand()/(double)RAND_MAX - na/2;
	tmpPoint.z = 0;

	posePub.publish(tmpPoint);
	loop_rate.sleep();
  }


  return 0;
}

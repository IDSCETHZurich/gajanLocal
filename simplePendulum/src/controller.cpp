#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher pointPub = n.advertise<geometry_msgs::Point>("basePoint", 2, true);

  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {

    geometry_msgs::Point point;

    double Rx = 0.05, Ry = 0.05;
    point.x = -Rx + 2*Rx*(double)rand()/(double)RAND_MAX;
    point.y = -Ry + 2*Ry*(double)rand()/(double)RAND_MAX;
    point.z = 0.0;

    pointPub.publish(point);

    std::cout << "Sending to hPlaneController "
    			<< point.x << ", " << point.y << std::endl;

    loop_rate.sleep();
  }

  return 0;
}

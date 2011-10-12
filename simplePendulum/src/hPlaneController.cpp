#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"


ros::Publisher posePub;
#define orignX 0.570
#define orignY 0.0
#define orignZ 0.740


void pointCallback(const geometry_msgs::Point::ConstPtr& point){
	geometry_msgs::Pose pose;

	//Always point up
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 1.0;

	pose.position.x = orignX + point->x;
	pose.position.y = orignY + point->y;
	pose.position.z = orignZ;

	posePub.publish(pose);

	std::cout << "Received point from controller, sending it to OROCOS "
			<< pose.position.x << ", " << pose.position.y << std::endl;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  //move the robot to the horizontal plane origin
  posePub = n.advertise<geometry_msgs::Pose>("poseDsr", 2, true);
  ros::Subscriber sub = n.subscribe("basePoint", 2, pointCallback);

  geometry_msgs::Pose pose;

  // Always point up
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  pose.position.x = orignX;
  pose.position.y = orignY;
  pose.position.z = orignZ;

  posePub.publish(pose);

  ros::spin();

  return 0;
}

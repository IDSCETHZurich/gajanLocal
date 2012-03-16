#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define PI 3.14159265
#define RATE 10

int main(int argc, char **argv)
{

  ros::init(argc, argv, "viz");
  ros::NodeHandle _nh;

  double theta = 0.0;

  ros::Publisher vis_pub = _nh.advertise<visualization_msgs::MarkerArray>( "re_visualization_marker_array", 0 );

  visualization_msgs::Marker marker1;
  marker1.header.frame_id = "/arm_0_link";
  marker1.header.stamp = ros::Time();
  marker1.ns = "articulationNS";
  marker1.id = 0;
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.action = visualization_msgs::Marker::ADD;
  //marker1.pose.position.x = 1;
  //marker1.pose.position.y = 1;
  marker1.pose.position.z = 1;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.scale.x = .3;
  marker1.scale.y = .3;
  marker1.scale.z = .3;
  marker1.color.a = 1.0;
  marker1.color.r = 0.0;
  marker1.color.g = 1.0;
  marker1.color.b = 0.0;

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "/arm_0_link";
  marker2.header.stamp = ros::Time();
  marker2.ns = "articulationNS";
  marker2.id = 1;
  marker2.type = visualization_msgs::Marker::ARROW;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = 0;
  marker2.pose.position.y = 0;
  marker2.pose.position.z = 0;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = -1./std::sqrt(2);
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1./std::sqrt(2);
  marker2.scale.x = 2;
  marker2.scale.y = 2;
  marker2.scale.z = 2;
  marker2.color.a = 1.0;
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "/arm_0_link";
  marker3.header.stamp = ros::Time();
  marker3.ns = "articulationNS";
  marker3.id = 2;
  marker3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.position.x = 0;
  marker3.pose.position.y = 0;
  marker3.pose.position.z = 1;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;
  marker3.scale.x = 0.5;
  marker3.scale.y = 0.5;
  marker3.scale.z = 0.5;
  marker3.color.a = 1.0;
  marker3.color.r = 1.0;
  marker3.color.g = 1.0;
  marker3.color.b = 1.0;


  ros::Rate loop_rate(RATE);

  while (ros::ok())
  {
    marker1.pose.position.x = 1*std::sin(theta);
    marker1.pose.position.y = 1*std::cos(theta);

    marker3.pose.position.x = marker1.pose.position.x;
    marker3.pose.position.y = marker1.pose.position.y;

    std::stringstream s;
    s << "(" << marker3.pose.position.x << ", " << marker3.pose.position.y << ")" ;
    marker3.text = s.str();

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker1);
    ma.markers.push_back(marker2);
    ma.markers.push_back(marker3);

    //ROS_INFO("Publishing marker1");
    vis_pub.publish( ma );

    ros::spinOnce();
    loop_rate.sleep();

    theta = theta + PI / (RATE*2);
  }


  return 0;
}

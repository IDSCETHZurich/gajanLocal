#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "kdl/frames.hpp"

#define PI 3.14159265

ros::Publisher vis_pub;

bool publishMarkerArray(geometry_msgs::Pose axis, std::vector<geometry_msgs::Pose> trajectoyPoints, std::string refFrameName){
	visualization_msgs::MarkerArray tmpMarkerArray;

	KDL::Rotation axisR = KDL::Rotation::Quaternion(axis.orientation.x, axis.orientation.y,axis.orientation.z,axis.orientation.w);
	KDL::Rotation deltaR = KDL::Rotation::Quaternion(0.0,std::sin(-PI/4),0.0,std::cos(-PI/4));
	KDL::Rotation markerR = axisR*deltaR;

	visualization_msgs::Marker axisMarker;
	axisMarker.header.frame_id = refFrameName;
	axisMarker.header.stamp = ros::Time();
	axisMarker.ns = "articulationNS";
	axisMarker.id = 0;
	axisMarker.type = visualization_msgs::Marker::ARROW;
	axisMarker.action = visualization_msgs::Marker::ADD;
	axisMarker.scale.x = 2;
	axisMarker.scale.y = 2;
	axisMarker.scale.z = 2;
	axisMarker.color.a = 1.0;
	axisMarker.color.r = 0.0;
	axisMarker.color.g = 1.0;
	axisMarker.color.b = 0.0;

	//std::cout << "Before: "<<axisMarker.pose.orientation.x << " " << axisMarker.pose.orientation.y << " " << axisMarker.pose.orientation.z << " " << axisMarker.pose.orientation.w << std::endl;
	markerR.GetQuaternion(axisMarker.pose.orientation.x,axisMarker.pose.orientation.y,axisMarker.pose.orientation.z,axisMarker.pose.orientation.w);
	//std::cout << "After: "<<axisMarker.pose.orientation.x << " " << axisMarker.pose.orientation.y << " " << axisMarker.pose.orientation.z << " " << axisMarker.pose.orientation.w << std::endl;


	visualization_msgs::Marker axisText;
	axisText.header.frame_id = refFrameName;
	axisText.header.stamp = ros::Time();
	axisText.ns = "articulationNS";
	axisText.id = 1;
	axisText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	axisText.action = visualization_msgs::Marker::ADD;
	axisText.scale.x = 0.5;
	axisText.scale.y = 0.5;
	axisText.scale.z = 0.5;
	axisText.color.a = 1.0;
	axisText.color.r = 1.0;
	axisText.color.g = 1.0;
	axisText.color.b = 1.0;

	visualization_msgs::Marker trajPoint;
	trajPoint.header.frame_id = refFrameName;
	trajPoint.header.stamp = ros::Time();
	trajPoint.ns = "articulationNS";
	trajPoint.type = visualization_msgs::Marker::SPHERE;
	trajPoint.action = visualization_msgs::Marker::ADD;
	trajPoint.scale.x = .3;
	trajPoint.scale.y = .3;
	trajPoint.scale.z = .3;
	trajPoint.color.a = 1.0;
	trajPoint.color.r = 0.0;
	trajPoint.color.g = 1.0;
	trajPoint.color.b = 0.0;

	axisText.pose = axis;
	//std::stringstream s;
	//s << "(" << axisText.pose.position.x << ", " << axisText.pose.position.y << ")" ;
	axisText.text = "Axis of Rotation";

	tmpMarkerArray.markers.push_back(axisMarker);
	tmpMarkerArray.markers.push_back(axisText);

	for(int i=0; i<(int)trajectoyPoints.size(); i++){
		trajPoint.pose = trajectoyPoints[i];
		trajPoint.id = 2 + i;
		tmpMarkerArray.markers.push_back(trajPoint);
	}

	ROS_INFO("Publishing MarkerArray: Size=%d",(int)tmpMarkerArray.markers.size());
	vis_pub.publish( tmpMarkerArray );

	return true;
}


int main(int argc, char **argv)
{
  //some housekeeping
  geometry_msgs::Pose axis;
  axis.orientation.x = 0.0;
  axis.orientation.y = 0.0;
  axis.orientation.z = 0.0;
  axis.orientation.w = 1.0;
  std::vector<geometry_msgs::Pose> trajectoyPoints;

  for(int i=0; i < 10; i++){
	  geometry_msgs::Pose tmpPose;
	  tmpPose.position.x = 10*std::sin(i*PI/20.);
	  tmpPose.position.y = 10*std::cos(i*PI/20.);
	  trajectoyPoints.push_back(tmpPose);
  }

  ros::init(argc, argv, "viz");
  ros::NodeHandle _nh;
  vis_pub = _nh.advertise<visualization_msgs::MarkerArray>( "re_visualization_marker_array", 0 );

	ros::Rate loop_rate(1);

	while (ros::ok())
	{
		publishMarkerArray(axis, trajectoyPoints, "/arm_0_link");
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}

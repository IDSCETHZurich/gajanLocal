#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "KinectTF_listener");

	ros::NodeHandle node;

	//ros::service::waitForService("service_name");
  	tf::TransformListener listener;

  	ros::Rate rate(1.0);
	while (node.ok()){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/torso", "/left_hand", ros::Time(0), transform);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		cout << transform.getOrigin().getX() << endl;
		rate.sleep();
	}
	return 0;	
};

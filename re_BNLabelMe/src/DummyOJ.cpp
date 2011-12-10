#include "ros/ros.h"
#include <re_kinect_object_detector/DetectionResult.h>
#include <re_kinect_object_detector/OrderedList.h>

using namespace std;

class DummyOJ {
public:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	DummyOJ(){
		pub = n.advertise<re_kinect_object_detector::DetectionResult>("re_kinect/detection_results", 1);
		sub = n.subscribe("re_kinect/orderedList", 1, &DummyOJ::orderedListCallback, this);
	};

	bool publishUnorderedListWithEvidence(){
		re_kinect_object_detector::DetectionResult resultMsg;;
		std::vector<string> uList;
		std::vector<string> evidence;

		uList.push_back("Gajan");
		uList.push_back("Kalai");

		evidence.push_back("Gajan");

		resultMsg.FullObjectList = uList;
		resultMsg.DetectedObjectList = evidence;

		pub.publish(resultMsg);

		return true;
	};

	void orderedListCallback(const re_kinect_object_detector::OrderedListConstPtr &msg){
		std::cout << "Received message" << std::endl;
	};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummyOJNode");
  DummyOJ doj;

  for(int i=0; i < 50; i++){
	  doj.publishUnorderedListWithEvidence();
	  ros::spinOnce();
  }

  return 0;
}

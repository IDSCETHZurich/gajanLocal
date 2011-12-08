#include "ros/ros.h"
#include "re_BNLabelMe/orderedList.h"
#include "re_BNLabelMe/unorderedListwithEvidence.h"

using namespace std;

class DummyOJ {
public:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	DummyOJ(){
		pub = n.advertise<re_BNLabelMe::unorderedListwithEvidence>("unorderedListwithEvidence", 1);
		sub = n.subscribe("orderedList", 1, &DummyOJ::orderedListCallback, this);
	};

	bool publishUnorderedListWithEvidence(){
		re_BNLabelMe::unorderedListwithEvidence uListEvidenceMsg;
		std::vector<string> uList;
		std::vector<string> evidence;

		uList.push_back("Gajan");
		uList.push_back("Kalai");

		evidence.push_back("Gajan");

		uListEvidenceMsg.uList = uList;
		uListEvidenceMsg.evidence = evidence;

		pub.publish(uListEvidenceMsg);

		return true;
	};

	void orderedListCallback(const re_BNLabelMe::orderedListConstPtr& msg){
		std::cout << "Received message" << std::endl;
	};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummyOJNode");
  DummyOJ doj;

  for(int i=0; i < 50; i++){
	  doj.publishUnorderedListWithEvidence();
	  ros::Duration(2.0).sleep();
  }

  return 0;
}

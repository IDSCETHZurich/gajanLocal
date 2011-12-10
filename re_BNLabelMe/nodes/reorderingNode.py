#!/usr/bin/env python
import roslib; roslib.load_manifest('re_BNLabelMe')
import rospy
from std_msgs.msg import String
from re_kinect_object_detector.msg import DetectionResult
from re_kinect_object_detector.msg import OrderedList

class reorderingClass:
	def __init__(self):
		rospy.Subscriber("re_kinect/detection_results", DetectionResult, self.callback)
		self.pub = rospy.Publisher('re_kinect/orderedList', OrderedList)
	
	def callback(self, data):
    		rospy.loginfo(rospy.get_name()+" uList: %s, evidence: %s",data.FullObjectList,data.DetectedObjectList)
		self.pub.publish([2,1])

if __name__ == '__main__':
	rospy.init_node('reorderingNode', anonymous=True)
 	roc = reorderingClass()
	rospy.spin()

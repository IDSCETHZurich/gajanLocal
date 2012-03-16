#!/usr/bin/env python
import roslib; roslib.load_manifest('re_BNLabelMe')
import rospy
from std_msgs.msg import String

from re_kinect_object_detector.msg import DetectionResult
from re_kinect_object_detector.msg import OrderedList

class reorderingClass:
	def __init__(self):

		rospy.init_node('reorderingNode', anonymous=True)
		rospy.Subscriber("re_kinect/detection_results", DetectionResult, self.callback)
		

def callback(data):
    rospy.loginfo(rospy.get_name()+" uList: %s, evidence: %s",data.FullObjectList,data.DetectedObjectList)

def listener():
    
    rospy.Subscriber("re_kinect/detection_results", DetectionResult, callback)
    rospy.spin()


if __name__ == '__main__':
	rospy.init_node('reorderingNode', anonymous=True)
 	roc = reorderingClass()
	rospy.spin()

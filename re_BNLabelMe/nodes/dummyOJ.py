#!/usr/bin/env python
import roslib; roslib.load_manifest('re_BNLabelMe')
import rospy
from std_msgs.msg import String
from re_kinect_object_detector.msg import DetectionResult
from re_kinect_object_detector.msg import OrderedList

class dummyOJ:
	def __init__(self):
		rospy.Subscriber('re_kinect/orderedList', OrderedList, self.callback)
		self.pub = rospy.Publisher("re_kinect/detection_results",  DetectionResult)
	
	def callback(self, data):
    		rospy.loginfo(rospy.get_name()+" reordered List: %s",data.FullOrderedObjectList)

	def publishDetectionResult(self):
		result = DetectionResult()
		result.FullObjectList = ['window','cupboard','sink','refrigerator','bottle']
		result.DetectedObjectList = ['sink', 'window']		
		self.pub.publish(result)

if __name__ == '__main__':
	rospy.init_node('reorderingNode', anonymous=True)
 	roc = dummyOJ()
	r = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		roc.publishDetectionResult()
		r.sleep()


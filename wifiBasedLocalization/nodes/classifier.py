#!/usr/bin/env python
import roslib; roslib.load_manifest('wifiBasedLocalization')

from wifiBasedLocalization.srv import *
from std_msgs.msg import String
import rospy

pub = rospy.Publisher('lastClassifiedLocation', String)

def localize(req):
	print 'Entering localize function'
	print str(len(req.addresses))	
	pub.publish('RobotRoom')
	return 'correctLabel'
	
if __name__ == "__main__":
	rospy.init_node('classifier')
	s = rospy.Service('wifiBasedLocationClassifier', Localize, localize)
	print 'Initialized wifiBasedLocationClassifier node' 
	rospy.spin()

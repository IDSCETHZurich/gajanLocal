#!/usr/bin/env python
import roslib; roslib.load_manifest('wifiBasedLocalization')

from wifiBasedLocalization.srv import *
import rospy

def localize(req):
	print 'Entering localize function'
	print str(len(req.addresses))
	return 'correctLabel'
	
if __name__ == "__main__":
	rospy.wait_for_service('wifiBasedLocationClassifier')
	try:
		localize = rospy.ServiceProxy('wifiBasedLocationClassifier', Localize)
		label = localize(['21:23', '23:78'], [-40.0,-30.0])
		print label
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


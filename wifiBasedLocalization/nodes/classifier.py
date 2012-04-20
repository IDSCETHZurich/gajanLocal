#!/usr/bin/env python
import roslib; roslib.load_manifest('wifiBasedLocalization')

from wifiBasedLocalization.srv import *
from std_msgs.msg import String
import rospy

import os.path
import pickle
from math import pow

pub = rospy.Publisher('lastClassifiedLocation', String)

def localize(req):
	print 'Entering localize function'
	print str(len(req.addresses))
	
	lowestPower = -100	
	if os.path.exists('centroids.pickle'):
		centroids = pickle.load(open('centroids.pickle','rb'))
	else:
		print 'centroids.pickle can not be read'
		estimatedModel = 'None'
		return estimatedLabel

	distances = []
	for centroid in centroids:
		featureVector = []
		for feature in centroid['featureList']:
			apExist = False;
			for i,address in enumerate(req.addresses):
				if address == feature:
					featureVector.append(req.strengths[i])
					apExist = True
			if apExist == False:						
				featureVector.append(lowestPower)
			distance = 0.0
			for i in range(len(featureVector)):
				distance = distance + pow(featureVector[i]-centroid['strenghtList'][i],2)
		distances.append(distance)

	estimatedLabel = centroids[distances.index(min(distances))]['label']
	print estimatedLabel	

	numOfFeatures = len(centroids[0]['featureList'])
	pub.publish(estimatedLabel)
	return estimatedLabel
	
if __name__ == "__main__":
	rospy.init_node('classifier')
	s = rospy.Service('wifiBasedLocationClassifier', Localize, localize)
	print 'Initialized wifiBasedLocationClassifier node' 
	rospy.spin()

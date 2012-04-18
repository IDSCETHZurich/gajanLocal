#!/usr/bin/env python 
import os.path
import pickle
from singleMsr import singleMsr
from math import pow

if __name__  ==  '__main__':

	lowestPower = -100	
	if os.path.exists('data.pickle') and os.path.exists('centroids.pickle'):
		data = pickle.load(open('data.pickle','rb'))
		centroids = pickle.load(open('centroids.pickle','rb'))
		# ^ data is list of dictionaries with the following keys 'id','label','measurements'
		
	else:
		raise SystemExit('data.pickle/centroids.pickle does not exist.')
		

	numOfFeatures = len(centroids[0]['featureList'])
	print numOfFeatures
	for dataPoint in data:
		print '--------------------------------'
		# print dataPoint['measurements'] # <- list of singleMsr classes
		distances = []
		for centroid in centroids:
			featureVector = []
			for feature in centroid['featureList']:
				apExist = False;
				for msr in dataPoint['measurements']:
					if msr.address == feature:
						featureVector.append(float(msr.signalStrength))
						apExist = True
				if apExist == False:						
					featureVector.append(lowestPower)
			#print centroid['label']
			#print featureVector
			#print centroid['strenghtList']
			#print '------'
			distance = 0.0
			for i in range(len(featureVector)):
				distance = distance + pow(featureVector[i]-centroid['strenghtList'][i],2)
			distances.append(distance)

		print 'dataPoint Label: '+dataPoint['label']
		print 'estimated Label: '+centroids[distances.index(min(distances))]['label']

				
			#Calculate the feature vector of the dataPoint
			#print centroid['label']







	

#!/usr/bin/env python 
import os.path
import pickle
from singleMsr import singleMsr

if __name__  ==  '__main__':	
	if os.path.exists('data.pickle') and os.path.exists('centroids.pickle'):
		data = pickle.load(open('data.pickle','rb'))
		centroids = pickle.load(open('centroids.pickle','rb'))
		# ^ data is list of dictionaries with the following keys 'id','label','measurements'
		
	else:
		raise SystemExit('data.pickle/centroids.pickle does not exist.')
		

	numOfFeatures = len(centroids[0]['featureList'])
	print numOfFeatures
	for dataPoint in data:
		distancespa
		
		for centroid in centroids:
			featureVector = []
			for msr in dataPoint['measurements']:
				pass
			#Calculate the feature vector of the dataPoint
			#print centroid['label']







	

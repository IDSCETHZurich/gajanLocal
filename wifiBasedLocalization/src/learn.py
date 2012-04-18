#!/usr/bin/env python 
import os.path
import pickle
from singleMsr import singleMsr

#class singleMsr:
#	def __init__(self, address, signalStrength, ESSID):
#		self.address = address
#		self.signalStrength = signalStrength
#		self.ESSID = ESSID
#	def __repr__(self):
#		return self.address+' '+str(self.signalStrength)

if __name__  ==  '__main__':	
	if os.path.exists('data.pickle'):
		data = pickle.load(open('data.pickle','rb'))
		# ^ data is list of dictionaries with the following keys 'id','label','measurements'
	else:
		raise SystemExit('data.pickle does not exist.')
		

	print '# of data points:'+str(len(data))
	
	labels = set()
	for dataPoint in data:
		labels.add(dataPoint['label'])

	print labels 
	
	centroids = []
	numOfFeatures = 10

	for label in labels:
		apStr = {} #Strength
		apCnt = {} #Counts
		for dataPoint in data:
			if dataPoint['label']==label:
				for msr in dataPoint['measurements']:
					if apStr.get(msr.address, False) is False:
						apStr[msr.address]=msr.signalStrength
						apCnt[msr.address] = 1
					else: 
						apStr[msr.address]=apStr[msr.address]+msr.signalStrength
						apCnt[msr.address] = apCnt[msr.address]+1
		# Average
		for key in apStr:
			apStr[key]=float(apStr[key])/apCnt[key]

		featureList = []
		strenghtList = []
		for key in sorted(apStr,key=apStr.get, reverse=True)[0:numOfFeatures]:
			featureList.append(key)
			strenghtList.append(apStr[key])
		
		centroids.append({'label':label, 'featureList':featureList,'strenghtList':strenghtList})
		
	print centroids

	pickle.dump(centroids,open('centroids.pickle','wb'))





	

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

	

#!/usr/bin/env python 
import subprocess 
import re 
import os.path
import pickle
from singleMsr import singleMsr

if __name__  ==  '__main__':
	proc = subprocess.Popen('iwlist eth2 scanning', shell=True, stdout=subprocess.PIPE, ) 
	stdout_str = proc.communicate()[0] 
	stdout_list=stdout_str.split('\n') 
	matchAdd = False
	matchSig = False
	matchESSID = False
	measurements = []
	for line in stdout_list: 
		line=line.strip() 
		if not matchAdd:
			matchAdd=re.search('Address: (\S+)',line) 
		if not matchSig:
			matchSig=re.search('Signal level:(\S+) dBm',line)
		if not matchESSID:
			matchESSID=re.search('ESSID:"(\S+)"',line) 
		if (matchAdd and matchSig and matchESSID):
			measurements.append(singleMsr(matchAdd.group(1),int(matchSig.group(1)),matchESSID.group(1)))
			matchAdd = False
			matchSig = False
			matchESSID = False

	#print measurements sorted wrt signal strength
	for msr in sorted(measurements, key=lambda msr:msr.signalStrength, reverse=True):
		print msr.address + ' ' + str(msr.signalStrength)
	
	if os.path.exists('data.pickle'):
		data = pickle.load(open('data.pickle','rb'))
		# ^ data is list of dictionaries with the following keys 'id','label','measurements'
	else:
		data = []

	label = raw_input('Enter label for the measurement:')
	data.append({'id':len(data),'label':label,'measurements':measurements})

	pickle.dump(data,open('data.pickle','wb'))
	print '# of data points:'+str(len(data))
	print 'Done'

	

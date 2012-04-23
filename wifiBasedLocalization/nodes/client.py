#!/usr/bin/env python
import ServiceAPI

import time
import subprocess 
import re 

def localize(req):
	print 'Entering localize function'
	print str(len(req.addresses))
	return 'correctLabel'
	
def getReading():
	proc = subprocess.Popen('sudo iwlist eth2 scanning', shell=True, stdout=subprocess.PIPE, ) 
	stdout_str = proc.communicate()[0] 
	stdout_list=stdout_str.split('\n') 
	matchAdd = False
	matchSig = False
	matchESSID = False
	address = []
	strength = []
	for line in stdout_list: 
		line=line.strip() 
		if not matchAdd:
			matchAdd=re.search('Address: (\S+)',line) 
		if not matchSig:
			matchSig=re.search('Signal level:(\S+) dBm',line)
		if not matchESSID:
			matchESSID=re.search('ESSID:"(\S+)"',line) 
		if (matchAdd and matchSig and matchESSID):
			address.append(matchAdd.group(1))
			strength.append(float(matchSig.group(1)))
			matchAdd = False
			matchSig = False
			matchESSID = False
	return (address,strength)

if __name__ == "__main__":
	#create environment 
	envID = ServiceAPI.changeEnv(nodesToAdd=[('wifiBasedLocalization/classifier.py', None)])
<<<<<<< HEAD
	
	#address, strength = getReading()
	address = ['00:03:52:5C:28:B1', '00:03:52:5C:28:B2', '00:03:52:5C:28:B3', '00:0F:61:86:EC:21', '00:0F:61:86:EC:22', '00:0F:61:86:EC:23', '00:03:52:5C:26:C0', '00:03:52:5C:26:C1', '00:03:52:5C:26:C2', '00:03:52:5C:26:C3', '00:0F:61:86:EC:20', '00:0F:61:B5:33:98']
        strength = [-85.0, -84.0, -83.0, -57.0, -57.0, -57.0, -73.0, -73.0, -72.0, -72.0, -68.0, -87.0]
	
	try:
		#build message
		msg = { 'addresses' : address, 'strengths' : strength }
=======

	address, strength = getReading()
	try:
		#build message
		msg = { 'addresses' : address, 'strengths' : strength }
		print msg
		#localize = rospy.ServiceProxy('wifiBasedLocationClassifier', Localize)
>>>>>>> 6a506141b4cfa0dcf85fd56e0a33a621c929d1dc

		#add task
		taskID = ServiceAPI.addTask(envID, 'wifiBasedLocalization/wifiBasedLocationClassifier', msg)

		#get task result
		(status, result) = ServiceAPI.getTask(envID, taskID, 15)
		
		#print result
		if status == 'completed':
			print result['label']
		else:
			print '{0}: {1}'.format(status, result)
	except SercviceAPI.RequestError as e:
		print "Service call failed: %s"%e


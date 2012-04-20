#!/usr/bin/env python
import roslib; roslib.load_manifest('wifiBasedLocalization')

from wifiBasedLocalization.srv import *
import rospy

import subprocess 
import re 

def localize(req):
	print 'Entering localize function'
	print str(len(req.addresses))
	return 'correctLabel'
	
def getReading():
	proc = subprocess.Popen('sudo iwlist eth1 scanning', shell=True, stdout=subprocess.PIPE, ) 
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
	rospy.wait_for_service('wifiBasedLocationClassifier')
	address, strength = getReading()
	try:
		localize = rospy.ServiceProxy('wifiBasedLocationClassifier', Localize)
		label = localize(address, strength)
		print label
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


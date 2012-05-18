#!/usr/bin/env python
import roslib; roslib.load_manifest('roombaEarth')
import rospy

import socket
import struct
import time

def checksum256(st):
    return reduce(lambda x,y:x+y, map(ord, st)) % 256

if __name__ == "__main__":
	rospy.init_node('roombaCommander')

	sock = socket.socket( socket.AF_INET, # Internet
                      socket.SOCK_DGRAM ) # UDP
	
	while not rospy.is_shutdown():
		#data_without_checksum = struct.pack('=cBHH','#',0x6F,250,500)
		#data = struct.pack('=cBHHB','#',0x6F,250,500,checksum256(data_without_checksum))
		data = struct.pack('=cBHHB','#',0x6F,250,1000,1)
		sock.sendto( data, ('192.168.1.206', 2000) )
		time.sleep(0.2)

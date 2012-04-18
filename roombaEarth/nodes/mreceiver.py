#!/usr/bin/env python
import socket
import struct

MCAST_GRP = '239.133.1.206'
MCAST_PORT = 2000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
	binaryString = sock.recv(1024)
	if(len(binaryString)==16):
		data = struct.unpack('=BBhhIIBB',binaryString)
		# startByte  start byte
		# pktType: packet type
		# dist: distance traveled; sum of the distance traveled by two wheels divided by 2 [mm]
		# angle: difference in the distance traveled by Roomba's two wheels [mm]
		# rangeIR: range measurement [raw]
		# mts: module timestamp [msec]
		# flag: indicates if new odometry has been received
		# chk: modular sum checksum
		startByte, pktType, dist, angle, rangeIR, mts, flag, chk = data 
		print data



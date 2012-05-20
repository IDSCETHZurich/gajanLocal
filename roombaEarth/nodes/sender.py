#!/usr/bin/env python
import socket
import time

UDP_IP="127.0.0.1"
UDP_PORT=5005
MESSAGE="Hello, World!"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "base-message:", MESSAGE

sock = socket.socket( socket.AF_INET, # Internet
                      socket.SOCK_DGRAM ) # UDP
i = 0
while True:
	sock.sendto( MESSAGE+" seq #: "+str(i), (UDP_IP, UDP_PORT) )
	i = i + 1
	time.sleep(1.0)

import socket
import struct
import binascii

MCAST_GRP = '239.133.1.206'
MCAST_PORT = 2000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
	binaryString = sock.recv(1024)
	print('Recived')
	print(binaryString)  
	print(struct.unpack('2B3HIB',binaryString))
	print('-------')

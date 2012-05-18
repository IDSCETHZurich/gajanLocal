#!/usr/bin/env python
import roslib; roslib.load_manifest('roombaEarth')
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

import socket
import struct
import math

def getRangeFromRawData(data):
	#Linear model:
	#f(x) = a*(x^-1) + c
	#Coefficients (with 95% confidence bounds):
	#a =  a =  1.494e+009  (1.411e+009, 1.577e+009)
	#c =  58.47  (9.087, 107.8)
	return (1.494e+009/data + 58.47)/1000.0 #in meters

if __name__ == "__main__":
	rospy.init_node('cloudBridge')
	
	#Create the publishers
	#/odom publisher
	odomPub = rospy.Publisher('odom',Odometry)
	odoMsg = Odometry()
	odoMsg.header.frame_id = 'odom'
	odoMsg.child_frame_id = 'base_footprint'
	odoMsg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
	
	#/joint_state publisher
	jntStatePub = rospy.Publisher('joint_state',JointState)
	jntState_name = ['left_wheel_joint', 'right_wheel_joint', 'front_castor_joint', 'rear_castor_joint']
	jntState = JointState()
	jntState.name = jntState_name
	jntState.effort = [0.0, 0.0, 0.1, 0.0]

	#/scan publisher
	scanPub = rospy.Publisher('scan',LaserScan)
	scanMsg = LaserScan()
	scanMsg.header.frame_id = 'laser'
	scanMsg.angle_min = 0.0
	scanMsg.angle_max = 0.0
	scanMsg.angle_increment = 0.0
	scanMsg.time_increment = 0.0
	scanMsg.scan_time = 0.0
	scanMsg.range_min = 0.02
	scanMsg.range_max = 1.5
	scanMsg.intensities = [1.0]

	#Socket initialization
	MCAST_GRP = '239.133.1.206'
	MCAST_PORT = 2000

	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	sock.bind(('', MCAST_PORT))
	mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

	sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

	#robot joint state (joint angles in radians)
	left_wheel_angle = 0.0; 
	right_wheel_angle = 0.0;

	# Open a file
	logFile = open("log.txt", "w")
	logFile.write( "dist, angle, rangeIR, mts, flag\n");

	dist_wheel_l = 0.0
	dist_wheel_r = 0.0

	# robot states w.r.t inertial frame
	robot_x_i = 0.0
	robot_y_i = 0.0
	robot_theta_i = 0.0
	
	robot_description_r = 0.258 / 2.0

	#Get data and publish infinitly
	while not rospy.is_shutdown():

		binaryString = sock.recv(1024)
		if(len(binaryString)==16):
			data = struct.unpack('=BBHHIIBB',binaryString)
			# startByte: start byte
			# pktType: packet type
			# dist: distance traveled; sum of the distance traveled by two wheels divided by 2 [mm]
			# angle: difference in the distance traveled by Roomba's two wheels [mm]
			# rangeIR: range measurement [raw]
			# mts: module timestamp [msec]
			# flag: indicates if new odometry has been received
			# chk: modular sum checksum
			startByte, pktType, dist, angle, rangeIR, mts, flag, chk = data 
			dist_wheel_l = dist_wheel_l + float(int(dist) + int(angle))
			dist_wheel_r = dist_wheel_r + float(int(dist) - int(angle))
		
			jntState.header.stamp = rospy.get_rostime()		
			jntState.position = [0.1, 0.0, 0.0, 0.0]
			jntState.velocity = [0.0, 0.1, 0.0, 0.0]
			jntStatePub.publish(jntState)
		

			# compute odometry
			robot_theta_i = robot_theta_i + angle/robot_description_r
			robot_x_i = robot_x_i + dist * math.cos(robot_theta_i)
			robot_y_i = robot_y_i + dist * math.sin(robot_theta_i)	


			odoMsg.header.stamp = rospy.get_rostime()
			odoMsg.pose.pose.position.x = robot_x_i
			odoMsg.pose.pose.position.y = robot_y_i
			odoMsg.pose.pose.position.z = 0.0
			odoMsg.pose.pose.orientation.x = 0.0 
			odoMsg.pose.pose.orientation.y = 0.0
			odoMsg.pose.pose.orientation.z = math.sin(robot_theta_i/2.0)
			odoMsg.pose.pose.orientation.w = math.cos(robot_theta_i/2.0)
			odomPub.publish(odoMsg)
			
			scanMsg.header.stamp = rospy.get_rostime()
			scanMsg.ranges = [getRangeFromRawData(rangeIR)]
			#print(getRangeFromRawData(rangeIR))
			scanPub.publish(scanMsg)
			logFile.write('{0}, {1}, {2}, {3}, {4}\n'.format(dist, angle, rangeIR, mts, flag))
			if flag == 1:			
				print('{0}, {1}, {2}, {3}, {4} --- {5}, {6}'.format(dist, angle, rangeIR, mts, flag, dist_wheel_l, dist_wheel_r ))
				#print('{0}, {1}, {2}'.format(robot_x_i, robot_y_i, robot_theta_i))
	# Close log file

	logFile.close()

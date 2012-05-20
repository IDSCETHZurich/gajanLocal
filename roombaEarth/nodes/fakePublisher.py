#!/usr/bin/env python
import roslib; roslib.load_manifest('roombaEarth')
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

import socket
import struct
import math
import time

def getRangeFromRawData(data):
	#Linear model:
	#f(x) = a*(x^-1) + c
	#Coefficients (with 95% confidence bounds):
	#a =  a =  1.494e+009  (1.411e+009, 1.577e+009)
	#c =  58.47  (9.087, 107.8)
	return (1.494e+009/data + 58.47)/1000.0 #in meters

if __name__ == "__main__":
	rospy.init_node('fakePublisher')
	
	#Create the publishers
	#/odom publisher
	odomPub = rospy.Publisher('roombaOdom',Odometry)
	odoMsg = Odometry()
	odoMsg.header.frame_id = '/odom'
	odoMsg.child_frame_id = '/odom'
	odoMsg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
	

	#/scan publisher
	scanPub = rospy.Publisher('scan',LaserScan)
	scanMsg = LaserScan()
	scanMsg.header.frame_id = 'ir_base'
	scanMsg.angle_min = 0.01
	scanMsg.angle_max = 0.01
	scanMsg.angle_increment = 0.02
	scanMsg.time_increment = 0.0
	scanMsg.scan_time = 0.0
	scanMsg.range_min = 0.02
	scanMsg.range_max = 1.5
	scanMsg.intensities = []


	#robot joint state (joint angles in radians)
	left_wheel_angle = 0.0; 
	right_wheel_angle = 0.0;

	# Open a file
	logFile = open("log.txt", "r")


	# robot states w.r.t inertial frame
	robot_x_i = 0.0
	robot_y_i = 0.0
	robot_theta_i = 0.0
	
	robot_description_r  = 0.258 / 2.0
	msgNum = 0
	#Get data and publish infinitly
	for line in logFile:
		msgNum = msgNum + 1
		data_t = line.split(',')
		
		d_dist = int(data_t[0].strip())/1000.0
		d_angle = int(data_t[1].strip())/1000.0

		range_raw = int( data_t[2].strip() ) 

		# compute odometry
		robot_theta_i = robot_theta_i + 2*d_angle/robot_description_r;
    		robot_x_i = robot_x_i + d_dist * math.cos(robot_theta_i);
    		robot_y_i = robot_y_i + d_dist * math.sin(robot_theta_i);


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
		scanRange = getRangeFromRawData(range_raw)
		scanMsg.ranges = [scanRange,scanRange]
		scanPub.publish(scanMsg)
		print("publishing msg #{0}".format(msgNum))
		time.sleep(1.0)

	# Close log file
	logFile.close()
	print('Finished playing')

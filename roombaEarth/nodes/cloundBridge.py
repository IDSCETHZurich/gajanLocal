#!/usr/bin/env python
import roslib; roslib.load_manifest('roombaEarth')
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

if __name__ == "__main__":
	rospy.init_node('cloudBridge')
	covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]

	'''Create the publishers'''
	'''/odom publisher'''
	odomPub = rospy.Publisher('odom',Odometry)
	
	
	'''/joint_state publisher'''
	jntStatePub = rospy.Publisher('joint_state',JointState)
	jntState_name = ['left_wheel_joint', 'right_wheel_joint', 'front_castor_joint', 'rear_castor_joint']


	#Socket initialization

	#Get data and publish infinitly
	while not rospy.is_shutdown():
		jntState = JointState()
		jntState.header.stamp = rospy.get_rostime()
		jntState.name = jntState_name
		jntState.position = [0.1, 0.0, 0.0, 0.0]
		jntState.velocity = [0.0, 0.1, 0.0, 0.0]
		jntState.effort = [0.0, 0.0, 0.1, 0.0]
		jntStatePub.publish(jntState)

		odoMsg = Odometry()
		odoMsg.header.stamp = rospy.get_rostime()
		odoMsg.header.frame_id = 'odom'
		odoMsg.child_frame_id = 'base_footprint'
		odoMsg.pose.pose.position.x = 0.0
		odoMsg.pose.pose.position.y = 0.0
		odoMsg.pose.pose.orientation.x = 0.0 
		odoMsg.pose.pose.orientation.y = 0.0
		odoMsg.pose.pose.orientation.z = 0.0
		odoMsg.pose.pose.orientation.w = 0.1
		odoMsg.pose.covariance = covariance
		odomPub.publish(odoMsg)
        	rospy.sleep(1.0)
				


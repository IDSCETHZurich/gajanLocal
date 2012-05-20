#!/usr/bin/env python  
import roslib
roslib.load_manifest('roombaEarth')
import rospy

from nav_msgs.msg import Odometry
import tf

def handle_roomba_odometry(odoMsg):
    br = tf.TransformBroadcaster()
    br.sendTransform((odoMsg.pose.pose.position.x, odoMsg.pose.pose.position.y, 0.0),
                     (0.0, 0.0, odoMsg.pose.pose.orientation.z, odoMsg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    br.sendTransform((0.0, 0.0, 0.05),
                     (0.0, 0.0, 1.0, 0.0),
                     rospy.Time.now(),
                     "ir_base",
                     "base_link")
    #print('handle_roomba_odometry called')

if __name__ == '__main__':
    rospy.init_node('roomba_tf_broadcaster')
    rospy.Subscriber('roombaOdom',
                     Odometry,
                     handle_roomba_odometry)
    rospy.spin()

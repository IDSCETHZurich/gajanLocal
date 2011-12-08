#!/usr/bin/env python
import roslib; roslib.load_manifest('re_BNLabelMe')
import rospy
from std_msgs.msg import String
from re_BNLabelMe.msg  import unorderedListwithEvidence
from re_BNLabelMe.msg  import orderedList
def callback(data):
    rospy.loginfo(rospy.get_name()+"uList: %s, evidence: %s",data.uList,data.evidence)

def listener():
    rospy.init_node('reorderingNode', anonymous=True)
    rospy.Subscriber("unorderedListwithEvidence", unorderedListwithEvidence, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

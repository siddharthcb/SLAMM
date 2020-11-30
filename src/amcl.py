#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def listener(): 
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)

if __name__ == '__main__':
    listener()

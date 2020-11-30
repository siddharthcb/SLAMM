#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

pose = PoseStamped()

def listener():
    rospy.init_node('trajectory', anonymous=True)
    rospy.Subscriber('trajectory', Path, callback)
    rospy.spin()
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)
        
if __name__ == '__main__':
    listener()


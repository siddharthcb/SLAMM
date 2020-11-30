#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv

fieldnames=["x","y","z","qx","qy","qz","qw","is_searching_area","reach_threshold"]
pub = rospy.Publisher('waypoints', Odometry, queue_size=10)
wpt=Odometry()
wpt.header.frame_id = "map"
wpt.child_frame_id = "map"
def callback(data):
    rospy.loginfo("Stored a point")
    X=data.pose.position.x
    Y=data.pose.position.y
    Z=data.pose.position.z
    QX=data.pose.orientation.x
    QY=data.pose.orientation.y
    QZ=data.pose.orientation.z
    QW=data.pose.orientation.w
    isa=0
    rt=3
    wpt.pose.pose.position.x=X
    wpt.pose.pose.position.y=Y
    wpt.pose.pose.position.z=Z
    wpt.pose.pose.orientation.x=QX
    wpt.pose.pose.orientation.y=QY
    wpt.pose.pose.orientation.z=QZ
    wpt.pose.pose.orientation.w=QW
    pub.publish(wpt)
    rospy.loginfo(wpt)
    
    with open('waypoint.csv', 'a') as csv_file:
            csv_writer=csv.DictWriter(csv_file,fieldnames)
            info ={"x":X,"y":Y,"z":Z,"qx":QX,"qy":QY,"qz":QZ,"qw":QW,"is_searching_area":isa,"reach_threshold":rt}
            csv_writer.writerow(info)
    
            
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('waypoint_rviz', anonymous=True)

    rospy.Subscriber("move_base_simple/goal", PoseStamped, callback)
   

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
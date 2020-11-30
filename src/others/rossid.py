import rospy, time, sys
from geometry_msgs.msg import Twist
 
def velocity_received_callback(message):
    wheel_base=0.091
    max_speed=0.5
    """Handle new velocity command message."""

    last_received = rospy.get_time()

    # Extract linear and angular velocities from the message
    linear = message.linear.x
    angular = message.angular.z

    # Calculate wheel speeds in m/s
    left_speed = linear - angular*wheel_base/2
    right_speed = linear + angular*wheel_base/2
    left_speed_percent = (100 * left_speed/max_speed)
    right_speed_percent = (100 * right_speed/max_speed)
    print("left_speed",left_speed)
 
if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('turtlesim_listener')
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber('turtle1/cmd_vel', Twist,velocity_received_callback)
    rospy.spin()

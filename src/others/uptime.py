#!/usr/bin/env python
import rospy
import socket
import struct
# from socket import *
# from thread import *
from geometry_msgs.msg import Twist

MOAB_PORT = 12346
SBUS_PORT = 31338
R_CART = 0.1534

left_speed = 0.0
right_speed = 0.0

def SendFloat(float1, float2,sbus_sock):
    udpPacket = struct.pack('ff', float1, float2)
    sbus_sock.sendto(udpPacket, ("192.168.8.20", MOAB_PORT))
    # print(float1,float2)

class Driver:

    def __init__(self):
        rospy.init_node('driver')
        rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)
        rospy.spin()

    def velocity_received_callback(self, message):
        global left_speed
        global right_speed
        self._last_received = rospy.get_time()
        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = (linear - angular) * R_CART * 25
        right_speed = (linear + angular) * R_CART * 25
        print("left_speed",left_speed)
        ## This is the sbus udp port.
        sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sbus_sock.bind(("0.0.0.0", SBUS_PORT))
        rpmL, rpmR = float(left_speed),float(right_speed)
        print([rpmL,rpmR])
        SendFloat(rpmL, rpmR, sbus_sock)
        # SendFloat(7,7, sbus_sock)
        # print("right_speed",self.right_speed)

if __name__ == '__main__':
    driver = Driver()

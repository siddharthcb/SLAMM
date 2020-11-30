#!/usr/bin/env python
import rospy
import socket
import struct
from geometry_msgs.msg import Twist
import numpy as np

MOAB_PORT = 12346
SBUS_PORT = 31338
R_CART = 0.153

left_speed = 0.0
right_speed = 0.0
R_wheel = 0.15

def SendFloat(float1, float2,sbus_sock):
    udpPacket = struct.pack('ff', float1, float2)
    sbus_sock.sendto(udpPacket, ("192.168.8.20", MOAB_PORT))
    # print(float1,float2)

class Driver:
    def __init__(self):
        rospy.init_node('uptime')
        rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)
        #hj=rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)
        # print(hj.linear,hj.angular)
        rospy.spin()

    def velocity_received_callback(self, message):
        global left_speed
        global right_speed
        self._last_received = rospy.get_time()
        # Extract linear and angular velocities from the message
        linear = message.linear.x
        # linear=linear*(-1)
        angular = message.angular.z * 0.32

        # Calculate wheel speeds in m/s
        left_speed = (linear - angular) * R_CART * 400
        right_speed = (linear + angular) * R_CART * 400
        #left_speed = (linear - angular*R_CART/2.0)/(2*np.pi*R_wheel) * 50
        #right_speed = (linear + angular*R_CART/2.0)/(2*np.pi*R_wheel)  * 50
        
        #print("left_speed {}  right_speed {}".format(left_speed, right_speed))
        # print("right_speed",right_speed)
        ## This is the sbus udp port.
        sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sbus_sock.bind(("0.0.0.0", SBUS_PORT))
        rpmL, rpmR = float(left_speed),float(right_speed)

        if (rpmL < 0.0 and rpmR > 0.0):
            rpmL = -5.0
            rpmR = 5.0
        elif (rpmL > 0.0 and rpmR < 0.0):
            rpmL = 5.0
            rpmR = -5.0

        print("rpml, rpmr:",rpmL,rpmR)
        SendFloat(rpmR, rpmL, sbus_sock)
        # SendFloat(7,7, sbus_sock)
        # print("right_speed",self.right_speed)

if __name__ == '__main__':
    driver = Driver()

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
L_cart = 0.6
Linear2RPM = 180.0/(R_wheel*6*np.pi)


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
        angular = message.angular.z * 0.5

        # Calculate wheel speeds in m/s
        # left_speed = (linear - angular) * R_CART * 400
        # right_speed = (linear + angular) * R_CART * 400
        #left_speed = (linear - angular*R_CART/2.0)/(2*np.pi*R_wheel) * 50
        #right_speed = (linear + angular*R_CART/2.0)/(2*np.pi*R_wheel)  * 50

        Vx = linear
        Wz = angular

        # rot_scale_up = 15.0
        scale_up = 1.0  #1.2

        ## Go straight, no turn
        if Vx == 0.0 and Wz == 0.0:
            rpmL = 0.0
            rpmR = 0.0
            print("No velocity__Vx {:.4f}  Wz {:.4f}  rpmL {:.4f}  rpmR {:.4f}".format(Vx,Wz,rpmL,rpmR))
        elif Wz == 0.0:
            rpmL = (Vx/2)*Linear2RPM
            rpmR = rpmL
            ## Scale up the rpm
            rpmL = rpmL*scale_up
            rpmR = rpmR*scale_up
            print("GO STRAIGHT___Vx {:.4f}  Wz {:.4f}  rpmL {:.4f}  rpmR {:.4f}".format(Vx,Wz,rpmL,rpmR))
        elif Vx == 0.0 and Wz != 0.0:
            rpmL = -Wz*(60/(2*np.pi))*(L_cart/2.0)
            rpmR = Wz*(60/(2*np.pi))*(L_cart/2.0)
            ## Scale up the rpm
            rpmL = rpmL*scale_up
            rpmR = rpmR*scale_up
            print("Skidding___Vx {:.4f}  Wz {:.4f}  rpmL {:.4f}  rpmR {:.4f}".format(Vx,Wz,rpmL,rpmR))
        elif Vx != 0.0 and Wz != 0.0:
            ICC_R = abs(Vx)/abs(Wz)
            if Wz > 0.0:
                VL = Wz*(ICC_R - L_cart/2)
                VR = Wz*(ICC_R + L_cart/2)
                rpmL = VL*Linear2RPM
                rpmR = VR*Linear2RPM
                ## Scale up the rpm
                rpmL = rpmL*scale_up
                rpmR = rpmR*scale_up
                print("Curve Left___Vx {:.4f}  Wz {:.4f}  rpmL {:.4f}  rpmR {:.4f}".format(Vx,Wz,rpmL,rpmR))
            else:
                VL = abs(Wz)*(ICC_R + L_cart/2)
                VR = abs(Wz)*(ICC_R - L_cart/2)
                rpmL = VL*Linear2RPM
                rpmR = VR*Linear2RPM
                ## Scale up the rpm
                rpmL = rpmL*scale_up
                rpmR = rpmR*scale_up
                print("Curve Right__Vx {:.4f}  Wz {:.4f}  rpmL {:.4f}  rpmR {:.4f}".format(Vx,Wz,rpmL,rpmR))
        else:
            rpmL = 0.0
            rpmR = 0.0
            print("None of conditions....")

        #print("left_speed {}  right_speed {}".format(left_speed, right_speed))
        # print("right_speed",right_speed)
        ## This is the sbus udp port.
        sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sbus_sock.bind(("0.0.0.0", SBUS_PORT))
        # rpmL, rpmR = float(left_speed),float(right_speed)

        if (rpmL < 0.0 and rpmR > 0.0):
            rpmL = -10.0
            rpmR = 10.0
        elif (rpmL > 0.0 and rpmR < 0.0):
            rpmL = 10.0
            rpmR = -10.0

        # print("rpml, rpmr:",rpmL,rpmR)
        SendFloat(rpmR, rpmL, sbus_sock)
        # SendFloat(7,7, sbus_sock)
        # print("right_speed",self.right_speed)

if __name__ == '__main__':
    driver = Driver()

#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ros_risk_udp.py

import rospy
import socket
import struct
from time import sleep
import numpy as np
from kriso_msgs.msg import CAtoVcc as CAtoVcc

serverAddressPort   = ("127.0.0.1", 20211)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

def callback(data):
    print("callback in!")
    buf = struct.pack('%sf' % len(data.ca_risk_arr), *data.ca_risk_arr)
    print("buffer size : ", len(buf))
    UDPClientSocket.sendto(buf, serverAddressPort)
    print("Risk Info was sent!")     
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('risk_udp', anonymous=True)

    rospy.Subscriber("/kriso/ca_to_vcc", CAtoVcc, callback)
    print('callback register!')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=MMUCaToVccPublish.py

import rospy
from mavros.utils import *
import numpy as np
from kriso_msgs.msg import CAtoVcc as CAtoVcc

from kriso_msgs.msg import Waypoint as Waypoint
import os

def talker():
    dir_path = os.path.dirname(os.path.abspath(__file__))
    
    pub = rospy.Publisher('/kriso/ca_to_vcc', CAtoVcc, queue_size=10)
    rospy.init_node('test_kriso_control_ca_to_vcc', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = CAtoVcc()

    msg.ca_risk_arr = []
    with open(dir_path+"/risk.csv", "r") as f:
        for line in f:
            msg.ca_risk_arr.append(float(line))

    msg.ca_mode = 1
    msg.ca_method = 2
    msg.ca_status = 3
    # msg.ca_risk_arr = [0.123]*900 #np.random.rand(900)
    msg.reactive_spd_d = 1.1
    msg.reactive_psi_d = 2.1
    msg.local_path = [Waypoint()] * 100

    while not rospy.is_shutdown():
        rospy.loginfo(msg.ca_method)
        pub.publish(msg)
        # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

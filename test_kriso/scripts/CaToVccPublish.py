#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=CaToVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import CAToVcc as CAToVcc

def talker():
    pub = rospy.Publisher('/kriso/ca_to_vcc', CAToVcc, queue_size=10)
    rospy.init_node('test_kriso_control_ca_to_vcc', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = CAToVcc()

    msg.ca_mode = 1
    msg.ca_method = 2
    msg.ca_status = 3
    msg.ca_risk_arr = 
    msg.reactive_spd_d = 1.1
    msg.reactive_psi_d = 2.1

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

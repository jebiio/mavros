#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=CKVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import CKtoVcc as CKtoVcc

def talker():
    pub = rospy.Publisher('/kriso/ck_to_vcc', CKtoVcc, queue_size=10)
    rospy.init_node('test_kriso_ck_to_vcc_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = CKtoVcc()

    msg.psi_d = 1.1
    msg.spd_d = 1.2

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        # msg.roll += 0.1;
        msg.psi_d += 0.1
        msg.spd_d += 0.1

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

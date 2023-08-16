#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ControlcmdToVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import ControlcmdtoVcc as ControlcmdtoVcc

def talker():
    pub = rospy.Publisher('/kriso/control_cmd_to_vcc', ControlcmdtoVcc, queue_size=10)
    rospy.init_node('test_kriso_control_cmd_to_vcc', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = ControlcmdtoVcc()

    msg.t1_rpm = 1.1
    msg.t2_rpm = 1.2
    msg.t3_rpm = 1.3
    msg.t3_angle = 1.4
    msg.t4_rpm = 1.5
    msg.t4_angle = 1.6
    msg.oper_mode = 1
    msg.mission_mode = 2 

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

#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=DPVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import DPtoVcc as DPtoVcc

def talker():
    pub = rospy.Publisher('/kriso/dp_to_vcc', DPtoVcc, queue_size=10)
    rospy.init_node('test_kriso_dp_to_vcc_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = DPtoVcc()

    msg.surge_error = 1.1
    msg.sway_error = 1.2
    msg.yaw_error = 1.1
    msg.dp_start_stop = 0

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

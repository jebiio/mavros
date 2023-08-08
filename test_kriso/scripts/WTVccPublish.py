#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=WTVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import WTtoVcc as WTtoVcc

def talker():
    pub = rospy.Publisher('/kriso/wt_to_vcc', WTtoVcc, queue_size=10)
    rospy.init_node('test_kriso_wt_to_vcc_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = WTtoVcc()

    msg.psi_d = 1.1
    msg.spd_d = 1.2
    msg.wp_lat_d = 1.3
    msg.wp_lon_d = 1.4
    msg.track_path_idx = 3
    
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

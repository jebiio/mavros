#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=DPVccPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import MiddlewareToVcc as MiddlewareToVcc

def talker():
    pub = rospy.Publisher('/kriso/middleware_to_vcc', MiddlewareToVcc, queue_size=10)
    rospy.init_node('test_kriso_middleware_to_vcc', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = MiddlewareToVcc()

    msg.nav_mode     = 1
    msg.nav_roll     = 0.1
    msg.nav_pitch    = 0.2
    msg.nav_yaw      = 3.3
    msg.nav_yaw_rate = 0.4
    msg.nav_cog      = 0.5
    msg.nav_sog      = 0.6
    msg.nav_uspd     = 0.7
    msg.nav_vspd     = 0.8
    msg.nav_wspd     = 0.9
    msg.nav_longitude= 23.23
    msg.nav_latitude = 127.2343
    msg.nav_heave    = 1.0
    msg.nav_gpstime  = 1.1
    msg.wea_airtem   = 2.1
    msg.wea_wattem   = 3.1
    msg.wea_press    = 4.1
    msg.wea_relhum   = 5.1
    msg.wea_dewpt    = 6.1
    msg.wea_windirt  = 7.1
    msg.wea_winspdt  = 8.1
    msg.wea_windirr  = 9.1
    msg.wea_watspdr  = 10.1
    msg.wea_watdir   = 11.1
    msg.wea_watspd   = 12.1
    msg.wea_visibiran= 13.1

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

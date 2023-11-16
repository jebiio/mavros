#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParserToSerial.py

import rospy
from kriso_msgs.msg import MiddlewareToVcc as MiddlewareToVcc
from kriso_msgs.msg import ToCooperation as ToCooperation

# middleware_msg = MiddlewareToVcc()
serial_msg = ToCooperation()

def callback_subscribe(msg):
    pass


def talker():
    sub = rospy.Subscriber('/kriso/middleware_to_vcc', MiddlewareToVcc, callback_subscribe)
    pub = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10)
    rospy.init_node('acm_to_serial', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    # msg = MiddlewareToVcc()

    # msg.nav_mode     = 1
    # msg.nav_roll     = 0.1
    # msg.nav_pitch    = 0.2
    # msg.nav_yaw      = 10.0
    # msg.nav_yaw_rate = 0.4
    # msg.nav_cog      = 0.5
    # msg.nav_sog      = 0.6
    # msg.nav_uspd     = 0.7
    # msg.nav_vspd     = 0.8
    # msg.nav_wspd     = 0.9
    # msg.nav_vspd     = 0.8
    # msg.nav_wspd     = 0.9
    # msg.nav_longitude= 126.6250512
    # msg.nav_latitude = 37.175871
    # msg.nav_heave    = 1.0
    # msg.nav_gpstime  = 1.1
    # msg.wea_airtem   = 2.1
    # msg.wea_wattem   = 3.1
    # msg.wea_press    = 4.1
    # msg.wea_relhum   = 5.1
    # msg.wea_dewpt    = 6.1
    # msg.wea_windirt  = 7.1
    # msg.wea_winspdt  = 8.1
    # msg.wea_windirr  = 9.1
    # msg.wea_watspdr  = 10.1
    # msg.wea_watdir   = 11.1
    # msg.wea_watspd   = 12.1
    # msg.wea_visibiran= 13.1

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        serial_msg = ACMParser(middleware_msg) 
        pub.publish(serial_msg)
        # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

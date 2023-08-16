#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=MiddlewareAISPublish.py

import rospy
from mavros.utils import *
from kriso_msgs.msg import MiddlewareAIS as MiddlewareAIS

def talker():
    pub = rospy.Publisher('/kriso/middleware_ais', MiddlewareAIS, queue_size=10)
    rospy.init_node('test_kriso_middleware_ais', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = MiddlewareAIS()
    msg.msg_type  = 1
    msg.repeat    = 2
    msg.mmsi      = 3
    msg.reserved_1= 4
    msg.speed     = 2.2
    msg.accuracy  = 1
    msg.lon       = 126.6250520
    msg.lat       = 37.175874
    msg.course    = 2.3
    msg.heading   = 3.3
    msg.second    = 22
    msg.reserved_2= 2
    msg.cs        = 1
    msg.display   = 1
    msg.dsc       = 1
    msg.band      = 1
    msg.msg22     = 1
    msg.assigned  = 1
    msg.raim      = 1
    msg.radio     = 2333

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

#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParserToSerial.py

import rospy
from kriso_msgs.msg import MiddlewareToVcc as MiddlewareToVcc
from kriso_msgs.msg import ToCooperation as ToCooperation

# middleware_msg = MiddlewareToVcc()
# coop_msg = ToCooperation()

def callback_middleware(msg):
    # make ToCooperation msg from MiddlewareToVcc msg
    # fields of MiddlewareToVcc msg
    # create ToCooperation msg
    coop_msg = ACMParser.getMsg(msg)
    # publish ToCooperation msg
    pub.publish(coop_msg)
    pass


def talker():
    sub = rospy.Subscriber('/kriso/middleware_to_vcc', MiddlewareToVcc, callback_middleware)
    pub = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10)
    rospy.init_node('acm_to_serial', anonymous=True)
    rate = rospy.Rate(10) # 1hz

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

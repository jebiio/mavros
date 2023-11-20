#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParserToSerial.py

import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
 
from kriso_msgs.msg import ToCooperation as ToCooperation
from kriso_msgs.msg import MiddlewareToVcc as MiddlewareToVcc
from kriso_msgs.msg import ToCooperation as ToCooperation

# middleware_msg = MiddlewareToVcc()
# coop_msg = ToCooperation()

pub = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10) 

def callback_middleware(msg):
    # make ToCooperation msg from MiddlewareToVcc msg
    # fields of MiddlewareToVcc msg
    # create ToCooperation msg
    # coop_msg = ACMParser.getMsg(msg)
    # publish ToCooperation msg
    rospy.loginfo('MiddlewareToVcc received!')
    tx_msg = Tx_Message()
    tx_msg.roll = msg.nav_roll
    tx_msg.pitch = msg.nav_pitch
    tx_msg.yaw = msg.nav_yaw
    tx_msg.uv_position_lat = msg.nav_latitude
    tx_msg.uv_position_lon = msg.nav_longitude
    # tx_msg.uv_ground_speed = ??
    # tx_msg.build_packet(echo_back)
    
    coop_msg = ToCooperation()
    coop_msg.length = 63
    coop_msg.packet = tx_msg.build_packet(bytearray(34))
    pub.publish(coop_msg)
    pass


def talker():
    sub = rospy.Subscriber('/kriso/middleware_to_vcc', MiddlewareToVcc, callback_middleware)
    # pub = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10)    
    rospy.init_node('acm_to_serial', anonymous=True)
    rate = rospy.Rate(10) # 1hz

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        # to_coop_msg = ToCooperation()# serial_msg = ACMParser(middleware_msg) 
        # pub.publish(to_coop_msg)
        # # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

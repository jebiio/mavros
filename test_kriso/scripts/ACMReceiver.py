#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMReceiver.py
# 미들웨어 msg전송 : > roslaunch test_kriso kriso_test.launch msg:=MiddlewareStatusPublish.py
# ACMParser : > roslaunch test_kriso kriso_test.launch msg:=ACMParser.py

import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
from kriso_msgs.msg import ToCooperation as ToCooperation


def callback_receive(msg):
    if msg.length == 63:
        # print msg.packet
        rospy.loginfo("middleware info received")
        check_packet(msg.packet)
    else:
        rospy.loginfo("Received packet length is not 63")

def check_packet(packet):
    if packet[0] == 0xAA and packet[1] == 0x55:
        print("Header is valid")
    
    if struct.unpack(">H", packet[34:36])[0] != 0: # roll
        print("roll is not valid")
        return False
    if struct.unpack(">H", packet[36:38])[0] != 32767:#pitch
        print("pitch is not valid")
        return False
    if struct.unpack(">H", packet[38:40])[0] != 0xffff: # yaw
        print("yaw is not valid")
        return False
     
    if struct.unpack(">l", packet[40:44])[0] != 1231234567: # latitude
        print("latitude is not valid")
        return False
    if struct.unpack(">l", packet[44:48])[0] != 321234567: # longitude
        print("longitude is not valid")
        return False

    if packet[50] != 51: # ground_speed
        print("ground_speed is not valid")
        return False
    if struct.unpack(">H", packet[51:53])[0] != 0xffff: # course_heading
        print("course_heading is not valid")
        return False    


def talker():
    sub = rospy.Subscriber('/kriso/to_cooperation', ToCooperation, callback_receive)

    rospy.init_node('acm_toserial_receiver', anonymous=True)
    rate = rospy.Rate(2) # 1hz
    count = 1
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

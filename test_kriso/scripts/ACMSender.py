#!/usr/bin/env python3
import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
from kriso_msgs.msg import FromCooperation as FromCooperation
# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMSender.py


def talker():
    pub = rospy.Publisher('/kriso/from_cooperation', FromCooperation, queue_size=10)
    rospy.init_node('acm_to_serial', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    count = 1
    while not rospy.is_shutdown():
        coop_msg = FromCooperation()
        coop_msg.length = 34
        coop_msg.packet = bytearray(34)

        if count % 3 == 0:
            coop_msg.packet[10] = 1
        elif count % 3 == 1:
            coop_msg.packet[10] = 2
        elif count % 3 == 2:
            coop_msg.packet[10] = 255
        # rospy.loginfo(msg)
        # serial_msg = ACMParser(middleware_msg) 
        # pub.publish(serial_msg)
        # msg.roll += 0.1;

        pub.publish(coop_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

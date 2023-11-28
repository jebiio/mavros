#!/usr/bin/env python3
import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
from kriso_msgs.msg import FromCooperation as FromCooperation
import struct 
# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMSender.py

PACKET_HEADER1 = 0
PACKET_HEADER2 = 1
PAYLOAD_LENGTH = 2
PACKET_SEQUENCE = 3
SOURCE_ID = 4
SOURCE_PORT = 5
DESTINATION_ID = 6
DESTINATION_PORT = 7
PACKET_PRIORITY = 8
MESSAGE_ID = 9
OPERATION_MODE = 10

def talker():
    pub = rospy.Publisher('/kriso/from_cooperation', FromCooperation, queue_size=10)
    rospy.init_node('acm_fromserial_publish', anonymous=True)
    rate = rospy.Rate(2) # 1hz
    count = 1
    while not rospy.is_shutdown():
        coop_msg = FromCooperation()
        coop_msg.length = 34
        coop_msg.packet = bytearray(34)

        if count % 3 == 0:
            coop_msg.packet = make_stick()
        elif count % 3 == 1:
            coop_msg.packet = make_waypoint()
        elif count % 3 == 2:
            coop_msg.packet = make_emergency()
        # rospy.loginfo(msg)
        # serial_msg = ACMParser(middleware_msg) 
        # pub.publish(serial_msg)
        # msg.roll += 0.1;
        count = count + 1
        pub.publish(coop_msg)
        rate.sleep()


def make_stick():
    packet = bytearray(34)
    packet[PACKET_HEADER1] = 0xAA
    packet[PACKET_HEADER2] = 0x55
    packet[PAYLOAD_LENGTH] = 22
    packet[PACKET_SEQUENCE] = 0
    packet[SOURCE_ID] = 0x02
    packet[SOURCE_PORT] = 0
    packet[DESTINATION_ID] = 0
    packet[DESTINATION_PORT] = 0
    packet[PACKET_PRIORITY] = 0
    packet[MESSAGE_ID] = 1
    packet[OPERATION_MODE] = 1 # 1, 2, 255

    packet[11:13] = struct.pack(">H", np.uint16(1)) # roll
    packet[13:15] = struct.pack(">H", np.uint16(2)) # pitch
    packet[15:17] = struct.pack(">H", np.uint16(3)) # throttle
    packet[17] = 4 # rudder

    packet[32:34] = struct.pack(">H", np.uint16(sum(self.packet[2:32])))
    return packet

def make_waypoint():
    packet = bytearray(34)
    packet[PACKET_HEADER1] = 0xAA
    packet[PACKET_HEADER2] = 0x55
    packet[PAYLOAD_LENGTH] = 22
    packet[PACKET_SEQUENCE] = 0
    packet[SOURCE_ID] = 0x02
    packet[SOURCE_PORT] = 0
    packet[DESTINATION_ID] = 0
    packet[DESTINATION_PORT] = 0
    packet[PACKET_PRIORITY] = 0
    packet[MESSAGE_ID] = 1
    packet[OPERATION_MODE] = 2

    packet[11:13] = struct.pack(">H", np.uint16(1)) # altitude
    packet[13:15] = struct.pack(">H", np.uint16(2)) # speed
    packet[15:17] = struct.pack(">H", np.uint16(3)) # heading_angle
    packet[21:25] = struct.pack(">l", np.int32(1281234567)) # target_waypoint_position_lat
    packet[25:29] = struct.pack(">l", np.int32(321234567)) # target_waypoint_position_lon
    
    packet[32:34] = struct.pack(">H", np.uint16(sum(self.packet[2:32])))
    return packet

def make_emergency():
    packet = bytearray(34)
    packet[PACKET_HEADER1] = 0xAA
    packet[PACKET_HEADER2] = 0x55
    packet[PAYLOAD_LENGTH] = 22
    packet[PACKET_SEQUENCE] = 0
    packet[SOURCE_ID] = 0x02
    packet[SOURCE_PORT] = 0
    packet[DESTINATION_ID] = 0
    packet[DESTINATION_PORT] = 0
    packet[PACKET_PRIORITY] = 0
    packet[MESSAGE_ID] = 1
    packet[OPERATION_MODE] = 255

    packet[11] = 1 # 0: hovering, 1: return home

    packet[32:34] = struct.pack(">H", np.uint16(sum(self.packet[2:32])))
    return packet

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

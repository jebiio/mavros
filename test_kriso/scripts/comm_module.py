#!/usr/bin/env python3

import serial
import time
import struct
import threading
import socket
import math
import numpy as np

'''
packet : binary format data for communication
message : data structure for handling data
'''

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

# RX_ERROR_CODE
RX_HEADER_ERROR = 0
RX_CHECKSUM_ERROR = 8
'''
rule for packet
0 : 0xAA
1 : 0x55
2 : 1, 2, 5
'''

# KRISO 스펙에 따라서 0, 1, 2, 9번째 packet을 체크
rx_header_check_rule = {0:[0xAA], 1:[0x55], 2:[22], 8:[0], 9:[1], 10:[1, 2, 255]}

class UsvStickMode(object):
    def __init__(self) -> None:
        self.stick_roll_cmd = 0.0
        self.stick_pitch_cmd = 0.0
        self.stick_throttle_cmd = 0.0
        self.raw_stick_roll_cmd = 0
        self.raw_stick_pitch_cmd = 0
        self.raw_stick_throttle_cmd = 0
        self.raw_stick_rudder_cmd = 0
    
    def parse_raw(self, packet) -> None:
        self.raw_stick_roll_cmd = struct.unpack(">H", packet[11:13])[0]
        self.raw_stick_pitch_cmd = struct.unpack(">H", packet[13:15])[0]
        self.raw_stick_throttle_cmd = struct.unpack(">H", packet[15:17])[0]
        self.raw_stick_rudder_cmd = packet[17] #struct.unpack(">B", packet[17])[0]
    def parse(self, packet) -> None:
        self.stick_roll_cmd = int.from_bytes(packet[11:13], 'big') * 0.00274 - 90.0
        self.stick_pitch_cmd = int.from_bytes(packet[13:15], 'big')# self.stick_pitch_cmd = int.from_bytes(packet[13:15], 'big') * 0.00015
        self.stick_throttle_cmd = int.from_bytes(packet[15:17], 'big') * 0.00015
        self.stick_rudder_cmd = packet[17]

class WaypointMode(object):
    def __init__(self) -> None:
        self.final_altitude_cmd = 0.0
        self.maximum_speed_cmd = 0.0
        self.heading_angle_cmd = 0.0
        self.target_waypoint_position_lat = 0.0
        self.target_waypoint_position_lon = 0.0

        self.raw_final_altitude_cmd = 0
        self.raw_maximum_speed_cmd = 0
        self.raw_heading_angle_cmd = 0
        self.raw_target_waypoint_position_lat = 0
        self.raw_target_waypoint_position_lon = 0

    def parse_raw(self, packet) -> None:
        self.raw_final_altitude_cmd = struct.unpack(">H", packet[11:13])[0]
        self.raw_maximum_speed_cmd = struct.unpack(">H", packet[13:15])[0]
        self.raw_heading_angle_cmd = struct.unpack(">H", packet[15:17])[0]
        self.raw_target_waypoint_position_lat = struct.unpack(">l", packet[21:25])[0] 
        self.raw_target_waypoint_position_lon = struct.unpack(">l", packet[25:29])[0] 
        
    def parse(self, packet) -> None:
        self.final_altitude_cmd = int.from_bytes(packet[11:13], 'big') * 0.0152
        self.maximum_speed_cmd = int.from_bytes(packet[13:15], 'big') * 0.054
        self.heading_angle_cmd = int.from_bytes(packet[15:17], 'big') * 0.00548
        self.target_waypoint_position_lat = struct.unpack(">l", packet[21:25])[0] # self.target_waypoint_position_lat = packet[21:25]
        self.target_waypoint_position_lon = struct.unpack(">l", packet[25:29])[0] # self.target_waypoint_position_lon = packet[25:29]

    def build_packet(self):
        packet = bytearray(10)
        packet[0:2]= int.to_bytes(self.final_altitude_cmd, 2, 'big')
        packet[2:4] = int.to_bytes(self.maximum_speed_cmd, 2, 'big')
        packet[4:6] = int.to_bytes(self.heading_angle_cmd, 2, 'big')
        return packet
    
class EmergencyMode(object):
    def __init__(self) -> None:
        self.current_operation_mode = 0
        self.raw_current_operation_mode = 0

    def parse_raw(self, packet) -> None:
        self.raw_current_operation_mode = packet[11] #struct.unpack(">B", packet[11])[0]
    def parse(self, packet) -> None:
        self.current_operation_mode = packet[11] # 0: hovering, 1: return home
    def build_packet(self):
        packet = bytearray(10)
        packet[0] = self.current_operation_mode
        return packet

class EndOperationModeData(object):
    def __init__(self) -> None:
        self.raw_target_waypoint_position_latitude = 0
        self.raw_target_waypoint_position_longitude = 0
        self.target_waypoint_position_latitude = 0
        self.target_waypoint_position_longitude = 0
        pass
    def parse_raw(self, packet) -> None:
        self.raw_target_waypoint_position_lati = struct.unpack(">l", packet[21:25])[0] 
        self.raw_target_waypoint_position_longitude = struct.unpack(">l", packet[25:29])[0]
    def parse(self, packet) -> None:
        self.target_waypoint_position_latitude = int.from_bytes(packet[21:25], 'big', signed=True)
        self.target_waypoint_position_longitude = int.from_bytes(packet[25:29], 'big', signed=True)


class Rx_Message(object):
    def __init__(self):
        self.sequence = 0
        self.source_id = 0
        self.source_port = 0
        self.destination_id = 0
        self.destination_port = 0
        self.packet_priority = 0
        self.message_id = 0
        self.operation_mode = 0

        self.error_code = -1
    def copy_packet(self, packet):
        self.packet[0:34] = packet[0:34]
    def check_header_validation(self, packet):
        # 2bytes arrays compare same value
        self.sequence = packet[PACKET_SEQUENCE]
        self.source_id = packet[SOURCE_ID]
        self.source_port = packet[SOURCE_PORT]
        self.destination_id = packet[DESTINATION_ID]
        self.destination_port = packet[DESTINATION_PORT]
        self.packet_priority = packet[PACKET_PRIORITY]        
        self.message_id = packet[MESSAGE_ID]
        
        self.operation_mode = packet[OPERATION_MODE]
        if self.operation_mode == 1: # stick mode
            self.stickmode = UsvStickMode()
            self.stickmode.parse(packet)
        elif self.operation_mode == 2: # waypoint mode
            self.waypointmode = WaypointMode()
            self.waypointmode.parse(packet)
        elif self.operation_mode == 255: # emergency mode
            self.emergencymode = EmergencyMode()
            self.emergencymode.parse(packet)
        
        self.endoperationmode = EndOperationModeData()
        self.endoperationmode.parse(packet)

    
class Tx_Message(object):
    def __init__(self) -> None:
        self.packet = bytearray(63)
        self.marker_detected = 0
        self.num_sat = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.uv_position_lat = 0.0
        self.uv_position_lon = 0.0
        self.uv_altitude = 0.0
        self.uv_ground_speed = 0
        self.course_heading = 0.0
        self.simulator = 0
        self.system_status = 0
        self.emergency_status = 0
        self.system_voltage1 = 0.0
        self.system_current1 = 0.0
        self.system_voltage2 = 0.0
        self.system_current2 = 0.0
        # self.checksum = 0

    def add_header(self):
        self.packet[0] = 0xAA
        self.packet[1] = 0x55
        self.packet[2] = 51
        self.packet[3] = 0
        self.packet[4] = 0
        self.packet[5] = 0
        self.packet[6] = 0
        self.packet[7] = 0
        self.packet[8] = 0
        self.packet[9] = 101
    def add_echo_back(self, rx_packet):
        self.packet[10:32] = rx_packet[10:32]

    def set_fields(self):
        self.roll = 0
        self.pitch = -180
        self.yaw = 0
        self.uv_position_lat = 12345678
        self.uv_position_lon =  3212345
        self.uv_ground_speed = 23.4
        self.simulator = 1
        self.system_status = 1
        self.emergency_status = 1

        self.system_voltage1 = 12.3
        self.system_current1 = 5.6

        self.system_voltage2 = 34.5
        self.system_current2 = 6.7

    def build_packet(self, rx_packet):
        self.add_header()
        self.add_echo_back(rx_packet)
        self.packet[32] = self.marker_detected # don't use
        self.packet[33] = self.num_sat
        # print('roll origin :', self.roll)
        # print('build roll :', self.get_range(int((self.roll + 180) / 0.00549), 0, 0xffff))
        self.packet[34:36] = struct.pack(">H", np.uint16(self.roll)) # self.packet[34:36] = struct.pack(">H", self.get_range(int((self.roll + 180) / 0.00549), 0, 0xffff))
        self.packet[36:38] = struct.pack(">H",np.uint16(self.pitch)) # self.packet[36:38] = struct.pack(">H", self.get_range(int((self.pitch + 180) / 0.00549), 0, 0xffff))
        self.packet[38:40] = struct.pack(">H", np.uint16(self.yaw)) # self.packet[38:40] = struct.pack(">H", self.get_range(int((self.yaw + 180) / 0.00549), 0, 0xffff))
        
        # print('lon : ', self.uv_position_lon, ' ', np.int32(self.uv_position_lon * 10000000))
        self.packet[40:44] = struct.pack(">l", np.int32(self.uv_position_lat)) # self.packet[40:44] = struct.pack(">l", np.int32(self.uv_position_lat * 10000000))
        self.packet[44:48] = struct.pack(">l", np.int32(self.uv_position_lon)) # self.packet[44:48] = struct.pack(">l", np.int32(self.uv_position_lon * 10000000))
        # self.packet[48:50] = struct.pack(">H", self.get_range(int((self.uv_altitude +100) / 0.0168), 0, 0xffff))
        # self.packet[48:50] = int.to_bytes(int(self.uav_altitude * 100), 2, 'big')   
        self.packet[50] = self.uv_ground_speed
        self.packet[51:53] = struct.pack(">H", np.uint16(self.course_heading)) # self.packet[51:53] = struct.pack(">H", self.get_range(int((self.course_heading+180)/0.00549), 0, 0xffff))#int.to_bytes(int(self.course_heading+180)/0.00549, 2, 'big') 

        self.packet[53] = self.simulator << 7
        self.packet[54] = self.system_status << 7 | self.emergency_status << 6

        self.packet[55:57] = struct.pack(">H", np.uint16(self.system_voltage1 * 100)) #int.to_bytes(int(self.system_voltage1 * 100), 2, 'big')
        self.packet[57] = int(self.system_current1 * 10)
        self.packet[58:60] = struct.pack(">H", np.uint16(self.system_voltage2 * 100)) #int.to_bytes(int(self.system_voltage2 * 100), 2, 'big')    
        self.packet[60] = int(self.system_current2 * 10)

        self.packet[61:63] = struct.pack(">H", np.uint16(sum(self.packet[2:61])))
        # self.packet[61:63] = self.checksum(self.packet)
        # self.packet[61:63] = np.uint16(self.checksum(self.packet))
        return self.packet 

    def checksum(self, packet):
        return sum(packet[2:61])
    
    def fromBuffer(self, buf): # for test
        self.marker_detected = buf[32]
        self.num_sat = buf[33]
        print ('roll : ' , int(struct.unpack(">H", buf[34:36])[0]))
        self.roll = int(struct.unpack(">H", buf[34:36])[0]) * 0.00549 - 180 #(int.from_bytes(buf[34:36], 'big', signed=False) - 180) * 0.00549
        self.pitch =int(struct.unpack(">H", buf[36:38])[0]) * 0.00549 - 180
        self.yaw =  int(struct.unpack(">H", buf[38:40])[0]) * 0.00549 - 180

        self.uv_position_lat = int(struct.unpack(">l", buf[40:44])[0]) * 0.0000001 
        self.usv_position_lon = int.from_bytes(buf[44:48], 'big', signed=True) * 0.0000001
        self.uav_altitude = int.from_bytes(buf[48:50], 'big', signed=True) * 0.0168 - 100
        self.uv_ground_speed = int(buf[50]) * 0.1
        self.course_heading = int.from_bytes(buf[51:53], 'big', signed=True) * 0.00549 - 180
        self.simulator = buf[53] >> 7
        self.system_status = buf[54] >> 7
        self.emergency_status = buf[54] >> 6
        self.system_voltage1 = int.from_bytes(buf[55:57], 'big', signed=True) * 0.01
        self.system_current1 = int(buf[57]) * 0.1
        self.system_voltage2 = int.from_bytes(buf[58:60], 'big', signed=True) * 0.01
        self.system_current2 = int(buf[60]) * 0.1

class Cooperation_Communication_Parser(object):
    def __init__(self):
        self.last_rx_packet = bytearray(34)
        self.tx_msg = Tx_Message()
        self.stick_mode = None
        self.waypoint_mode = None
        self.end_operation_mode = None
        self.emergency_mode = None
        self.rx_header_check_rule = {0:[0xAA], 1:[0x55], 2:[22], 8:[0], 9:[1], 10:[1, 2, 255]}

    
    def build_tx_packet(self, loopback_echo, tx_msg) -> bytearray(63):
        tx_packet = bytearray(63)
        # 1. make header (0 ~ 9)
        tx_packet[0] = 0xAA
        tx_packet[1] = 0x55
        tx_packet[2] = 51
        tx_packet[3] = 0
        tx_packet[4] = 0
        tx_packet[5] = 0
        tx_packet[6] = 0
        tx_packet[7] = 0
        tx_packet[8] = 0
        tx_packet[9] = 101

        # 2. loopback echo - header 제외하고
        tx_packet[10:32] = loopback_echo[10:32] # 10 ~ 31 echo back

        tx_packet[32] = tx_msg.marker_detected
        tx_packet[33] = tx_msg.num_sat
        # tx_packet[] = self.make_tx_header(tx_packet)
        return tx_packet
    
    def parse_rx_packet(self, packet) -> Rx_Message:
        msg = Rx_Message()
        # packet size check
        msg.header1 = packet[0] # 0xAA
        msg.header2 = packet[1] # 0x55
        msg.payload_length = packet[2] # 22
        msg.sequence = packet[3]
        msg.source_id = packet[4]
        msg.source_port = packet[5]
        msg.destination_id = packet[6]
        msg.destination_port = packet[7]
        msg.packet_priority = packet[8]  # 0
        msg.message_id = packet[9] # 1 | 101
        msg.operation_mode = packet[10] # 1, 2, 4, 5, 6, 7, 255
        if msg.operation_mode == 1:
            self.stick_mode = UsvStickMode()
            self.stick_mode.parse_raw(packet)
        elif msg.operation_mode == 2 :
            self.waypoint_mode = WaypointMode()
            self.waypoint_mode.parse_raw(packet)
        elif msg.operation_mode == 3:
            pass # takeoff_mode = TakeoffMode(packet[11:16])
        elif msg.operation_mode == 4:
            pass # landing_mode = LandingMode(packet[11:16])
        elif msg.operation_mode ==255:
            self.emergency_mode = EmergencyMode() # emergency_mode = EmergencyMode(packet[11])
            self.emergency_mode.parse_raw(packet)

        self.target_waypoint_position_lat = packet[21:25]
        self.target_waypoint_position_lon = packet[25:29]

        if self.checksum(packet[2:32], packet[32:34]) == False:
            msg.error_code = RX_CHECKSUM_ERROR

        return msg

    def checksum(self, payload, checksum_part):
        if int.from_bytes(checksum_part, byteorder="big", signed=False) == sum(payload) :
            return True
        return False

    def check_header(self, packet):
        check_index = self.rx_header_check_rule.keys()
        for i in check_index:
            if packet[i] in self.rx_header_check_rule[i]:
                pass
            else:
                return False
        return True

class ConverterTool:
    def __init__(self) -> None:
        pass
    def convert_angle(self, kriso_angle):
        if kriso_angle > 180:
            kriso_angle = kriso_angle - 360
        elif kriso_angle < -180:
            kriso_angle = kriso_angle + 360
        return np.uint16((kriso_angle+180)*65535/360)

    def convert_latlon(self, kriso_latlon):
        return np.int32(kriso_latlon*10000000)

    def convert_altitude(self, kriso_heave):
        return np.uint16((kriso_heave+100)*65535/1100)

    def convert_speed(self, kriso_sog):
        return np.uint8(kriso_sog*0.5144*10)

# uint8 nav_mode          # 항법모드 (single/RTK/INS)
# float32 nav_roll        # degree roll
# float32 nav_pitch       # degree pitch
# float32 nav_yaw         # degree yaw
# float32 nav_yaw_rate    # [degree/s] yaw_rate 
# float32 nav_cog         # degree COG
# float32 nav_sog         # knots degree SOG
# float32 nav_uspd        # konts longitudinal speed
# float32 nav_vspd        # knots traversal speed
# float32 nav_wspd        # knots vertical speed
# float64 nav_longitude   # degree 경도
# float64 nav_latitude    # degree 위도
# float32 nav_heave       # m Heave
# float32 nav_gpstime     #  hhmmss.s GPS시간
# float32 wea_airtem      # degree 공기 온도
# float32 wea_wattem      # degree 해수 온도
# float32 wea_press       # bar 기압
# float32 wea_relhum      # % 상대습도
# float32 wea_dewpt       # degree 이슬점
# float32 wea_windirt     # degree 풍향(절대)
# float32 wea_winspdt     # knots 풍속(절대)
# float32 wea_windirr     # degree 풍향(상대)
# float32 wea_watspdr     # knots 풍속(상대)
# float32 wea_watdir      # degree 유향
# float32 wea_watspd      # knots 유속
# float32 wea_visibiran   # m 가시거리


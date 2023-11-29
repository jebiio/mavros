#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParser.py
# rosserial 실행 : rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode, ConverterTool
from kriso_msgs.msg import FromCooperation as FromCooperation
from kriso_msgs.msg import CmdtoController as CmdToController
from kriso_msgs.msg import WTtoController as WtToController
from kriso_msgs.msg import MTCmd as MtCmd

from kriso_msgs.msg import ToCooperation as ToCooperation
from kriso_msgs.msg import MiddlewareToVcc as MiddlewareToVcc

echo_packet = bytearray(32)
pub_cmd_to_controller = rospy.Publisher('/kriso/cmdtocontroller', CmdToController, queue_size=10)
pub_mt_cmd = rospy.Publisher('/kriso/mt_cmd', MtCmd, queue_size=10)
pub_wt_to_controller = rospy.Publisher('/kriso/wt_to_controller', WtToController, queue_size=10)

received_message = False
received_middleware_msg = None
last_message_time = rospy.Time()
pub_to_acm = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10) 

def from_acm_callback(msg):
    global echo_packet
    parser = Cooperation_Communication_Parser()
    if parser.check_header(msg.packet) == False:
        print('Header is not valid!!')
        return
    # for test msg.packet[32:34] == cal_chk_sum(msg.packet[2:32]):
    # parser.parse_rx_packet(msg.packet)  KRISO 디버깅
    echo_packet = msg.packet[0:32]
    print(msg.packet)
    if parser.stick_mode is not None:
        publish_stick_mode(parser.stick_mode)
        rospy.loginfo("stick_mode")
    if parser.waypoint_mode is not None:
        publish_waypoint_mode(parser.waypoint_mode)
        rospy.loginfo("waypoint_mode")
    if parser.emergency_mode is not None:
        publish_emergency_mode(parser.emergency_mode)
        rospy.loginfo("emergency_mode")

def publish_stick_mode(stick_mode):
    global pub_cmd_to_controller, pub_mt_cmd
    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 6

    mt = MtCmd()
    mt.start = 0
    mt.t1_rpm = 0
    mt.t2_rpm = 0
    mt.t3_rpm = stick_mode.raw_stick_throttle_cmd * 1800/100
    mt.t3_angle = stick_mode.raw_stick_roll_cmd
    mt.t4_rpm = stick_mode.raw_stick_throttle_cmd * 1800/100
    mt.t4_angle = stick_mode.raw_stick_roll_cmd

    pub_cmd_to_controller.publish(cmd)
    pub_mt_cmd.publish(mt)


def publish_waypoint_mode(waypoint_mode):
    global pub_cmd_to_controller, pub_wt_to_controller

    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 7
    cmd.ca_mode = 1

    wt = WtToController()
    wt.global_path[0].lat = waypoint_mode.raw_target_waypoint_position_lat /10000000# / 10000000 해줘야할듯.
    wt.global_path[0].lon = waypoint_mode.raw_target_waypoint_position_lon /10000000# / 10000000 해줘야할듯.
    wt.global_path[0].spd_cmd = waypoint_mode.raw_maximum_speed_cmd
    wt.nav_surge_pgain = 1
    wt.nav_surge_dgain = 1
    wt.nav_yaw_pgain = 1
    wt.nav_yaw_dgain = 1
    wt.count = 1

    pub_cmd_to_controller.publish(cmd)
    pub_wt_to_controller.publish(wt)

def publish_emergency_mode(emergency_mode):
    global pub_cmd_to_controller, pub_mt_cmd

    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 8

    mt = MtCmd()
    mt.start = 0
    mt.t1_rpm = 0
    mt.t2_rpm = 0
    mt.t3_rpm = 0
    mt.t3_angle = 0
    mt.t4_rpm = 0
    mt.t4_angle = 0

    pub_cmd_to_controller.publish(cmd)
    pub_mt_cmd.publish(mt)   


def to_acm_timer_callback(event):
    global pub_to_acm, received_middleware_msg

    if received_middleware_msg:
        current_time = rospy.Time.now()
        if (current_time - last_message_time).to_sec() < 1.0: # 1초 이내 들어오는 경우만 보내기
            # Publish the message if the last message was received within the last second
            rospy.loginfo('MiddlewareToVcc received!')
            converter = ConverterTool()
            tx_msg = Tx_Message()
            tx_msg.roll = converter.convert_angle(received_middleware_msg.nav_roll)
            tx_msg.pitch = converter.convert_angle(received_middleware_msg.nav_pitch)
            tx_msg.yaw = converter.convert_angle(received_middleware_msg.nav_yaw)
            tx_msg.uv_position_lat = converter.convert_latlon(received_middleware_msg.nav_latitude)
            tx_msg.uv_position_lon = converter.convert_latlon(received_middleware_msg.nav_longitude)
            tx_msg.uv_altitude = converter.convert_altitude(received_middleware_msg.nav_heave)
            tx_msg.uv_ground_speed = converter.convert_speed(received_middleware_msg.nav_sog)
            tx_msg.course_heading = converter.convert_angle(received_middleware_msg.nav_cog)
            # tx_msg.build_packet(echo_back)
            
            coop_msg = ToCooperation()
            coop_msg.length = 63
            coop_msg.packet = tx_msg.build_packet(echo_packet)
            pub_to_acm.publish(coop_msg)
        received_middleware_msg = None
        # received_message = False


def middleware_callback(msg):
    global last_message_time, received_middleware_msg
    # received_message = True
    received_middleware_msg = msg
    last_message_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('acm_parser')
    rate = rospy.Rate(5)  # 5 Hz fixed rate

    sub_from_acm = rospy.Subscriber('/kriso/from_cooperation', FromCooperation, from_acm_callback)
    sub_middleware = rospy.Subscriber('/kriso/middleware_to_vcc', MiddlewareToVcc, middleware_callback)

    # 5Hz로 동작
    timer = rospy.Timer(rospy.Duration(0.2), to_acm_timer_callback)

    while not rospy.is_shutdown():        
        rate.sleep()

#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParserFromSerial.py

import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
 
from kriso_msgs.msg import FromCooperation as FromCooperation
from kriso_msgs.msg import CmdtoController as CmdToController
from kriso_msgs.msg import WTtoController as WtToController
from kriso_msgs.msg import MTCmd as MtCmd

# middleware_msg = MiddlewareToVcc()
serial_msg = FromCooperation()
pub_cmd_to_controller = rospy.Publisher('/kriso/cmdtocontroller', CmdToController, queue_size=10)
pub_mt_cmd = rospy.Publisher('/kriso/mt_cmd', MtCmd, queue_size=10)
pub_wt_to_controller = rospy.Publisher('/kriso/wt_to_controller', WtToController, queue_size=10)

def callback_from_cooperation(msg):
    parser = Cooperation_Communication_Parser()
    parser.parse_rx_packet(msg.packet)

    if parser.stick_mode is not None:
        publish_stick_mode(parser.stick_mode)
        rospy.loginfo("stick_mode")
    if parser.waypoint_mode is not None:
        publish_waypoint_mode(parser.waypoint_mode)
        rospy.loginfo("waypoint_mode")
    if parser.emergency_mode is not None:
        publish_emergency_mode(parser.emergency_mode)
        rospy.loginfo("emergency_mode")

    # msg.packet을 parse해서 
    # acm_parser = ACMParser(msg.packet)
    # acm_parser.parse()
    # if acm_parser.stick_mode is not None:
    #     publish_stick_mode()
    # elif acm_parser.waypoint_mode is not None:
    #     publish_waypoint_mode()
    # elif acm_parser.emergency_mode is not None:
    #     publish_emergency_mode()
    # else:
    #     print("Nothing to publish")
    # pub_cmd_to_controller.publish(cmd)
    # pub_mt_cmd.publish(mt)

def publish_stick_mode(stick_mode):
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
    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 7
    cmd.ca_mode = 1

    wt = WtToController()
    wt.global_path[0].lat = waypoint_mode.raw_target_waypoint_position_lat
    wt.global_path[0].lon = waypoint_mode.raw_target_waypoint_position_lon
    wt.global_path[0].spd_cmd = waypoint_mode.raw_maximum_speed_cmd
    wt.nav_surge_pgain = 1
    wt.nav_surge_dgain = 1
    wt.nav_yaw_pgain = 1
    wt.nav_yaw_dgain = 1
    wt.count = 1

    pub_cmd_to_controller.publish(cmd)
    pub_wt_to_controller.publish(wt)

def publish_emergency_mode(emergency_mode):
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

def talker():
    sub = rospy.Subscriber('/kriso/from_cooperation', FromCooperation, callback_from_cooperation)

    # pub_cmd_to_controller = rospy.Publisher('/kriso/cmdtocontroller', CmdToController, queue_size=10)
    # pub_mt_cmd = rospy.Publisher('/kriso/mt_cmd', MtCmd, queue_size=10)
    # pub_wt_to_controller = rospy.Publisher('/kriso/wt_to_controller', WtToController, queue_size=10)

    rospy.init_node('acm_to_serial', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        # serial_msg = ACMParser(middleware_msg) 
        # pub.publish(serial_msg)
        # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

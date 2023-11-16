#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMParserToSerial.py

import rospy
from kriso_msgs.msg import FromCooperation as FromCooperation
from kriso_msgs.msg import CmdToController as CmdToController
from kriso_msgs.msg import WtToController as WtToController
from kriso_msgs.msg import MtCmd as MtCmd

# middleware_msg = MiddlewareToVcc()
serial_msg = FromCooperation()

def callback_from_cooperation(msg):
    pub_cmd_to_controller.publish(cmd)
    pub_mt_cmd.publish(mt)

def publish_stick_mode():
    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 6

    mt = MtCmd()
    mt.start = 0
    mt.t1_rpm = 0
    mt.t2_rpm = 0
    mt.t3_rpm = throttle * 1800/100
    mt.t3_angle = roll
    mt.t4_rpm = throttle * 1800/100
    mt.t4_angle = roll

    pub_cmd_to_controller.publish(cmd)
    pub_mt_cmd.publish(mt)


def publish_waypoint_mode():
    cmd = CmdToController()
    cmd.oper_mode = 2
    cmd.mission_mode = 7
    cmd.ca_mode = 1

    wt = WtToController()
    wt.global_path[0].longitude = lat
    wt.global_path[0].latitude = lon
    wt.global_path[0].speed = speed
    wt.nav_surge_pgain = 1
    wt.nav_surge_dgain = 1
    wt.nav_yaw_pgain = 1
    wt.nav_yaw_dgain = 1
    wt.count = =1

    pub_cmd_to_controller.publish(cmd)
    pub_wt_to_controller.publish(wt)

def publish_emergency_mode():
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
    pub = rospy.Publisher('/kriso/to_cooperation', ToCooperation, queue_size=10)
    pub_cmd_to_controller = rospy.Publisher('/kriso/cmdtocontroller', CmdToController, queue_size=10)
    pub_mt_cmd = rospy.Publisher('/kriso/mt_cmd', MtCmd, queue_size=10)
    pub_wt_to_controller = rospy.Publisher('/kriso/wt_to_controller', WtToController, queue_size=10)

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
        # serial_msg = ACMParser(middleware_msg) 
        # pub.publish(serial_msg)
        # msg.roll += 0.1;

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

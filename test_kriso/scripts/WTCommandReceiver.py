#!/usr/bin/env python3

# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=WTCommandReceiver.py

import threading
import socket
import rospy
from mavros.utils import *
from kriso_msgs.msg import WTtoController as WTtoController
import struct

# Define the format for Waypoint and WaypointControl
waypoint_format = 'ddff'
waypoint_control_format = waypoint_format*100 + '4fB'
IP_ADDRESS = '127.0.0.1'
PORT = 20222

pub = rospy.Publisher('/kriso/wt_to_controller', WTtoController, queue_size=10)

def parse_waypoint(data):
    lat, lon, spd_cmd, acceptance_radius = struct.unpack(waypoint_format, data)
    return {
        'lat': lat,
        'lon': lon,
        'spd_cmd': spd_cmd,
        'acceptance_radius': acceptance_radius
    }

def parse_waypoint_control(data):
    waypoints_data = data[:struct.calcsize(waypoint_format * 100)]
    waypoints = [
        parse_waypoint(waypoints_data[i:i+struct.calcsize(waypoint_format)])
        for i in range(0, len(waypoints_data), struct.calcsize(waypoint_format))
    ]
    print('100 * waypoint_format size:', struct.calcsize(waypoint_format*100))

    rest_data = data[struct.calcsize(waypoint_format * 100):]
    print('size rest:', len(rest_data))
    nav_surge_pgain, nav_surge_dgain, nav_yaw_pgain, nav_yaw_dgain, count = struct.unpack('4fB', rest_data)
    return {
        'global_path': waypoints,
        'nav_surge_pgain': nav_surge_pgain,
        'nav_surge_dgain': nav_surge_dgain,
        'nav_yaw_pgain': nav_yaw_pgain,
        'nav_yaw_dgain': nav_yaw_dgain,
        'count': count
    }

def receive_udp():
    # udp server 생성 : ip주소는 192.168.0.2, port는 20222
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP_ADDRESS, PORT))

    # 데이터 받기
    while True:
        data, addr = sock.recvfrom(6000)
        print(len(data))
        wt = WTtoController()
        result = parse_waypoint_control(data)
        # result['global_path']를 wt.global_path에 복사하기
        global_paths = result['global_path']
        for i in range(len(global_paths)):
            wt.global_path[i].lat = global_paths[i]['lat']
            wt.global_path[i].lon = global_paths[i]['lon']
            wt.global_path[i].spd_cmd = global_paths[i]['spd_cmd']
            wt.global_path[i].acceptance_radius = global_paths[i]['acceptance_radius']    
        wt.nav_surge_pgain = result['nav_surge_pgain']
        wt.nav_surge_dgain = result['nav_surge_dgain']
        wt.nav_yaw_pgain = result['nav_yaw_pgain']
        wt.nav_yaw_dgain = result['nav_yaw_dgain']
        wt.count = result['count']
        pub.publish(wt)

def talker():
    rospy.init_node('kriso_udp_receiver_wt_to_controller', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    # Create a thread for receive_udp() and start it
    udp_thread = threading.Thread(target=receive_udp)
    udp_thread.start()
    
    while not rospy.is_shutdown():
        rate.sleep()
    # thread close
    udp_thread.join()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

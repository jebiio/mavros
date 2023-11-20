#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
from comm_module import Cooperation_Communication_Parser, Rx_Message , Tx_Message, UsvStickMode, WaypointMode, EmergencyMode
from kriso_msgs.msg import ToCooperation as ToCooperation
# 실행 방법 : > roslaunch test_kriso kriso_test.launch msg:=ACMReceiver.py


def callback_receive(msg):
    if msg.length == 63:
        # print msg.packet
        rospy.loginfo("middleware info received")
    else:
        rospy.loginfo("Received packet length is not 63")


    
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

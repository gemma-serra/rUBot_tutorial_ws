#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def pong_callback(msg):
    pong_msg = String()
    if (msg.data == "Ping"):
        pong_msg.data = "Pong"
    else:
        pong_msg.data = "Failed!"
    pub.publish(pong_msg)
    rospy.loginfo(pong_msg.data)

rospy.init_node('pong_node')
sub = rospy.Subscriber('/ping', String, pong_callback)
pub = rospy.Publisher('/pong', String, queue_size=10)
rospy.spin()

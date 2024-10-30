#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo(f"Received from Unity - Moving: {data.data}")

if __name__ == '__main__':
    rospy.init_node('raspimouse_toggle_subscriber')
    rospy.Subscriber("/unity_movement_feedback", Bool, callback)
    rospy.spin()
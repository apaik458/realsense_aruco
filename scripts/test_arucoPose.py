#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

class Listener:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/pose_topic", PoseStamped, self.callback)

    def callback(self, data):
        print(data.pose.position.x, data.pose.position.y, data.pose.position.z,
              data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
Listener()
rospy.init_node('listener', anonymous=True)
rospy.spin()

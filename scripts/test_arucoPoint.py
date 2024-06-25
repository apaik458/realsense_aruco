#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

class Listener:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/point_topic", Point, self.callback)

    def callback(self, data):
        print(data.x, data.y, data.z)
    
Listener()
rospy.init_node('listener', anonymous=True)
rospy.spin()

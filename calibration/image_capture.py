#!/usr/bin/env python3
import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Aruco:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.image_counter = 1

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv.imshow("Image window", cv_image)

        if cv.waitKey(3) == ord(' '):
            image_name = "calibration_image_{}.png".format(self.image_counter)
            cv.imwrite("/home/user/temp_ws/src/youbot_visual_servo/camera_calibration/images/" + image_name, cv_image)
            print("{} written".format(image_name))
            self.image_counter += 1
            
            if self.image_counter == 11:
                rospy.signal_shutdown("Closing windows")

Aruco()

rospy.init_node('aruco', anonymous=True)

rospy.spin()
cv.destroyAllWindows()

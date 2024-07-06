#!/usr/bin/env python3
import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class Aruco:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Note: previous cv2 versions use cv.aruco.Dictionary_get()
        # and cv.aruco.DetectorParameters_create()
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
        self.arucoParams = cv.aruco.DetectorParameters()

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.point_pub = rospy.Publisher("/point_topic", Point, queue_size=10)
        self.point_data = Point()

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv.aruco.detectMarkers(cv_image, self.arucoDict, parameters=self.arucoParams)
        
        if ids is not None:
            cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
            
            # compute the center of the contour
            M = cv.moments(corners)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # draw the center of the shape on the image
            cv.circle(cv_image, (cX, cY), 5, (0,255,0), cv.FILLED)

            self.point_data.x = cX
            self.point_data.y = cY
            self.point_data.z = 0

            self.point_pub.publish(self.point_data)

        cv.imshow("Image window", cv_image)

        if cv.waitKey(3) == ord('q'):
            rospy.signal_shutdown("Closing windows")

def main():
    Aruco()

    rospy.init_node('aruco', anonymous=True)

    rospy.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
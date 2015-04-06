#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class Detector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber("/camera/image_raw", Image, self.callback)
        rospy.init_node("step01", anonymous=True)

    def __del__(self):
        rospy.loginfo("Shutting down detector")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except cv_bridge.CvBridgeError, e:
             print e
        cv2.imshow("Debug", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        d = Detector()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

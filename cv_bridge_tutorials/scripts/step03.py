#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class Detector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node("step03", anonymous=True)
        rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.publisher = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

    def __del__(self):
        rospy.loginfo("Shutting down detector")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except cv_bridge.CvBridgeError, e:
             print e
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        debug_rgb_image = np.concatenate((cv_image[:,:,0],cv_image[:,:,1],cv_image[:,:,2]),1)
        debug_hsv_image = np.concatenate((hsv_image[:,:,0],hsv_image[:,:,1],hsv_image[:,:,2]),1)
        debug_image = np.concatenate((debug_rgb_image,debug_hsv_image),0)

        outmsg = self.bridge.cv2_to_imgmsg(debug_image, encoding='mono8')
        self.publisher.publish(outmsg)

if __name__ == '__main__':
    try:
        d = Detector()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

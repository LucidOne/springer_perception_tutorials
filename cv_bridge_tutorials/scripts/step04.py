#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class Detector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node("step04", anonymous=True)
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

        output_image = np.zeros(hsv_image[:,:,0].shape, dtype=np.uint8)
        it = np.nditer(hsv_image[:,:,0], flags=['multi_index'])
        while not it.finished:
            # Segment by Hue
            if it[0] > 28 and it[0] < 45:
                # Segment by Saturation
                if hsv_image[it.multi_index + (1,)] > 64:
                    output_image[it.multi_index] = 255
            it.iternext()


        outmsg = self.bridge.cv2_to_imgmsg(output_image, encoding='mono8')
        self.publisher.publish(outmsg)

if __name__ == '__main__':
    try:
        d = Detector()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

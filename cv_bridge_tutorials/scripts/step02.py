#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class Detector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node("step02", anonymous=True)
        rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.publisher = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

    def __del__(self):
        rospy.loginfo("Shutting down detector")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except cv_bridge.CvBridgeError, e:
             print e
        cv2.putText(cv_image,"DEBUG",(16,32),cv2.FONT_HERSHEY_PLAIN,1,(255,0,0))
        outmsg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self.publisher.publish(outmsg)

if __name__ == '__main__':
    try:
        d = Detector()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

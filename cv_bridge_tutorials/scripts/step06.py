#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class Detector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node("step06", anonymous=True)
        rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.publisher = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

    def __del__(self):
        rospy.loginfo("Shutting down detector")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except cv_bridge.CvBridgeError, e:
             print e
        cv_image = cv2.resize(cv_image,(cv_image.shape[1]/4,cv_image.shape[0]/4))
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        filtered_image = np.zeros(hsv_image[:,:,0].shape, dtype=np.uint8)
        it = np.nditer(hsv_image[:,:,0], flags=['multi_index'])
        while not it.finished:
            # Segment by Hue
            if it[0] > 29 and it[0] < 45:
                # Segment by Saturation
                if hsv_image[it.multi_index + (1,)] > 75:
                    filtered_image[it.multi_index] = 255
            it.iternext()
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
        dilated_image = cv2.dilate(filtered_image,kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
        output_image = cv2.erode(dilated_image,kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        output_image = cv2.dilate(output_image,kernel)
        output_image = cv2.dilate(output_image,kernel)
        output_image = cv2.bitwise_not(output_image)
        output_image = cv2.GaussianBlur(output_image,(7,7),9)
        # Fix for OpenCV bug
        if hasattr(cv2, 'HOUGH_GRADIENT'):
            circles = cv2.HoughCircles(output_image,cv2.HOUGH_GRADIENT,4,30)
        else:
            circles = cv2.HoughCircles(output_image,cv2.cv.CV_HOUGH_GRADIENT,4,30)
        if circles is not None:
            rospy.loginfo(circles)
            for i in circles[0,:]:
                cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)

        outmsg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self.publisher.publish(outmsg)

if __name__ == '__main__':
    try:
        d = Detector()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

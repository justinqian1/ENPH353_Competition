#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class PlateDetector:
    """
    @class PlateDetector
    @brief Class to detect plate

    Processes images from camera live feed and takes a picture once plate is found, to send over **ROSTOPIC**
    """
    def platemask(self, image):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        b_minus_g=channel_b-channel_g
        b_minus_r=channel_b-channel_r
        g_minus_r=channel_g-channel_r

        sign_mask=(b_minus_g>90) & (b_minus_g<110) & (b_minus_r>90) & (b_minus_r<110)
        analysis_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        analysis_mask[sign_mask] = 255

        sign_count = np.count_nonzero(sign_mask)
        self.pixel_pub.publish(sign_count)

        contours, _ = cv2.findContours(analysis_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            # Publish zeros when nothing detected
            self.area_pub.publish(0)
            self.cx_pub.publish(-1.0)    # -1 means “no detection”
            return

        # Pick contour with largest area
        c = max(contours, key=cv2.contourArea)
        area = int(cv2.contourArea(c))

        # Bounding box
        x, y, w, h = cv2.boundingRect(c)

        # Centroid (of the bbox, not the contour)
        cx = float(x + w/2.0)                

        cv2.imshow('camera feed', image)
        cv2.imshow('plate', analysis_mask)
        cv2.waitKey(1)

    def __init__(self):
        self.pixel_pub = rospy.Publisher("/plate_pixel_count", Int32, queue_size=10)
        self.area_pub = rospy.Publisher("/plate/largest_area", Int32, queue_size=10)
        self.cx_pub = rospy.Publisher("/plate/centroid_x", Float32, queue_size=10)
        image_topic = rospy.get_param("~image_topic", "/B1/rrbot/camera1/image_raw")
        self.image_sub = rospy.Subscriber(
            image_topic, Image, self.callback, queue_size=1
        )
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        self.platemask(cv_image)
        return

def main():
    rospy.init_node('plate_detector', anonymous=True)
    ic = PlateDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from time import time
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

SIGN_MASK = [110, 90, 110, 90, 256, -256]
GRAY_MASK = [10, -10, 10, -10, 10, -10]
TEXT_MASK = [256, 30, 256, 30, 256, -256]
MASK_DICT = {"b_g_upper":0, "b_g_lower":1, "b_r_upper":2,
             "b_r_lower":3, "g_r_upper":4, "g_r_lower":5}
MIN_MASK_SIZE = 4000
MIN_PLATE_AREA = 3000
OUTPUT_SHAPE = (400,200)
COOL_DOWN_TIME = 5.0

class PlateDetector:
    """
    @class PlateDetector
    @brief Class to detect plate

    Processes images from camera live feed and takes a picture once plate is found, to send over **ROSTOPIC**
    """
    def apply_mask(self, image, mask):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        b_minus_g=channel_b-channel_g
        b_minus_r=channel_b-channel_r
        g_minus_r=channel_g-channel_r

        feature_mask = (b_minus_g>mask[MASK_DICT["b_g_lower"]]) & (b_minus_g<mask[MASK_DICT["b_g_upper"]]) & (b_minus_r>mask[MASK_DICT["b_r_lower"]]) & (b_minus_r<mask[MASK_DICT["b_r_upper"]]) & (g_minus_r>mask[MASK_DICT["g_r_lower"]]) & (g_minus_r<mask[MASK_DICT["g_r_upper"]]) 

        analysis_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        analysis_mask[feature_mask] = 255

        return analysis_mask

    def scan_sign(self):
        """

        """
        images = [self.centre_image, self.left_image, self.right_image]

        for image in images:
            mask = self.apply_mask(image, SIGN_MASK)
            print(np.count_nonzero(mask))
            if np.count_nonzero(mask) < MIN_MASK_SIZE:
                continue

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            sign_cnt = max(contours, key=cv2.contourArea)
            req_acc = 0.02 * cv2.arcLength(sign_cnt, True)
            approx = cv2.approxPolyDP(sign_cnt, req_acc, True)

            print("Area: " + str(cv2.contourArea(approx)))

            if len(approx) != 4 or cv2.contourArea(approx) < MIN_PLATE_AREA:
                continue 

            print("How about here?")
            corners = approx.reshape(4, 2).astype(np.float32)

            # Sanity reminder: top left (0,0), bottom right (w,h).
            def order_pts(pts):
                sum_xy = pts.sum(axis=1)
                diff_xy = np.diff(pts, axis=1).reshape(-1)
                tl = pts[np.argmin(sum_xy)]
                br = pts[np.argmax(sum_xy)]
                tr = pts[np.argmin(diff_xy)]
                bl = pts[np.argmax(diff_xy)]
                return np.array([tl, tr, br, bl])

            # Now we order the points (must find 
            corners_ordered = order_pts(corners)

            # Defining what we want the output rectangle to be.
            W, H = OUTPUT_SHAPE
            dest = np.array([
                [0,0],
                [W - 1, 0],
                [W - 1, H - 1],
                [0, H - 1],
                ], dtype=np.float32)

            M = cv2.getPerspectiveTransform(corners_ordered, dest)
            plate_rectified = cv2.warpPerspective(image, M, (W, H))

            return plate_rectified
        return None



    def __init__(self):
        self.curr_plate = 1
        self.last_scan_time = time()

        self.lpixel_pub = rospy.Publisher("/left_plate_pixel_count", Int32, queue_size=10)
        self.cpixel_pub = rospy.Publisher("/centre_plate_pixel_count", Int32, queue_size=10)
        self.rpixel_pub = rospy.Publisher("/right_plate_pixel_count", Int32, queue_size=10)
        #self.area_pub = rospy.Publisher("/plate/largest_area", Int32, queue_size=10)
        #self.cx_pub = rospy.Publisher("/plate/centroid_x", Float32, queue_size=10)
        centre_image_topic = rospy.get_param("~centre_image_topic", "/B1/rrbot/camera1/image_raw")
        left_image_topic = rospy.get_param("~left_image_topic", "/B1/rrbot/lcam/image_raw")
        right_image_topic = rospy.get_param("~right_image_topic", "/B1/rrbot/rcam/image_raw")
        self.centre_image_sub = rospy.Subscriber(
            centre_image_topic, Image, self.centre_callback, queue_size=1
        ) 
        self.left_image_sub = rospy.Subscriber(
            left_image_topic, Image, self.left_callback, queue_size=1
        )
        self.right_image_sub = rospy.Subscriber(
            right_image_topic, Image, self.right_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.callback)
        self.centre_image = None
        self.left_image = None
        self.right_image = None
        self.bridge = CvBridge()

    def callback(self, event):
        if time() - self.last_scan_time < COOL_DOWN_TIME:
            return
        if self.centre_image is None or self.left_image is None or self.right_image is None:
            return
        poss_plate=self.scan_sign()
        if poss_plate is not None:
            cv2.imshow("Plate " + str(self.curr_plate), poss_plate)
            cv2.waitKey(3)
            self.curr_plate += 1
            self.last_scan_time = time()

    def centre_callback(self, data):
        try:
            self.centre_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        return

    def left_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        return

    def right_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
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

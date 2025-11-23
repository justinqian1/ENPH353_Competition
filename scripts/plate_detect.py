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
MIN_BLUE_COUNT = 10000
MIN_PLATE_AREA = 24000
MIN_SIGN_COUNT = 100
MIN_SIGN_AREA = 100
OUTPUT_SHAPE = (800,400)
COOL_DOWN_TIME = 5.0
CHAR_WIDTH_PROP = 0.077
CHAR_HEIGHT_PROP = 0.152
BOT_ROW_Y = 268/400
TOP_ROW_Y = 45/400
BOT_ROW_X = 0.044
TOP_ROW_X = 332/800
CHARS_TOP_ROW = 7
CHARS_BOT_ROW = 12


class PlateDetector:
    """
    @class PlateDetector
    @brief Class to detect plate

    Processes images from camera live feed and takes a picture once plate is found, to send over **ROSTOPIC**
    """

    def __init__(self):
        self.curr_plate = 1
        self.last_scan_time = time()

        self.pixel_pub = rospy.Publisher("/plate/pixel_count", Int32, queue_size=10)
        self.area_pub = rospy.Publisher("/plate/largest_area", Int32, queue_size=10)

        left_image_topic = rospy.get_param("~left_image_topic", "/B1/rrbot/lcam/image_raw")
        right_image_topic = rospy.get_param("~right_image_topic", "/B1/rrbot/rcam/image_raw")
        self.left_image_sub = rospy.Subscriber(
            left_image_topic, Image, self.left_callback, queue_size=1
        )
        self.right_image_sub = rospy.Subscriber(
            right_image_topic, Image, self.right_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.callback)
        self.left_image = None
        self.right_image = None
        self.bridge = CvBridge()

    def callback(self, event):
        if time() - self.last_scan_time < COOL_DOWN_TIME:
            return
        if self.left_image is None or self.right_image is None:
            return
        poss_plate=self.scan_sign(SIGN_MASK, MIN_BLUE_COUNT, MIN_PLATE_AREA)
        if poss_plate is not None:
            poss_sign=self.extract_process_plate(poss_plate, GRAY_MASK, MIN_SIGN_COUNT, MIN_SIGN_AREA)
            if poss_sign is not None:                
                extracted_text = self.apply_mask(poss_sign, TEXT_MASK)
                #isolated_contours = self.isolate_letters(extracted_text)
                isolated_contours = self.text_predefined_boxes(extracted_text)
                cv2.imshow("Plate " + str(self.curr_plate), isolated_contours)
                cv2.waitKey(3)
                self.curr_plate += 1
                self.last_scan_time = time()

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

    def apply_mask(self, image, mask):
        """
        @brief applies and binarizes channel difference mask
        @param image, the input image
        @param mask, the difference mask to apply
        @returns binarized applied channel difference mask.
        """
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

    def extract_process_plate(self, frame, mask, min_count, min_area, output_shape=OUTPUT_SHAPE):
        """
        @brief extracts and unskews largest masked quadrilateral in frame
        @param frame the image to analyze
        @param mask the mask to apply to find prospective figures
        @param min_count the minimal pixel count required
        """
        analysis_mask = self.apply_mask(frame, mask)

        pixel_count = np.count_nonzero(analysis_mask)
        self.pixel_pub.publish(pixel_count)

        if pixel_count < min_count:
            return None

        contours, _ = cv2.findContours(analysis_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        sign_cnt = max(contours, key=cv2.contourArea)

        req_acc = 0.02 * cv2.arcLength(sign_cnt, True) # tells approxPolyDP maximal distance allowed between contour and simplified polygon.
        approx = cv2.approxPolyDP(sign_cnt, req_acc, True)
        area = cv2.contourArea(approx)
        
        self.area_pub.publish(int(area))

        # If the shape of the largest contour isn't a rectangle, it's likely some of the border is off the screen, so we shouldn't trust that the text will be fully included.
        if len(approx) != 4  or area < min_area:
            return None

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
        W, H = output_shape
        dest = np.array([
            [0,0],
            [W - 1, 0],
            [W - 1, H - 1],
            [0, H - 1],
            ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(corners_ordered, dest)
        plate_rectified = cv2.warpPerspective(frame, M, (W, H))

        return plate_rectified

    def scan_sign(self, mask, min_count, min_area):
        """

        """
        images = [self.left_image, self.right_image]

        for image in images:
            poss_plate = self.extract_process_plate(image, mask, min_count, min_area)
            if poss_plate is not None:
                return poss_plate
        return None

    def isolate_letters(self, text_img):
        contours, _ = cv2.findContours(text_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img_width, img_height = text_img.shape
        w = int(img_width * CHAR_WIDTH_PROP)
        h = int(img_height * CHAR_HEIGHT_PROP)
        for cnt in contours:
            x, y, _w, _h = cv2.boundingRect(cnt)
            print("[top left](x,y): (" + str(x) + ", " + str(y) + "), (w, h): (" + str(_w) + ", " + str(_h) + ")")
    
            # Draw the rectangle on the original image
            # (image, top-left corner, bottom-right corner, color, thickness)
            cv2.rectangle(text_img, (x, y), (x + _w, y + _h), (255, 255, 0), 1)

        return text_img

    def text_predefined_boxes(self, text_img):
        img_height, img_width = text_img.shape
        print("Image width: " + str(img_width) + " Image height: " + str(img_height))
        w = int(img_width * CHAR_WIDTH_PROP)
        h = int(img_height * CHAR_HEIGHT_PROP)
        x = int(img_width * TOP_ROW_X)
        y = int(img_height * TOP_ROW_Y)
        print("Top left of top row: (" + str(x) + ", " + str(y) + ").")
        for char in range(CHARS_TOP_ROW):
            cv2.rectangle(text_img, (x, y), (x + w, y + h), (255, 255, 0), 1)
            x += w

        x = int(img_width * BOT_ROW_X)
        y = int(img_height * BOT_ROW_Y)
        print("Top left of bottom row: (" + str(x) + ", " + str(y) + ").")
        for char in range(CHARS_BOT_ROW):
            cv2.rectangle(text_img, (x, y), (x + w, y + h), (255, 255, 0), 1)
            x += w

        return text_img



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

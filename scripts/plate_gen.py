#!/usr/bin/env python3

from __future__ import print_function
from typing import Tuple
import rospy
import math
import random
import csv
import cv2
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import ImageFont, ImageDraw

# POSITIONS
# [x, y, z, ox, oy, oz, w]
POS_PLATE_1=[5.489,2.00,0.04,0,0,-1.76]
POS_PLATE_2=[5.47,-0.97,0.04,0,0,-1.45]
POS_PLATE_3=[4.41,-1.77,0.04,0,0,2.29]
POS_PLATE_4=[0.504,-0.81,0.04,0,0,1.644]
POS_PLATE_5=[0.646,2.05,0.04,0,0,-2.43]
POS_PLATE_6=[-3.07,1.43,0.04,0,0,-3.09]
POS_PLATE_7=[-4.29,-1.85,0.04,0,0,-0.898]
POS_PLATE_8=[-1.39,-1.36,1.85,0.01,0.0375,-0.03]
POS_PLATES=[
        POS_PLATE_1, POS_PLATE_2, POS_PLATE_3, POS_PLATE_4, 
        POS_PLATE_5, POS_PLATE_6, POS_PLATE_7, POS_PLATE_8
        ]
POS_VAR = 0.1
W_VAR = 0.1
CHAR_WIDTH_PROP = 0.077
CHAR_HEIGHT_PROP = 0.152
BOT_ROW_Y = 268/400
TOP_ROW_Y = 45/400
BOT_ROW_X = 0.046
TOP_ROW_X = 333/800
CHARS_TOP_ROW = 7
CHARS_BOT_ROW = 12
MIN_BLUE_COUNT = 10000
MIN_PLATE_AREA = 24000
MIN_SIGN_COUNT = 100
MIN_SIGN_AREA = 100
OUTPUT_SHAPE = (800,400)
# MASKS: [b_minus_g_upper, b_minus_g_lower, b_minus_r_upper,
#         b_minus_r_lower, g_minus_r_upper, g_minus_r_lower]
SIGN_MASK = [110, 90, 110, 90, 256, -256]
GRAY_MASK = [10, -10, 10, -10, 10, -10]
TEXT_MASK = [256, 50, 256, 50, 256, -256]
MASK_DICT = {"b_g_upper":0, "b_g_lower":1, "b_r_upper":2,
             "b_r_lower":3, "g_r_upper":4, "g_r_lower":5}
CSV_PATH = '/home/fizzer/ros_ws/src/2025_competition/enph353/enph353_gazebo/scripts/plates.csv'
OUTPUT_PATH = '/home/fizzer/labelled_chars'

class PlateGenerator:
    """
    @class PlateGenerator
    @brief Class to generate data for plate processing.

    Teleports robot to approximate picture taking positions, and then takes pictures and uploads them to data collection folder for image processing.
    """

    def __init__(self):
        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_pub = rospy.Publisher(position_topic, Float32MultiArray, queue_size=10)

        left_image_topic = rospy.get_param("~left_image_topic", "/B1/rrbot/lcam/image_raw")
        right_image_topic = rospy.get_param("~right_image_topic", "/B1/rrbot/rcam/image_raw")
        self.left_image_sub = rospy.Subscriber(
            left_image_topic, Image, self.left_callback, queue_size=1
        )
        self.right_image_sub = rospy.Subscriber(
            right_image_topic, Image, self.right_callback, queue_size=1
        )
        self.plate_pics = []
        self.left_image = None
        self.right_image = None
        self.bridge = CvBridge()

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
    
    def extract_process_plate(self, frame, mask, output_shape=(400,200)):
        analysis_mask = self.apply_mask(frame, mask)

        contours, _ = cv2.findContours(analysis_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            cv2.imshow("Failed blue mask: ", frame)
            return None

        sign_cnt = max(contours, key=cv2.contourArea)

        req_acc = 0.02 * cv2.arcLength(sign_cnt, True) # tells approxPolyDP maximal distance allowed between contour and simplified polygon.
        approx = cv2.approxPolyDP(sign_cnt, req_acc, True)

        # If the shape of the largest contour isn't a rectangle, it's likely some of the border is off the screen, so we shouldn't trust that the text will be fully included.
        if len(approx) != 4:
            cv2.imshow("Blue mask passed num count, not contour shape: ", frame)
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

    def eul_to_qua(self, eul_rep):
        quat = R.from_euler('xyz', eul_rep[-3:], degrees=False).as_quat()
        return eul_rep[:3] + quat.tolist()

    def scan_sign(self, mask):
        """

        """
        images = [self.left_image, self.right_image]

        for image in images:
            poss_plate = self.extract_process_plate(image, mask)
            if poss_plate is not None:
                return poss_plate
        return None

    def isolate_letters(self, text_img, clue, value):
        img_to_show = np.copy(text_img)
        contours, _ = cv2.findContours(text_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        upper_char_rects = []
        lower_char_rects = []


        img_height, img_width = img_to_show.shape
 
        def store_in_right_row(x,y,w,h):
            if y >= int(img_height / 2):
                lower_char_rects.append((x,y,w,h))
            else:
                upper_char_rects.append((x,y,w,h))

        w = int(img_width * CHAR_WIDTH_PROP)
        h = int(img_height * CHAR_HEIGHT_PROP)
        lastCharX = 0 # Tracks the left column of a given char, for handling of several contours in one letter.
        print("Expected width: " + str(w) + " Expected height: " + str(h))
        for cnt in contours:
            x, y, _w, _h = cv2.boundingRect(cnt)
            if _w > int(1.5 * w):
                print("Width: " + str(_w))
                chars_jumb = math.ceil(_w / w)
                avg_width = int(_w / chars_jumb)

                for char_to_parse in range(chars_jumb):
                    cv2.rectangle(img_to_show, (x + char_to_parse * avg_width, y), (x + (char_to_parse + 1) * avg_width, y + _h), (255, 255, 0), 1)
                    store_in_right_row(x + char_to_parse * avg_width, y, avg_width, _h)

            # If a letter contains some contour that's very thin, that indicates to us it was chopped in several pieces.
            elif _w < int(0.6 * w): 
                # If we're sufficiently far from the beginning of the last letter, we know this is the first small chunk, so we manually create bounding box.
                if (x - lastCharX) > int(0.8 * w): 
                    # MIGHT FAIL ON Q? (unless we increase the default height of a letter)
                    cv2.rectangle(img_to_show, (x, y), (x + w, y + h), (255, 255, 0), 1) 
                    store_in_right_row(x, y, w, h)
                    # Now, if there's another small contour in the same letter that's not part of the next letter, it won't get pass this if test, so no box will be drawn:)
                    x = lastCharX 
            else: 
                # Draw the rectangle on the original image
                # (image, top-left corner, bottom-right corner, color, thickness)
                cv2.rectangle(img_to_show, (x, y), (x + _w, y + _h), (255, 255, 0), 1)  
                store_in_right_row(x, y, _w, _h)

        upper_char_rects.sort(key=lambda r: r[0])
        lower_char_rects.sort(key=lambda r: r[0])

        print("Upper character length: " + str(len(upper_char_rects)))

        for char_idx in range(len(clue)):
            x,y,w,h = upper_char_rects[char_idx]
            print("Top left corner: (" + str(x) + ", " + str(y) + "), Width: " + str(w) + ", Height: " + str(h))
            char_img = text_img[y:y+h, x:x+w]
            char_name = clue + str(char_idx) + clue[char_idx] + ".png"
            full_path = os.path.join(OUTPUT_PATH, char_name)
            cv2.imwrite(full_path, self.pad32(char_img))


        for char_idx in range(len(value)):
            x,y,w,h = lower_char_rects[char_idx]
            print("Top left corner: (" + str(x) + ", " + str(y) + "), Width: " + str(w) + ", Height: " + str(h))
            char_img = text_img[y:y+h, x:x+w]
            char_name = value + str(char_idx) + value[char_idx] + ".png"
            full_path = os.path.join(OUTPUT_PATH, char_name)
            cv2.imwrite(full_path, self.pad32(char_img))

        return img_to_show

    # From ChatGPT!
    def pad32(self, img):
        h, w = img.shape
        if h>32 or w>32:
            s = 32/max(h,w)
            img = cv2.resize(img,(int(w*s),int(h*s)),cv2.INTER_NEAREST)
            h,w = img.shape
        t = (32-h)//2; b = 32-h-t
        l = (32-w)//2; r = 32-w-l
        return cv2.copyMakeBorder(img,t,b,l,r,cv2.BORDER_CONSTANT,0)

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

    def add_var(self, qua_rep):
        pos_noise = random.uniform(-POS_VAR, POS_VAR)
        #w_noise = random.uniform(-W_VAR, W_VAR)
        w_noise = W_VAR
        for pos_ind in range(2):
            qua_rep[pos_ind] += pos_noise
        qua_rep[-1] += w_noise
        return qua_rep

    def get_clues(self):
        clues = []

        with open(CSV_PATH, 'r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            for row in reader:
                if row:
                    clues.append(tuple(row))
        return clues


def main():
    rospy.init_node('plate_generator', anonymous=True)
    ic = PlateGenerator()
    position = [0,0,0,0,0,0,0]
    clues = ic.get_clues()
    msg = Float32MultiArray()
    rospy.sleep(0.3)
    for plate in range(len(POS_PLATES)):
        #position = ic.add_var(ic.eul_to_qua(POS_PLATES[plate]))
        print("Topic: " + clues[plate][0] + ", Clue: " + clues[plate][1])
        position = ic.eul_to_qua(POS_PLATES[plate])
        msg.data = position
        rospy.sleep(0.05)
        ic.pos_pub.publish(msg)
        rospy.sleep(0.4)
        poss_plate=ic.scan_sign(SIGN_MASK)
        if poss_plate is not None:
            poss_sign = ic.extract_process_plate(poss_plate, GRAY_MASK)
            if poss_sign is not None:
                extracted_text = ic.apply_mask(poss_sign, TEXT_MASK)
                isolated_contours = ic.isolate_letters(extracted_text, clues[plate][0], clues[plate][1])
                #isolated_contours = ic.text_predefined_boxes(extracted_text)
                ic.plate_pics.append(isolated_contours)
                cv2.imshow('Plate' + str(plate+1), isolated_contours) # CONT here next 16:08
                cv2.waitKey(10)
            else:
                print("Plate " + str(plate+1) + " didn't pass gray mask???")
        else: 
            print("Plate " + str(plate+1) + " Didn't pass blue mask?")
        rospy.sleep(0.21)

    while(True):
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass # Handles potential interruptions cleanly

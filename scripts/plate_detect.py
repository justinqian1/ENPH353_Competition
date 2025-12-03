#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
import csv
import math
import os
import tensorflow as tf
from sensor_msgs.msg import Image
from time import time
from std_msgs.msg import Int32MultiArray, String
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

SIGN_MASK = [110, 90, 110, 90, 256, -256]
GRAY_MASK = [10, -10, 10, -10, 10, -10]
TEXT_MASK = [256, 50, 256, 50, 256, -256]
MASK_DICT = {"b_g_upper":0, "b_g_lower":1, "b_r_upper":2,
             "b_r_lower":3, "g_r_upper":4, "g_r_lower":5}
MIN_BLUE_COUNT = 8000
MIN_PLATE_AREA = 16000
MIN_SIGN_COUNT = 100
MIN_SIGN_AREA = 100
MIN_CNT_SIZE = 50
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

TEAMID = "team5"
PASSWORD = "password"

CLUE_TOPICS = {'SIZE': 1, 'VICTIM': 2, 'CRIME': 3, 'TIME': 4, 'PLACE': 5, 'MOTIVE': 6, 'WEAPON': 7, 'BANDIT': 8}
CSV_PATH = '/home/fizzer/ros_ws/src/2025_competition/enph353/enph353_gazebo/scripts/plates.csv'
OUTPUT_PATH = '/home/fizzer/cnn_train/test_chars'
MODEL_PATH = '/home/fizzer/cnn_train/char_reader_cnn.tflite'
POSS_CHARS = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 
              'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 
              'W', 'X', 'Y', 'Z', '0', '1', '2', '3', '4', '5', '6',
              '7', '8', '9'
              ]


class PlateDetector:
    """
    @class PlateDetector
    @brief Class to detect plate

    Processes images from camera live feed and takes a picture once plate is found, to send over **ROSTOPIC**
    """

    def __init__(self):
        # Load the TFLite model
        self.interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()

        # Get input/output indices
        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_index = self.interpreter.get_output_details()[0]["index"]

        self.curr_plate = 1
        self.last_scan_time = time()

        #For CNN training!
        self.clues = self.get_clues()

        self.location_pub = rospy.Publisher('/location', String, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
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
            # cv2.imshow("Possible plate " + str(self.curr_plate), poss_plate)
            poss_sign=self.extract_process_plate(poss_plate, GRAY_MASK, MIN_SIGN_COUNT, MIN_SIGN_AREA)
            if poss_sign is not None:                
                # extracted_text = self.apply_mask(poss_sign, TEXT_MASK)
                # isolated_contours = self.find_word(extracted_text)
                isolated_contours = self.find_word(poss_sign, TEXT_MASK)
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
        if self.curr_plate == 1 or self.curr_plate == 2:
            min_count -= 4000
            min_area -= 2000

        analysis_mask = self.apply_mask(frame, mask)

        pixel_count = np.count_nonzero(analysis_mask)
        if mask == SIGN_MASK:
            self.pixel_pub.publish(pixel_count)

        if pixel_count < min_count:
            if min_count == MIN_SIGN_COUNT and mask == SIGN_MASK:
                print("Only " + str(pixel_count))
                # cv2.imshow(str(pixel_count) + " gray pixels, too little?", analysis_mask)
                cv2.waitKey(3)
            return None

        contours, _ = cv2.findContours(analysis_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("Landed here somehow?")
            return None

        sign_cnt = max(contours, key=cv2.contourArea)

        req_acc = 0.05 * cv2.arcLength(sign_cnt, True) # tells approxPolyDP maximal distance allowed between contour and simplified polygon.
        approx = cv2.approxPolyDP(sign_cnt, req_acc, True)
        area = cv2.contourArea(approx)
        
        if mask == SIGN_MASK:            
            self.area_pub.publish(int(area))

        # If the shape of the largest contour isn't a rectangle, it's likely some of the border is off the screen, so we shouldn't trust that the text will be fully included.
        if len(approx) != 4  or area < min_area:
            if area < min_area:
                print(str(area) + " is the area seen.")
                # cv2.imshow(str(area) + " culprit area!", analysis_mask)
                cv2.waitKey(3)
            else:
                print(len(approx))
                print("Doesn't see it as a rectangle, I suppose!")
                # cv2.imshow("Not a rectangle", frame)
                cv2.waitKey(3)
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
        @brief Scans for sign across two cameras according to mask and external processing method result.
        @param mask the mask to apply to locate rectangular sign.
        @param min_count mask dependent for passing into extract_process_plate
        @param min_area mask dependent for passing into extract_process_plate
        @return None if no sign is found, cvimage of sign otherwise.
        """
        images = [self.left_image, self.right_image]

        for image in images:
            poss_plate = self.extract_process_plate(image, mask, min_count, min_area)
            if poss_plate is not None:
                return poss_plate
        return None

    def find_word(self, sign_img, text_mask, sw_mask_called=0):
        """
        @brief Extracts and sorts letters contained in images.
        @param text_img the binary image to pull letters from.
        @returns modified text_img with rectangles around each detected letter.
        """ 

        text_img = self.apply_mask(sign_img, text_mask)
        img_to_show = np.copy(text_img)
        contours, _ = cv2.findContours(text_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        upper_char_rects = []
        lower_char_rects = []
        
        def store_in_right_row(x,y,w,h):
            if y >= int(img_height / 2):
                lower_char_rects.append((x,y,w,h))
            else:
                upper_char_rects.append((x,y,w,h))

        img_height, img_width = img_to_show.shape

        w = int(img_width * CHAR_WIDTH_PROP)
        h = int(img_height * CHAR_HEIGHT_PROP)
        lastCharX = 0

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CNT_SIZE:
                continue

            x, y, _w, _h = cv2.boundingRect(cnt)
            if _w > int(1.5 * w):
                if sw_mask_called != 2:
                    text_mask[1] += 5
                    text_mask[3] += 5
                    print("Recursively calling with a stronger mask to prevent jumbling")
                    return self.find_word(sign_img, text_mask, sw_mask_called=1)
                
                chars_jumb = math.ceil(_w / w)
                avg_width = int(_w / chars_jumb)

                for char_to_parse in range(chars_jumb):
                    cv2.rectangle(img_to_show, (x + char_to_parse * avg_width, y), (x + (char_to_parse + 1) * avg_width, y + _h), (255, 255, 0), 1)
                    store_in_right_row(x + char_to_parse * avg_width, y, avg_width, _h) 
            elif _w < int(0.4 * w) or _h < int(0.6 * h):
                text_mask[1] -= 5
                text_mask[3] -= 5
                print("Recursively calling with a weaker mask to prevent fraying!")
                return self.find_word(sign_img, text_mask, sw_mask_called=2)
            else:
                cv2.rectangle(img_to_show, (x, y), (x + _w, y + _h), (255, 255, 0), 1)
                store_in_right_row(x, y, _w, _h)

        upper_char_rects.sort(key=lambda r: r[0])
        lower_char_rects.sort(key=lambda r: r[0])

        if len(upper_char_rects) == 0 or len(lower_char_rects) == 0:
            cv2.imshow("Too jumbled?", img_to_show)
            cv2.waitKey(3)
        clue_pred = self.cnn_proc(text_img, upper_char_rects, w, "")

        if clue_pred in CLUE_TOPICS.keys():
            self.curr_plate = CLUE_TOPICS[clue_pred]
        else:
            self.curr_plate = CLUE_TOPICS[self.closest_clue(clue_pred)]

        value_pred = self.cnn_proc(text_img, lower_char_rects, w, self.clues[self.curr_plate-1][1], True)

        rlmsg = TEAMID + ',' + PASSWORD + ',' + str(self.curr_plate) + ',' + value_pred
        message = clue_pred + ', ' + value_pred
        self.time_pub.publish(rlmsg)
        print(message)

        return img_to_show 
    
    def cnn_proc(self, text_img, char_rects, avg_w, actual_word, upload=False):
        input_imgs = []
        last_x = char_rects[0][0]
        space_indices = []
        actual_word = actual_word.replace(" ", "")

        for char_idx in range(len(char_rects)):
            x,y,w,h = char_rects[char_idx]
            if x - last_x > int(1.5 * avg_w):
                space_indices.append(char_idx)
            last_x = x
            char_img = text_img[y:y+h, x:x+w]
            cnn_input = self.pad32(char_img).astype(np.float32) / 255.0
            cnn_input = np.expand_dims(cnn_input, axis=-1)
            input_imgs.append(cnn_input)
            
            if upload and char_idx < len(actual_word):
                char_name = actual_word + str(char_idx) + actual_word[char_idx] + ".png"
                full_path = os.path.join(OUTPUT_PATH, char_name)
                cv2.imwrite(full_path, self.pad32(char_img), [cv2.IMWRITE_PNG_BILEVEL, 1])

        return self.predict_word(input_imgs, space_indices)

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

    def predict_word(self, images, space_indices):
        chars = []

        for img_idx in range(len(images)):
            if img_idx in space_indices:
                chars.append(' ')
            input_tensor = np.expand_dims(images[img_idx], axis=0)
            self.interpreter.set_tensor(self.input_index, input_tensor)
            self.interpreter.invoke()
            output = self.interpreter.get_tensor(self.output_index)
            predicted_char = POSS_CHARS[np.argmax(output[0])]
            chars.append(predicted_char)

        pred_string = ''.join(chars)

        return pred_string

    def get_clues(self):
        clues = []

        with open(CSV_PATH, 'r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            for row in reader:
                if row:
                    clues.append(tuple(row))
        return clues

    def closest_clue(self, read_clue):
        def hamming_dist(s1, s2):
            if len(s1) != len(s2):
                return max(len(s1), len(s2))
            return sum(char1 != char2 for char1, char2 in zip(s1,s2))
        def dist_read_clue(s):
            return hamming_dist(s, read_clue)
        return min(CLUE_TOPICS.keys(), key=dist_read_clue)

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

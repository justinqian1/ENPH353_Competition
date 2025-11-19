#!/usr/bin/env python3

from __future__ import print_function
from typing import Tuple
import rospy
import random
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
import numpy as np
from scipy.spatial.transform import Rotation as R

# POSITIONS
# [x, y, z, ox, oy, oz, w]
POS_PLATE_1=[5.45,2.08,0.04,0,0,-0.92]
POS_PLATE_2=[5.42,-0.9,0.04,0,0,-1.9]
POS_PLATE_3=[4.59,-1.89,0.04,0,0,-3.12]
POS_PLATE_4=[0.56,-1.03,0.04,0,0,1.083]
POS_PLATE_5=[0.613,2.02,0.04,0,0,-1.144]
POS_PLATE_6=[-2.83,1.346,0.04,0,0,2.592]
POS_PLATE_7=[-4.22,-1.68,0.04,0,0,-0.644]
POS_PLATE_8=[-1.63,-1.33,1.86,0.01,0.0366,0.203]
POS_PLATES=[
        POS_PLATE_1, POS_PLATE_2, POS_PLATE_3, POS_PLATE_4, 
        POS_PLATE_5, POS_PLATE_6, POS_PLATE_7, POS_PLATE_8
        ]
POS_VAR = 0.1
W_VAR = 0.1
# MASKS: [b_minus_g_upper, b_minus_g_lower, b_minus_r_upper,
#         b_minus_r_lower, g_minus_r_upper, g_minus_r_lower]
SIGN_MASK = [110, 90, 110, 90, 256, -256]
GRAY_MASK = [10, -10, 10, -10, 10, -10]
MASK_DICT = {"b_g_upper":0, "b_g_lower":1, "b_r_upper":2,
             "b_r_lower":3, "g_r_upper":4, "g_r_lower":5}

class PlateGenerator:
    """
    @class PlateGenerator
    @brief Class to generate data for plate processing.

    Teleports robot to approximate picture taking positions, and then takes pictures and uploads them to data collection folder for image processing.
    """

    def platemask(self, image, mask):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        b_minus_g=channel_b-channel_g
        b_minus_r=channel_b-channel_r
        g_minus_r=channel_g-channel_r

        feature_mask = (b_minus_g>mask[MASK_DICT["b_g_lower"]]) & (b_minus_g<mask[MASK_DICT["b_g_upper"]]) & (b_minus_r>mask[MASK_DICT["b_r_lower"]]) & (b_minus_r<mask[MASK_DICT["b_r_upper"]]) & (g_minus_r>mask[MASK_DICT["g_r_lower"]]) & (g_minus_r<mask[MASK_DICT["g_r_upper"]]) 

        analysis_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        analysis_mask[feature_mask] = 255

        return cv2.findContours(analysis_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    def extract_process_plate(self, frame, mask, output_shape=(400,200)):
        contours, _ = self.platemask(frame, mask)

        if not contours:
            raise ValueError("Image doesn't contain a sign!")

        sign_cnt = max(contours, key=cv2.contourArea)

        req_acc = 0.02 * cv2.arcLength(sign_cnt, True) # tells approxPolyDP maximal distance allowed between contour and simplified polygon.
        approx = cv2.approxPolyDP(sign_cnt, req_acc, True)

        # If the shape of the largest contour isn't a rectangle, it's likely some of the border is off the screen, so we shouldn't trust that the text will be fully included.
        if len(approx) != 4:
            raise ValueError("Full sign not included in frame!")

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

    def add_var(self, qua_rep):
        pos_noise = random.uniform(-POS_VAR, POS_VAR)
        #w_noise = random.uniform(-W_VAR, W_VAR)
        w_noise = W_VAR
        for pos_ind in range(2):
            qua_rep[pos_ind] += pos_noise
        qua_rep[-1] += w_noise
        return qua_rep

    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "/B1/rrbot/camera1/image_raw")
        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_pub = rospy.Publisher(position_topic, Float32MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.plate_pics = []
        self.cv_image = None
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        return

def main():
    rospy.init_node('plate_generator', anonymous=True)
    ic = PlateGenerator()
    position = [0,0,0,0,0,0,0]
    msg = Float32MultiArray()
    rospy.sleep(0.3)
    for plate in range(len(POS_PLATES)):
        #position = ic.add_var(ic.eul_to_qua(POS_PLATES[plate]))
        position = ic.eul_to_qua(POS_PLATES[plate])
        msg.data = position
        rospy.sleep(0.05)
        ic.pos_pub.publish(msg)
        rospy.sleep(0.1)
        extracted_plate_border = ic.extract_process_plate(ic.cv_image, SIGN_MASK)
        extracted_plate = ic.extract_process_plate(extracted_plate_border, GRAY_MASK)
        ic.plate_pics.append(extracted_plate)
        cv2.imshow('Plate' + str(plate+1), extracted_plate) # CONT here next 16:08
        cv2.waitKey(10)

    while(True):
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass # Handles potential interruptions cleanly

#!/usr/bin/env python3

from __future__ import print_function
from typing import Tuple
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
import numpy as np

class PlateGenerator:
    """
    @class PlateGenerator
    @brief Class to generate data for plate processing.

    Teleports robot to approximate picture taking positions, and then takes pictures and uploads them to data collection folder for image processing.
    """

    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "/B1/rrbot/camera1/image_raw")
        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_pub = rospy.Publisher(position_topic, Float32MultiArray, queue_size=10)


def main():
    rospy.init_node('plate_generator', anonymous=True)
    ic = PlateGenerator()
    position = [0,0,0,0,0,0,0]
    msg = Float32MultiArray()
    msg.data = position
    rospy.sleep(0.1)
    ic.pos_pub.publish(msg)
    rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass # Handles potential interruptions cleanly

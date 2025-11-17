#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
import numpy as np

class PlateGenerator:
    """
    @class PlateGenerator
    @brief Class to generate data for plate processing.

    Teleports robot to approximate picture taking positions, and then takes pictures and uploads them to data collection folder for image processing.
    """
    def spawn_position(self, position):
        msg = ModelState()
        msg.model_name = 'B1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', msg)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "/B1/rrbot/camera1/image_raw")

def main():
    rospy.init_node('plate_generator', anonymous=True)
    ic = PlateGenerator()
    position = (0,0,0,0,0,0,0)
    self.spawn_position(position)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass # Handles potential interruptions cleanly

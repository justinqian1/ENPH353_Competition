#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def find_line(image,threshold=100):
    """
    @brief finds line near the robot
    @param image, input image from camera
    @param threshold, red channel value to determine what counts as the line
    @return list of x and y coordinates of line closest to robot,
            direction of the line further out in case the robot loses it,
            length of the line at the target y coordinate
            returns -1 for all coordinates if line not detected
    """
    '''
    y_coord=790
    line=np.where(image[y_coord,:,2]<threshold)[0]
    length=len(line)
    while length<5 and y_coord>400:
        y_coord-=20
        line=np.where(image[y_coord,:,2]<threshold)[0]
        length=len(line)
    
    far_line=np.where(image[600,:,2]<threshold)[0]
    if len(far_line) == 0:
        dir = -1
    else:
        dir = (far_line[0]+far_line[-1])>800 # left = 0, right = 1

    if length < 5:
        return [-1,-1,-1,-1] # no line
    else:
        if length > 200:
            if np.median(line) > 400:
                x_coord=line[length//3]
            else:
                x_coord=line[2*length//3]
        else:
            x_coord=np.median(line).astype(int)
        return [x_coord,y_coord,dir,length]
    '''
    return None
    
class LineDetector:
    """
    @class LineDetector
    @brief class to detect line
    
    Wrapper class for find_line function that gets images and publishes results.
    """
    def __init__(self):
        image_topic = rospy.get_param('~image_topic', '/B1/rrbot/camera1/image_raw')
        line_location = rospy.get_param('~line_location', '/line_location')
        self.result_pub = rospy.Publisher(line_location, Int32MultiArray, queue_size=1)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.bridge = CvBridge()

    def callback(self,data):
        """
        @brief responds to image inputs
        @param data, the input image
        
        executes the find_line function and publishes to line_location
        """
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        cv2.imwrite("/tmp/frame.png", cv_image)
        cv2.imshow('camera feed',cv_image)
        cv2.waitKey(1)
        #msg = Int32MultiArray()
        #msg.data = find_line(cv_image)
        #self.result_pub.publish(msg)

def main():
    rospy.init_node('line_detector', anonymous=True)
    ic = LineDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
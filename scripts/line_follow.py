#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def find_features(image):
    channel_b=image[:,:,0]
    channel_g=image[:,:,1]
    channel_r=image[:,:,2]
    b_minus_g=channel_b-channel_g
    b_minus_r=channel_b-channel_r
    g_minus_r=channel_g-channel_r

    line1_mask=(channel_b>245) & (channel_g>245) & (channel_r>245)
    line2_mask=(channel_r>185) & (channel_r<220) & (channel_g>185) & (channel_g<220) & (channel_b>130) & (channel_b<170)
    line_mask=line1_mask | line2_mask
    road_mask=(b_minus_g==0) & (b_minus_r==0) & (g_minus_r==0) & (channel_b>=80) & (channel_b<=90)
    #sign_mask=(b_minus_g>90) & (b_minus_g<110) & (b_minus_r>90) & (b_minus_r<110)
    red_line_mask=(channel_r>245) & (channel_b<10) & (channel_g<10)
    pink_line_mask=(channel_r>245) & (channel_b>245) & (channel_g<10)
    ped_mask=(b_minus_g>3) & (b_minus_g<13) & (g_minus_r>5) & (g_minus_r<15)
    ped_mask[:150,:]=False # no ped at top of scrn
    ped_mask[:,:50]=False # no ped on sides
    ped_mask[:,-50:]=False
    truck_mask=(channel_b>120) & (channel_b<240) & (b_minus_g==0) & (b_minus_r==0) & (g_minus_r==0)

    features_mask=np.zeros(image.shape,dtype=np.uint8)
    features_mask[210:,140:145,2]=80 # driving box 1
    features_mask[210:,175:180,2]=80
    features_mask[210:215,140:180,2]=80
    features_mask[240:,70:75,2]=120 # driving box 2
    features_mask[240:,245:250,2]=120
    features_mask[240:245,70:250,2]=120
    features_mask[line_mask]=255
    features_mask[road_mask]=80
    #features_mask[:,:,0][sign_mask]=255
    features_mask[:,:,2][red_line_mask]=255
    features_mask[:,:,0][pink_line_mask]=255
    features_mask[:,:,2][pink_line_mask]=255
    features_mask[ped_mask]=100
    features_mask[truck_mask]=200
    #cv2.imwrite('/tmp/frame.png',image)
    cv2.imshow('camera feed', image)
    cv2.imshow('line',features_mask)
    cv2.waitKey(1)

    # decide if line is directly in front of robot (need to turn)
    line_in_front=np.sum(line_mask[210:240,140:180])+np.sum(line_mask[240:,70:250]) 
    line_left=np.where(line_mask[:,0])[0]
    line_left_coord=line_left[-3] if len(line_left)>=3 else -1 # use -3 to avoid outliers/noise
    line_mid=np.where(line_mask[:,160])[0]
    line_mid_coord=line_mid[-1] if len(line_mid)>=2 else -1 # thresholding based on length 2 but choosing last px for middle
    line_right=np.where(line_mask[:,-1])[0]
    line_right_coord=line_right[-3] if len(line_right)>=3 else -1
    road_left=np.where(road_mask[:,0])[0]
    road_left_coord=road_left[3] if len(road_left)>=3 else -1 # use -3 to avoid outliers/noise
    red_line_sz=np.sum(red_line_mask[:])
    pink_line_sz=np.sum(pink_line_mask[:])
    ped_sz=np.sum(ped_mask[:])
    truck_sz=np.sum(truck_mask[:])
    #sign_size=np.sum(sign_mask[:])

    return [line_in_front,line_left_coord,line_mid_coord,line_right_coord,red_line_sz,pink_line_sz,ped_sz,truck_sz,road_left_coord]
    
class LineDetector:
    """
    @class LineDetector
    @brief class to detect line
    
    Wrapper class for find_line function that gets images and publishes results.
    """
    def __init__(self):
        image_topic = rospy.get_param('~image_topic', '/B1/rrbot/camera1/image_raw')
        drive_info = rospy.get_param('~drive_info', '/drive_info')
        self.result_pub = rospy.Publisher(drive_info, Int32MultiArray, queue_size=1,latch=False)
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
        find_features(cv_image)
        msg = Int32MultiArray()
        msg.data = find_features(cv_image)
        self.result_pub.publish(msg)

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

#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy import ndimage

start_timer = String('team,pass,0,whatever')
stop_timer = String('team,pass,-1,whatever')

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
    sign_mask=(b_minus_g>90) & (b_minus_g<110) & (b_minus_r>90) & (b_minus_r<110)
    red_line_mask=(channel_r>245) & (channel_b<10) & (channel_g<10)
    pink_line_mask=(channel_r>245) & (channel_b>245) & (channel_g<10)
    ped_mask=(b_minus_g>3) & (b_minus_g<13) & (g_minus_r>5) & (g_minus_r<15)
    ped_mask[:380,:]=False
    truck_mask=(channel_b>120) & (channel_b<240) & (b_minus_g==0) & (b_minus_r==0) & (g_minus_r==0)

    features_mask=np.zeros(image.shape,dtype=np.uint8)
    features_mask[520:,350:355,2]=90 # driving box 1
    features_mask[520:,445:450,2]=90
    features_mask[520:525,350:450,2]=90
    features_mask[600:,180:185,2]=120 # driving box 2
    features_mask[600:,615:620,2]=120
    features_mask[600:605,180:620,2]=120
    features_mask[line_mask]=255
    features_mask[:,:,0][sign_mask]=255
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
    line_in_front=np.sum(line_mask[520:,350:450])+np.sum(line_mask[600:,180:620]) 
    line_left=np.where(line_mask[:,0])[0]
    line_left_coord=line_left[-5] if len(line_left)>=5 else -1 # use -5 to avoid outliers/noise
    line_right=np.where(line_mask[:,-1])[0]
    line_right_coord=line_right[-5] if len(line_right)>=5 else -1
    red_line_sz=np.sum(red_line_mask[:])
    pink_line_sz=np.sum(pink_line_mask[:])
    ped_sz=np.sum(ped_mask[:])
    truck_sz=np.sum(truck_mask[:])
    
    
    sign_size=np.sum(sign_mask[:])
    '''
    if sign_size>2000: # enough of the sign is visible    
        labeled, _ = ndimage.label(sign_mask)
        largest=np.bincount(labeled.ravel())[1:].max()
        print(largest)
        #if largest>4000: # one sign; now check if all sides of sign are visible
        # TODO
    '''

    return [line_in_front,line_left_coord,line_right_coord,sign_size,red_line_sz,pink_line_sz,ped_sz,truck_sz]
    
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
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
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
    rospy.sleep(1)
    ic.time_pub.publish(start_timer)
    # I'd like to implement so that we only send stop message when we stop moving even if we're sending some commands, which means we fell somewhere; I'm assuming this will happen haha.
    rospy.sleep(3)
    ic.time_pub.publish(stop_timer)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

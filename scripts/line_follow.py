#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
    
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
        self.loc_sub = rospy.Subscriber('/B1/loc', String,self.state_callback,queue_size=1)
        self.bridge = CvBridge()
        self.section='1'

    def callback(self,data):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        msg = Int32MultiArray()
        if self.section=='1':
            msg.data = self.features_part1(cv_image)
        elif self.section=='2':
            msg.data = self.features_part2(cv_image)
        elif self.section=='3':
            msg.data = self.features_part3(cv_image)
        self.result_pub.publish(msg)

    def state_callback(self,data):
        self.section=data.data
        print(f'Entering section {self.section}!')
    
    def features_part1(self,image):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        b_minus_g=channel_b-channel_g
        b_minus_r=channel_b-channel_r
        g_minus_r=channel_g-channel_r

        line_mask=(channel_b>245) & (channel_g>245) & (channel_r>245)
        road_mask=(b_minus_g==0) & (b_minus_r==0) & (g_minus_r==0) & (channel_b>=80) & (channel_b<=90)
        road_mask[:180,:]=False # don't consider road at top
        red_line_mask=(channel_r>245) & (channel_b<10) & (channel_g<10)
        pink_line_mask=(channel_r>245) & (channel_b>245) & (channel_g<10)
        ped_mask=(b_minus_g>3) & (b_minus_g<13) & (g_minus_r>5) & (g_minus_r<15)
        ped_mask[:150,:]=False # no ped at top of scrn
        ped_mask[:,:50]=False # no ped on sides
        ped_mask[:,-50:]=False
        truck_mask=(channel_b>110) & (channel_b<195) & (b_minus_g==0) & (b_minus_r==0) & (g_minus_r==0)

        features_mask=np.zeros(image.shape,dtype=np.uint8)
        features_mask[210:,140:145,2]=80 # driving box 1
        features_mask[210:,175:180,2]=80
        features_mask[210:215,140:180,2]=80
        features_mask[240:,70:75,2]=120 # driving box 2
        features_mask[240:,245:250,2]=120
        features_mask[240:245,70:250,2]=120
        features_mask[line_mask]=255
        features_mask[road_mask]=80
        features_mask[:,:,2][red_line_mask]=255
        features_mask[:,:,0][pink_line_mask]=255
        features_mask[:,:,2][pink_line_mask]=255
        features_mask[ped_mask]=100
        features_mask[truck_mask]=150
        #cv2.imwrite('/tmp/frame.png',image)
        cv2.imshow('camera feed', image)
        cv2.imshow('line',features_mask)
        cv2.waitKey(1)

        # decide if line is directly in front of robot (need to turn)
        line_in_front=np.sum(line_mask[210:240,140:180])+np.sum(line_mask[240:,70:250]) 
        line_left=np.where(line_mask[:,0])[0]
        line_left_coord=line_left[-3] if len(line_left)>=3 else -1 # use -3 to avoid outliers/noise
        line_mid=np.where(line_mask[:,160])[0]
        line_mid_coord=line_mid[-1] if len(line_mid)>=1 else -1 # thresholding based on length 2 but choosing last px for middle
        line_right=np.where(line_mask[:,-1])[0]
        line_right_coord=line_right[-3] if len(line_right)>=3 else -1
        road_left=np.where(road_mask[:,0])[0]
        road_left_coord=road_left[2] if len(road_left)>=3 else -1 # use -3 to avoid outliers/noise
        road_sz=np.sum(road_mask[:])
        red_line_sz=np.sum(red_line_mask[:])
        pink_line_sz=np.sum(pink_line_mask[:])
        ped_sz=np.sum(ped_mask[:])
        truck_sz=np.sum(truck_mask[:])

        return [line_in_front,line_left_coord,line_mid_coord,line_right_coord,red_line_sz,pink_line_sz,ped_sz,truck_sz,road_sz,road_left_coord]

    def features_part2(self,image):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        kernel = np.ones((3,3), np.uint8)

        line_mask=(channel_b>130) & (channel_b<170) & (channel_g>175) & (channel_r < 212)
        line_mask = line_mask.astype(np.uint8) * 255
        blurred_line1 = cv2.GaussianBlur(line_mask, (19,19), 0)
        _, blurred_line1 = cv2.threshold(blurred_line1, 50, 255, cv2.THRESH_BINARY)
        blurred_line1 = cv2.morphologyEx(blurred_line1, cv2.MORPH_OPEN, kernel)
        line_mask=blurred_line1>0  
        line_mask[260:,:]=False  
        
        features_mask=np.zeros(image.shape,dtype=np.uint8)

        # water
        water_mask= (channel_b>120) & (channel_b<200) & (channel_b+10>channel_g) & (channel_b+10>channel_r)
        water_mask[:150,:]=False
        features_mask[218:223,:,1]=200

        # pink ln
        pink_line_mask=(channel_r>180) & (channel_b>120) & (channel_g<130)

        # DRIVING BOX
        features_mask[185:190,65:-65,2]=100 # main driving box (200:230, 80:-80)
        features_mask[220:225,65:-65,2]=100 
        features_mask[190:220,65:70,2]=100 
        features_mask[190:220,-70:-65,2]=100 
        
        features_mask[line_mask]=255
        features_mask[:,:,0][water_mask]=180
        features_mask[:,:,0][pink_line_mask]=255
        features_mask[:,:,2][pink_line_mask]=255

        line_sz=np.sum(line_mask[190:220,70:-70])
        water_sz=np.sum(water_mask[:])
        line_left=np.where(line_mask[:,0])[0]
        line_left_coord=line_left[0] if len(line_left)>=4 else -1 # require certain line height to reduce noise impact; use top instead of bottom of line for same reason
        line_right=np.where(line_mask[:,-1])[0]
        line_right_coord=line_right[0] if len(line_right)>=4 else -1
        line_left_amt=np.sum(line_mask[:,:10])
        line_right_amt=np.sum(line_mask[:,-10:])
        land_loc=np.where(~water_mask[220,:])[0]
        land_left=land_loc[2] # avoid outliers
        land_right=320-land_loc[-3] # dist from right edge
        pink_ln_sz=np.sum(pink_line_mask[:])
        pink_ln_row=np.any(pink_line_mask,axis=0)
        pink_ln_idx=np.flatnonzero(pink_ln_row)
        pink_ln_mid=(pink_ln_idx[-1]+pink_ln_idx[0])//2 if len(pink_ln_idx)>1 else -1

        #cv2.imwrite('/tmp/frame2.png',image)
        #cv2.imshow('camera feed', image)
        cv2.imshow('line',features_mask)
        cv2.waitKey(1)
        return [line_sz,line_left_coord,line_right_coord,line_left_amt,line_right_amt,water_sz,land_left,land_right,pink_ln_sz,pink_ln_mid]
    
    def features_part3(self,image):
        channel_b=image[:,:,0]
        channel_g=image[:,:,1]
        channel_r=image[:,:,2]
        gs=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        yoda=(gs>36) & (gs<52) & (channel_r>channel_b) & (channel_b>=channel_g) & (channel_b<channel_g+3)
        car=(gs>35)&(gs<45)&(channel_r==channel_b)&(channel_r==channel_g)
        pink_line_mask=(channel_r>250) & (channel_b>250) & (channel_g<10)

        features_mask=np.zeros(image.shape,dtype=np.uint8)
        features_mask[:,:,1][yoda]=255
        features_mask[car]=60
        features_mask[:,:,0][pink_line_mask]=255
        features_mask[:,:,2][pink_line_mask]=255

        yoda_sz=np.sum(yoda[:])
        car_sz=np.sum(car[:])
        yoda_row=np.any(yoda,axis=0)
        yoda_idx=np.flatnonzero(yoda_row)
        yoda_mid=(yoda_idx[-3]+yoda_idx[2])//2 if len(yoda_idx)>5 else -1

        #cv2.imwrite('/tmp/frame.png',image)
        cv2.imshow('camera feed', image)
        cv2.imshow('features',features_mask)
        cv2.waitKey(1)
        return [yoda_sz,yoda_mid,car_sz]

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

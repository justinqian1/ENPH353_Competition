#!/usr/bin/env python3
import rospy
import cv2
import os
import sys
import termios
import tty
import threading
import csv
import tensorflow as tf
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

SAVE_PATH = "/tmp/training_data"

KEY2STATE = {
    'i': "FWD",
    'u': "FWD_LEFT",
    'j': "LEFT",
    'l': "RIGHT",
    'k': "STOP"
}
ID2STATE = {
    0: "FWD",
    1: "FWD_LEFT",
    2: "LEFT",
    3: "RIGHT",
    4: "STOP"
}
MODEL_PATH="/home/fizzer/ros_ws/src/team5_code/models/hill_cnn_jq_v10.tflite"
BASE_SPEED=2.0
stop_timer = String('team,pass,-1,whatever')

class DataCollector:
    def __init__(self):
        rospy.init_node("data_collector")
        self.active=False

        self.bridge = CvBridge()
        self.state = "FWD"
        os.makedirs(SAVE_PATH, exist_ok=True)
        self.csv_path = os.path.join(SAVE_PATH, "data.csv")
        new_file = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if new_file:
            self.csv_writer.writerow(["image", "action"])
        self.collecting_data=False
        self.auto_drive=True
        self.interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()

        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_index = self.interpreter.get_output_details()[0]["index"]

        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.last_save = 0
        self.save_interval = 0.3

        self.image_sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.callback, queue_size=1)
        self.loc_sub = rospy.Subscriber('/B1/loc', String,self.loc_callback,queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.stop_timer = rospy.Timer(rospy.Duration(30.0), self.stop_timer_callback, oneshot=True)


        rospy.loginfo("Data collector started.")

    def stop_timer_callback(self, event):
        self.drive_pub.publish(Twist())    
        self.time_pub.publish(stop_timer)  
        rospy.loginfo("Time limit reached. Stopping driver.")
        rospy.signal_shutdown("Timed shutdown")

    def keyboard_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while not rospy.is_shutdown():
                key = sys.stdin.read(1)
                if key in KEY2STATE:
                    self.state = KEY2STATE[key]
                    rospy.loginfo(f"State set to: {self.state}")
                elif key=='a': # toggle data collection
                    print("Toggling data collection")
                    self.collecting_data=not self.collecting_data
                    self.auto_drive=not self.auto_drive
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def callback(self, msg):
        if not self.active:
            return
        try:
           cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        blue = cv_image[:,:,0]
        cropped = blue[140:300, :]
        
        cv2.imshow('input',cropped)
        cv2.waitKey(1)

        if self.auto_drive:
            input = cropped.astype(np.float32) / 255.0
            input = np.expand_dims(input, axis=(-1,0))
            self.interpreter.set_tensor(self.input_index, input)
            self.interpreter.invoke()
            output = self.interpreter.get_tensor(self.output_index)
            self.state=ID2STATE[output.argmax()]
            print(self.state)

        now = rospy.Time.now().to_sec()
        if self.collecting_data and now - self.last_save > self.save_interval:
            self.last_save = now

            ts = int(msg.header.stamp.to_sec() * 1000) 
            filename = f"{ts}.png"
            img_path = os.path.join(SAVE_PATH, filename)
            cv2.imwrite(img_path, cropped)

            self.csv_writer.writerow([filename, self.state])
            self.csv_file.flush()
            rospy.loginfo_throttle(2.0,
                f"Saved frame {ts}, action={self.state}"
            )

        twist = Twist()
        if self.state == 'FWD':
            twist.linear.x = BASE_SPEED
            twist.angular.z = 0
        elif self.state == 'LEFT':
            twist.linear.x = 0
            twist.angular.z = BASE_SPEED*2
        elif self.state == 'FWD_LEFT':
            twist.linear.x = BASE_SPEED
            twist.angular.z = BASE_SPEED*2
        elif self.state == 'RIGHT':
            twist.linear.x = 0
            twist.angular.z = -BASE_SPEED*2
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        self.drive_pub.publish(twist)
    
    def loc_callback(self,msg):
        if msg.data == "4":
            print("Hill climbing CNN active!")
            self.active = True

    def __del__(self):
        try:
            self.csv_file.close()
        except:
            pass
if __name__ == "__main__":
    DataCollector()
    rospy.spin()

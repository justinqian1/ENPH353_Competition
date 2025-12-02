#! /usr/bin/env python3
import rospy
import cv2
import sys
import numpy as np
import os
import random
from pynput import mouse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtCore import Qt, pyqtSignal, QEvent
from python_qt_binding import loadUi

SCRIPT_DIR = os.path.dirname(__file__) # Directory of the current script
UI_PATH = os.path.join(SCRIPT_DIR, 'KappaDriver_app.ui')
START_ROW = 24
GUI_SCALE_FACT = 7
LINEAR_X = 2.0
KAPPA_SLIDER_FACT = 0.015
LIN_DIAL_FACT = 0.1
RESPAWN_COORD = [-3.74, -2.31, 0.04, 0, 0, 0]
OUTPUT_PATH = "/home/fizzer/il_train/labelled_data"
        
class KappaDrive(QtWidgets.QMainWindow):
    '''
    @class KappaDrive
    @brief Class to send drive commands according to desired kappa (radius of curvature inverse) from GUI user input, and when desired to store command camera feed pairs as training data.
    '''
    def __init__(self):
        self.max_lin_x = 3
        self.linear_x = 0
        self.kappa = 0.0

        self.labelled_data = []
        self.recording = False

        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_pub = rospy.Publisher(position_topic, Float32MultiArray, queue_size=10)
        image_topic = rospy.get_param('~image_topic', '/B1/rrbot/camera1/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        time_topic = rospy.get_param('~clock_topic', '/clock')
        self.clock_sub = rospy.Subscriber(time_topic, Clock, self.time_update, queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.pub_vel)
        self.image = None
        self.label_prefix = ""
        self.bridge = CvBridge()

        super(KappaDrive, self).__init__()
        loadUi(UI_PATH, self)

        self.kappa_slider.setRange(-100, 100)
        self.kappa_slider.sliderPressed.connect(self.driving)
        self.kappa_slider.sliderReleased.connect(self.stop)
        self.kappa_slider.valueChanged.connect(self.adjKappa)

        record_shortcut_key = QtGui.QKeySequence(Qt.Key_Space)
        respawn_shortcut_key = QtGui.QKeySequence(Qt.Key_R)
        upload_shortcut_key = QtGui.QKeySequence(Qt.Key_S)
        delete_shortcut_key = QtGui.QKeySequence(Qt.Key_D)

        self.record_shortcut = QShortcut(record_shortcut_key, self.kappa_slider)
        self.record_shortcut.activated.connect(self.toggle_recording)

        self.linear_speed_dial.setRange(0, 100)
        self.linear_speed_dial.valueChanged.connect(self.set_lin_x)

        self.scratch_button.clicked.connect(self.scratch)

        self.delete_shortcut = QShortcut(delete_shortcut_key, self.scratch_button)
        self.delete_shortcut.activated.connect(self.scratch)

        self.respawn_button.clicked.connect(self.respawn)

        self.respawn_shortcut = QShortcut(respawn_shortcut_key, self.respawn_button)
        self.respawn_shortcut.activated.connect(self.respawn)

        self.upload_shortcut = QShortcut(upload_shortcut_key, self.respawn_button)
        self.upload_shortcut.activated.connect(self.upload_data)


    def driving(self):
        self.linear_x = self.max_lin_x

    def stop(self):
        self.linear_x = 0
        self.kappa_slider.setSliderPosition(0)

    def adjKappa(self, sliderVal):
        self.kappa = sliderVal * KAPPA_SLIDER_FACT

    def pub_vel(self, event):
        twist = Twist()
        if self.linear_x != 0:
            twist.linear.x = self.linear_x
            twist.angular.z = -1.0 * self.kappa * self.linear_x
            self.drive_pub.publish(twist)

            # Adding to labelled collection.
            prefix = self.label_prefix + "_"
            if self.kappa > 0.0:
                prefix += "1_"
            else:
                prefix += "0_"
            kappa_str = f"{self.kappa:.3f}"
            label = prefix + kappa_str
            data_point = (label, self.image)
            self.labelled_data += data_point
        else:
            self.drive_pub.publish(twist)

    def time_update(self, msg):
        secs = msg.clock.secs
        nsecs = msg.clock.nsecs
        self.label_prefix = str(secs) + str(nsecs)[:3]

    def set_lin_x(self, dialVal):
        self.max_lin_x = dialVal * LIN_DIAL_FACT

    def respawn(self):
        msg = Float32MultiArray()
        msg.data = RESPAWN_COORD.copy()
        y_noise = random.uniform(0, 0.05)
        w_noise = random.uniform(0, 0.15)
        msg.data[1] += y_noise
        msg.data[-1] -= w_noise
        self.pos_pub.publish(msg)

    def toggle_recording(self):
        if self.recording:
            self.recording = False
            print("Not recording")
        else:
            self.recording = True
            print("Recording")

    def upload_data(self):
        print("Uploading data!")

        for labelled in self.labelled_data:
            file_name = labelled[0] + ".png"
            full_path = os.path.join(OUTPUT_PATH, file_name)
            cv2.imwrite(full_path, labelled[1])

        print("Uploaded " + str(len(self.labelled_data)) + " points!")

    def scratch(self):
        print("Deleting data!")
        self.labelled_data.clear()

    def callback(self,data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image = self.image[START_ROW:, :]
        pixmap = self.convert_cv_to_pixmap(self.image)
        scaled_pixmap = pixmap.scaled(self.image.shape[1] * GUI_SCALE_FACT, self.image.shape[0] * GUI_SCALE_FACT, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.display_live_image.setPixmap(scaled_pixmap)

    # Source: stackoverflow.com/questions/34232632/
    def convert_cv_to_pixmap(self, cv_img):
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytesPerLine = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)

def main():
    rospy.init_node('kappa_driver')
    app = QtWidgets.QApplication(sys.argv)
    kd = KappaDrive()
    kd.show()
    rospy.sleep(0.5)
    sys.exit(app.exec_())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

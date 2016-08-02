#!/usr/bin/env python
import cv2
import rospy
import time
from std_msgs.msg import *
from sensor_msgs.msg import *
from racecar_34.msg import BlobDetections
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt

class Calibrate:
    def __init__(self):
        self.node_name = "calibrate_node"
        self.header = std_msgs.msg.Header()
        self.last_time_saved = rospy.get_time()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",Image, self.cbImage, queue_size=1)
        self.joy_control = rospy.Subscriber("/vesc/joy", Joy, take_picture)
        rospy.loginfo("Initialized")
        print("Show red paper and press joy button")
        self.color_index = 0
        self.colors = ["red", "yellow", "green", "blue", "pink"]
        self.curr_image = Image()

    def cbImage(self, msg):
        self.curr_image = msg
        
    def take_picture(self, msg):
        if msg.buttons[0] == 1:
            image_cv = self.bridge.imgmsg_to_cv2(self.curr_image)
            #save image into another folder, label with date
            cv2.imwrite("color_"+self.colors[self.color_index]+".jpg", self.curr_image)
            self.color_index += 1

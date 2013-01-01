#!/usr/bin/env python
import cv2
import rospy
import time
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt
import os, sys

class Calibrate:
    def __init__(self):
        self.node_name = "calibrate_node"
        self.header = std_msgs.msg.Header()
        self.last_time_saved = rospy.get_time()
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",Image, self.cbImage, queue_size=1)
        self.joy_control = rospy.Subscriber("/vesc/joy", Joy, self.take_picture)
        rospy.loginfo("Initialized")
        print("Show red paper and press joy button")
        self.color_index = 0
        self.colors = ["red", "yellow", "green", "blue", "pink", "arstasr","Arstasrtars","arstasrtasdarda"]
        self.curr_image = Image()

        #samples
        #self.red_sample = cv2.imread('./in/red.jpg')
        #self.red_hsv = cv2.cvtColor(self.red_sample, cv2.COLOR_BGR2HSV)
        #self.process_image(self.red_hsv)

        #self.yellow_sample = cv2.imread('./in/yellow.jpg')
        #self.yellow_hsv = cv2.cvtColor(self.yellow_sample, cv2.COLOR_BGR2HSV)
        #self.process_image(self.yellow_hsv)

        #self.green_sample = cv2.imread('./in/green.jpg')
        #self.green_hsv = cv2.cvtColor(self.green_sample, cv2.COLOR_BGR2HSV)
        #self.process_image(self.green_hsv)

        #self.blue_sample = cv2.imread('./in/blue.jpg')
        #self.blue_hsv = cv2.cvtColor(self.blue_sample, cv2.COLOR_BGR2HSV)
        #self.process_image(self.blue_hsv)

        #self.pink_sample = cv2.imread('./in/pink.jpg')
        #self.pink_hsv = cv2.cvtColor(self.pink_sample, cv2.COLOR_BGR2HSV)
        #self.process_image(self.pink_hsv)

    def cbImage(self, msg):
        self.curr_image = msg
        #print(msg.data)
        
    def take_picture(self, msg):
        print("callback")
        if msg.buttons[0] == 1:
            image_cv = self.bridge.imgmsg_to_cv2(self.curr_image)
            print("got it")
            os.chdir("/home/racecar/racecar-ws/src/racecar_34/test/pictures")
            cv2.imwrite("color_"+self.colors[self.color_index]+".jpg", image_cv)
            img_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
            print("processing" + self.colors[self.color_index] + "...")
            if self.color_index < 8:
                self.process_image(img_hsv)
                print("put " + self.colors[self.color_index] + " paper and press joystick button")

    def process_image(self, im):
        size = 5
        if self.color_index < 8:
            os.chdir("/home/racecar/racecar-ws/src/racecar_34/test/out")
            sys.stdout = open(self.colors[self.color_index]+".out", 'w')
            for r in range(640-size, 640+size):
                for c in range(360-size, 360+size):
                    print(im[c][r])
            #sys.stdout.close()
            #sys.stdout.open("/dev/stdout", "w")
            self.color_index += 1
            print("calc")
        else:
            print("done")
        return

if __name__ == '__main__':
    rospy.init_node('calibrate_node')
    node = Calibrate()
    rospy.spin()

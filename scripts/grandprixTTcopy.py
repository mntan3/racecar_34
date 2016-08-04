#!/usr/bin/env python
import rospy
import cv2
import math
import threading
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import *
from sensor_msgs.msg import *

class GrandPrixTTMainControl:
    #Other nodes will be controlled from here
    def __init__(self):
        self.node_name = "grand_prix_time_trial_main_control"
        self.header = std_msgs.msg.Header()
        #self.ifSeeingGreen = False
        self.ifSeeingRed = False
        self.counter = 0

        self.K_energy = -0.2

        self.bridge = CvBridge()
        self.thread_lock = threading.Lock()

        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=1)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_cb, queue_size=1)

        #self.pub_image = rospy.Publisher("simple_blob_detections", Image, queue_size=1)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        rospy.loginfo("initialized")

        self.red_lower = np.array([0,150,80])
        self.red_upper = np.array([6,255,255])

        #self.green_lower = np.array([50,100,100])
        #self.green_upper = np.array([70,255,255])

    def laser_cb(self, msg):

        # boost is only for y dimension, left and right steering.
        boost = 0
      
        if self.counter == 0:
            if self.ifSeeingRed:
                print("starting turn")
                boost = self.K_energy / 0.005**2
                self.counter = 1
            else: 
                boost = -self.K_energy / 0.045**2  
        else:
            print("still turning")
            boost = self.K_energy /0.005**2
            self.counter = self.counter + 1
            if self.counter == 12:
                self.counter = 0
            
        x_force_total = -self.K_energy / 0.02**2
        
        y_force_total = boost

        # let's make sure that this is the ranges we want
        for i in range(180, 900):
            angle = math.radians((i - 540)/4)
            dist = msg.ranges[i]

            force = self.K_energy / dist**2

            if 440 < i < 640:
                y_force = 5 * math.sin(angle) * force
            else:
                y_force = math.sin(angle) * force

            x_force = math.cos(angle) * force

            x_force_total = x_force_total + x_force
            y_force_total = y_force_total + y_force

        # if we're feeling bold, we can remove the speed, and set it to be fixed. IDK if that will work for our algorithm though.
        speed = 0.005 * math.sqrt(x_force_total**2 + y_force_total**2) * np.sign(x_force_total)
        # this is the most important part though
        steering_angle = 1.0 * math.atan2(y_force_total, x_force_total) * np.sign(x_force_total)
        if np.mean(msg.ranges[800:1000]) > 0.05 and not self.counter == 0:
            if steering_angle > 0.2:
                steering_angle = 0.2
        print(steering_angle)
        self.pub_drive.publish(AckermannDriveStamped(self.header, AckermannDrive(speed = speed, steering_angle = steering_angle)))
    def image_cb(self, image_msg):
        thread = threading.Thread(target=self.image_callback,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def image_callback(self, image_msg):
        if not self.thread_lock.acquire(False):
            return

        #self.ifSeeingGreen = False
        self.ifSeeingRed = False

        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        image_cv_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(image_cv_hsv, self.red_lower, self.red_upper)
        #green_mask = cv2.inRange(image_cv_hsv, self.green_lower, self.green_upper)

        # if it's too slow, me might want to remove these ones and not make it gray scaled, just to speed up our code

        #red_obj = cv2.bitwise_and(image_cv, image_cv, mask = red_mask)
        #green_obj = cv2.bitwise_and(image_cv, image_cv, mask = green_mask)

        # I'm still unsure of if this would work with the masks inside instead of the objects
        #full_image = [red_obj, green_obj]
        #full_image = [red_mask, green_mask]
        full_image = [red_mask]

        for i in range(0, len(full_image)):
            # maybe we don't even have to make it gray! (maybe we do though)
            #im_gray = cv2.cvtColor(full_image[i], cv2.COLOR_BGR2GRAY)
            contours, hierarchy = cv2.findContours(full_image[i], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            sorted_contours = sorted(contours, key = lambda x: cv2.contourArea(x), reverse=True)
            if len(sorted_contours) > 10:
                sorted_contours = sorted_contours[0:9]
            for contour in sorted_contours:
                if cv2.contourArea(contour) > 5000:
                    if i == 0:
                        #text_color = (0,0,255)
                        self.ifSeeingRed = True
                    #if i == 1:
                        #text = "hUH? grEeN's in the ayre?"
                        #text_color = (0,255,0)
                        #self.ifSeeingGreen = True

                    # Once we know our algorithm works well, we can remove this part of it... for code speed of course.
                    #x,y,w,h = cv2.boundingRect(contour)
                    #cv2.rectangle(image_cv, (x,y), (x+w, y+h), (147,20,255),2)
                    #cv2.putText(image_cv,text,(x,y),4,1,text_color)
                    #cv2.circle(image_cv, (x+(w/2),y+(h/2)), 5, (255,0,0),5)
        #try:
        #    self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        #except CvBridgeError as e:
        #    print(e)
        self.thread_lock.release()
        
if __name__=="__main__":
    rospy.init_node("grand_prix_time_trial_main_control")
    node = GrandPrixTTMainControl()
    rospy.spin()

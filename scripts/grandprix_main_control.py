#!/usr/bin/env python
import rospy
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *
from sensor_msgs.msg import *

class GrandPrixMainControl:
    #Other nodes will be controlled from here
    def __init__(self):
        self.node_name = "grand_prix_main_control"
        self.header = std_msgs.msg.Header()
        self.ifSeeingGreen = False
        self.ifSeeingRed = False

        self.K_energy = -0.12

        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=1)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_cb, queue_size=1)

        self.pub_image = rospy.Publisher("simple_blob_detections", Image, queue_size=1)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        rospy.loginfo("initialized")

        self.red_lower = np.array([-7,125,100])
        self.red_upper = np.array([7,255,255])

        self.green_lower = np.array([50,100,100])
        self.green_upper = np.array([70,255,255])

    def laser_cb(self, msg):
        boost = 0

        if self.ifSeeingGreen:
            boost = -self.K_energy / 0.5**2
        if self.ifSeeingRed:
            boost = self.K_energy / 0.5**2
            
        x_force_total = -self.K_energy / 0.025**2
        y_force_total = boost

        
        for i in range(180, 900):
            angle = math.radians((i - 540)/4)
            dist = msg.ranges[i]

            force = self.K_energy / dist**2

            x_force = math.cos(angle) * force
            y_force = math.sin(angle) * force

            x_force_total = x_force_total + x_force
            y_force_total = y_force_total + y_force

        speed = 0.005 * math.sqrt(x_force_total**2 + y_force_total**2) * np.sign(x_force_total)
        steering_angle = 1.0 * math.atan2(y_force_total, x_force_total) * np.sign(x_force_total)
        print(x_force_total, y_force_total, speed, steering_angle)
        self.drive_pub.publish(AckermannDriveStamped(self.header, AckermannDrive(speed = speed, steering_angle = steering_angle)))
    
    def image_cb(self, image_msg):

        self.ifSeeingGreen = False
        self.ifSeeingRed = False

        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        height,width,depth = image_cv.shape
        image_cv_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(image_cv_hsv, self.red_lower, self.red_upper)
        green_mask = cv2.inRange(image_cv_hsv, self.green_lower, self.green_upper)

        red_obj = cv2.bitwise_and(image_cv, image_cv, mask = red_mask)
        green_obj = cv2.bitwise_and(image_cv, image_cv, mask = green_mask)

        full_image = [red_obj, green_obj]

        for i in range(0, len(full_image)):
            
            im_gray = cv2.cvtColor(full_image[i], cv2.COLOR_BGR2GRAY)
            contours, hierarchy = cv2.findContours(full_image[i], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            sorted_contours = sorted(contours, key = lambda x: cv2.contourArea(x), reverse=True)
            if len(sorted_contours) > 20:
                sorted_contours = sorted_contours[0:19]
            for contour in sorted_contours:
                if cv2.contourArea(contour) > 1000:
                    if i == 0:
                        text = "Ya Saw red yA gink"
                        text_color = (0,0,255)
                        self.ifSeeingRed = True
                    if i == 1:
                        text = "hUH? grEeN's in the ayre?"
                        text_color = (0,255,0)
                        self.ifSeeingGreen = True
                    x,y,w,h = cv2.boundingRect(contour)
                    cv2.rectangle(image_cv, (x,y), (x+w, y+h), (147,20,255),2)
                    cv2.putText(image_cv,text,(x,y),4,1,text_color)
                    cv2.circle(image_cv, (x+(w/2),y+(h/2)), 5, (255,0,0),5)
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        except CVBridgeError as e:
            print(e)
        
if __name__=="__main__":
    rospy.init_node("grand_prix_main_control")
    node = GrandPrixMainControl()
    rospy.spin()

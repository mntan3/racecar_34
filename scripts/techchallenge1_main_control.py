#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *
from sensor_msgs.msg import *
from team34.msg import BlobDetections

class MainControl1:
    def __init__(self):
        self.node_name = "tech_challenge_1_main_control_node"
        self.ableToControl = True

        #self.joy_control = rospy.Subscriber("/vesc/joy", Joy, joy_buttons)
        #self.sub_blob_detection = rospy.Subscriber("blob_detections", BlobDetections, self.blob_cb)
        self.sub_winter = rospy.Subscriber("main_drive", AckermannDriveStamped, self.movement_cb)
        self.sub_safety = rospy.Subscriber("safety", AckermannDriveStamped, self.safety_cb)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

    def movement_cb(self, msg):
        if self.ableToControl:
            self.drive_pub.publish(msg)

    def safety_cb(self, msg):
        rospy.loginfo("Started safety controller")
        self.ableToControl = False
        self.drive_pub.publish(msg)
        self.ableToControl = True
        
    #Optional function to start programs using the joystick
    #def joy_buttons(self, msg):


if __name__=="__main__":
    rospy.init_node("tech_challenge_1_main_control_node")
    node = MainControl1()
    rospy.spin()

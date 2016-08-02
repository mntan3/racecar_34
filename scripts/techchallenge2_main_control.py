#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *

class MainControl:
    #Other nodes will be controlled from here
    def __init__(self):
        self.node_name = "tech_challenge_2_main_control_node"
        self.actionInProgress = 1
        #rospy.Subscriber("/transition", String, self.transition_callback)
        #self.joy_control = rospy.Subscriber("/vesc/joy", Joy, joy_buttons)
        self.sub_blob = rospy.Subscriber("blob_detections", BlobDetections, blob_cb)
        self.sub_visual_servo = rospy.Subscriber("visual_servo", AckermannDriveStamped, first_drive_cb)
        self.sub_turn = rospy.Subscriber("turn_drive", AckermannDriveStamped, second_drive_cb)
        self.sub_wall_follower = rospy.Subscriber("wall_follow", AckermannDriveStamped, third_drive_cb)

        #Add publisher for every node that is to be controlled
        self.wall_pub = rospy.Publisher("/wallfollow", String, queue_size=1)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        rospy.Publisher("/transition", String, queue_size=10)

	#

    #When message recieved from /transition topic, start the appropriate node by publishing a message 
    def transition_callback(self, msg):
        if msg.data == "follow left" or msg.data == "follow right" or msg.data == "stop follow"
            self.wall_pub.publish(msg)
    def first_drive_cb(self, msg):
        if self.actionInProgress == 1:

    def second_drive_cb(self, msg):
        if self.actionInProgress == 2:

    def thrid_drive_cb(self, msg):
        if self.actionInProgress == 3:

    def blob_cb(self, msg):
        

    #Optional function to start programs using the joystick
    #def joy_buttons(self, msg):


if __name__=="__main__":
    rospy.init_node("tech_challenge_2_main_control_node")
    node = MainControl()
    rospy.spin()

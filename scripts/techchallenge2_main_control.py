#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

class MainControl2:
    #Other nodes will be controlled from here
    def __init__(self):
        self.node_name = "tech_challenge_2_main_control_node"

        self.ableToControl = True
        self.actionInProgress = 1
        rospy.loginfo("INITIALIZING STATE TO 1")
        #self.joy_control = rospy.Subscriber("/vesc/joy", Joy, joy_buttons)
<<<<<<< HEAD

        #self.sub_blob = rospy.Subscriber("blob_detections", BlobDetections, blob_cb)
        self.sub_visual_servo = rospy.Subscriber("visual_servo", AckermannDriveStamped, self.first_drive_cb)
        self.sub_turn = rospy.Subscriber("turn_drive", AckermannDriveStamped, self.second_drive_cb)
        self.sub_wall_follower = rospy.Subscriber("wall_follow", AckermannDriveStamped, self.third_drive_cb)
	self.sub_safety = rospy.Subscriber("safety", AckermannDriveStamped, self.safety_cb)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

        self.header = std_msgs.msg.Header()

    #Assume that when a node is finished, it always publishes for the robot to stop. In this case, move to next state
    def first_drive_cb(self, msg):
        if self.actionInProgress == 1 and self.ableToControl:
            self.drive_pub.publish(msg)
            if msg.drive.speed == 0.0:
                self.actionInProgress = 2
                rospy.loginfo("TRANSITIONING TO TURNING")

    def second_drive_cb(self, msg):
        if self.actionInProgress == 2 and self.ableToControl:
            self.drive_pub.publish(msg)
            if msg.drive.speed == 0.0:
                self.actionInProgress = 3
                rospy.loginfo("TRANSITIONING TO WALL FOLLOW")

    def third_drive_cb(self, msg):
        if self.actionInProgress == 3 and self.ableToControl:
            self.drive_pub.publish(msg)

    def safety_cb(self, msg):
        rospy.loginfo("STARTED SAFETY CONTROLLER")
        self.ableToControl = False
        self.drive_pub.publish(msg)
        self.ableToControl = True
    #def blob_cb(self, msg):
        
=======
        self.sub_blob = rospy.Subscriber("blob_detections", BlobDetections, blob_cb)
        self.sub_visual_servo = rospy.Subscriber("visual_servo", AckermannDriveStamped, first_drive_cb)
        self.sub_turn = rospy.Subscriber("turn_drive", AckermannDriveStamped, second_drive_cb)
        self.sub_wall_follower = rospy.Subscriber("wall_follow", AckermannDriveStamped, third_drive_cb)
        self.sub_turn_transition = rospy.Subscriber("/turn", String, turn_cb, queue_size = 2)
        self.sub_wall_transition = rospy.Subscriber("/wallfollow", String, wall_cb, queue_size=1)

        #Add publisher for every node that is to be controlled
        #self.wall_pub = rospy.Publisher("/wallfollow", String, queue_size=1)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        #rospy.Publisher("/transition", String, queue_size=10)

    #When message recieved from /transition topic, start the appropriate node by publishing a message 
    def transition_callback(self, msg):
        if msg.data == "follow left" or msg.data == "follow right" or msg.data == "stop follow"
            self.wall_pub.publish(msg)
    def first_drive_cb(self, msg):
        if self.actionInProgress == 1:
            self.pub_drive.publish(msg)  

    def second_drive_cb(self, msg):
        if self.actionInProgress == 2:
            self.pub_drive.publish(msg)

    def thrid_drive_cb(self, msg):
        if self.actionInProgress == 3:
            self.pub_drive.publish(msg)

    def turn_cb(self, msg):
        self.actionInProgress = 2

    def wall_cb(self, msg):
        self.actionInProgress = 3
>>>>>>> 67134dbbf8fb47955b04836eba9528b72ff9b611

    #Optional function to start programs using the joystick
    #def joy_buttons(self, msg):

if __name__=="__main__":
    rospy.init_node("tech_challenge_2_main_control_node")
    node = MainControl2()
    rospy.spin()

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

        #self.sub_blob = rospy.Subscriber("blob_detections", BlobDetections, self.blob_cb)
        self.sub_visual_servo = rospy.Subscriber("visual_servo", AckermannDriveStamped, self.first_drive_cb)
        self.sub_turn = rospy.Subscriber("turn_drive", AckermannDriveStamped, self.second_drive_cb)
        self.sub_wall_follower = rospy.Subscriber("wall_follow", AckermannDriveStamped, self.third_drive_cb)
        self.sub_turn_transition = rospy.Subscriber("/turn", String, self.turn_cb, queue_size = 2)
        self.sub_wall_transition = rospy.Subscriber("/wall_transition", Int16, self.wall_cb, queue_size=1)
	#self.sub_safety = rospy.Subscriber("safety", AckermannDriveStamped, self.safety_cb)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

        self.header = std_msgs.msg.Header()

    #Assume that when a node is finished, it always publishes for the robot to stop. In this case, move to next state

    #def safety_cb(self, msg):
    #    rospy.loginfo("STARTED SAFETY CONTROLLER")
    #    self.ableToControl = False
    #    self.drive_pub.publish(msg)
    #    self.ableToControl = True

    #When message recieved from /transition topic, start the appropriate node by publishing a message 

    def first_drive_cb(self, msg):
        if self.actionInProgress == 1:
            rospy.loginfo("VISUAL SERVO")
            self.drive_pub.publish(msg)  

    def second_drive_cb(self, msg):
        if self.actionInProgress == 2:
            #rospy.loginfo("TURN")
            self.drive_pub.publish(msg)

    def third_drive_cb(self, msg):
        if self.actionInProgress == 3:
            #rospy.loginfo("WALL FOLLOW")
            self.drive_pub.publish(msg)

    def turn_cb(self, msg):
        if self.actionInProgress == 1 and not msg.data == "":
            self.actionInProgress = 2
            rospy.loginfo("TRANSITIONING TO TURN")

    def wall_cb(self, msg):
        if self.actionInProgress == 2 and not msg.data == "":
            self.actionInProgress = 3
            rospy.loginfo("TRANSITIONING TO WALLFOLLOW")

    #Optional function to start programs using the joystick
    #def joy_buttons(self, msg):

if __name__=="__main__":
    rospy.init_node("tech_challenge_2_main_control_node")
    node = MainControl2()
    rospy.spin()

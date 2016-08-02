#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import *
from std_msgs.msg import *

#Need to somehow communicate to turn node with side to turn to

class Turn:
	def __init__(self):
		self.direction = 0		#0 is left, 1 is right
		rospy.Subscriber("/fork", String, self.setSide)
		self.drive_pub = rospy.Publisher("turn_drive", AckermannDriveStamped, queue_size = 1)
		self.turn_counter = 0
		self.header = std_msgs.msg.Header()

	def setSide(self, msg):
		if msg.data == "green":
			self.direction = 1
		elif msg.data == "red":
			self.direction = 0
		self.turn_timer = rospy.Timer(rospy.Duration(.2), self.turn)
                drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
                self.drive_pub.publish(drive_msg)

	def turn(self, _):
		if self.direction == 0:
			drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=1.5, steering_angle=0.4))
			self.drive_pub.publish(drive_msg)
			#turn left, (1.5, .4)
		else:
			drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=1.0,steering_angle=-0.35))
			self.drive_pub.publish(drive_msg)
			#turn right, (1.0, -0.35)
		self.turn_counter += 1
		if self.turn_counter > 6:
			self.turn_timer.shutdown()
		self.turn_counter = 0
if __name__ == "__main__":
	rospy.init_node("turn_node")
	node = Turn()
	rospy.spin()

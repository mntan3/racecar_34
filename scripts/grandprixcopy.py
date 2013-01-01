#!/usr/bin/env python

import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import *
from ackermann_msgs.msg import *

class GrandPrixMainControl:
    def __init__(self):
        self.node_name = "grand_prix_main_control_node"
        # this value will tell us how much other objects affect us compared to our own forward momentum
        self.K_energy = -1.5

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.header = std_msgs.msg.Header()

        rospy.loginfo("INITIALIZED")

    def laser_callback(self, msg):

        # we may want to tweak our values to be more aggressive against the other cars and try to hug the left wall.
        # most important is figuring out how to take very sharp turns so we can get the edge on other teams.

        x_force_total = -self.K_energy / 0.02**2
        # theoretically, we can make this as big as we want, and it won't shove us into the wall, because as dist to wall approaches zero, its repel value approaches infinity.
        y_force_total = -self.K_energy / 0.04**2

        for i in range(180, 900):
            angle = math.radians((i - 540)/4)
            dist = msg.ranges[i]
            force = self.K_energy * 3 / dist**2
 
            if 500 < i < 580: 
                y_force = 3 * math.sin(angle) * force
            else:
                y_force = math.sin(angle) * force

            x_force = math.cos(angle) * force

            x_force_total = x_force_total + x_force
            y_force_total = y_force_total + y_force

        # we may want to tweak this completely to ensure that our speed is fixed, or at least fix the boost on x.
        speed = 0.005 * math.sqrt(x_force_total**2 + y_force_total**2) * np.sign(x_force_total)
        steering_angle = 1.0 * math.atan2(y_force_total, x_force_total) * np.sign(x_force_total)
        print(x_force_total, y_force_total, speed, steering_angle)
        self.drive_pub.publish(AckermannDriveStamped(self.header, AckermannDrive(speed = speed, steering_angle = steering_angle)))

if __name__ == "__main__":
    rospy.init_node("grand_prix_main_control_node")
    node = GrandPrixMainControl()
    rospy.spin()


#!/usr/bin/env python

import rospy
from mw_sawppy.roverchassis import chassis
from geometry_msgs.msg import Twist
import os, time

class Rover():
    def __init__(self):
        rospy.init_node('Rover', log_level=rospy.DEBUG)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 25))
        r = rospy.Rate(self.rate)

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

        self.myChassis = chassis(self.base_frame)
        self.myChassis.ensureready()

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            self.myChassis.poll()

            r.sleep()

    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Rover Node...")

if __name__ == '__main__':
    myRover = Rover()

#!/usr/bin/env python

import re
import rospy
from std_msgs.msg import Int8, String
import serial
import sys

class LedDisplay():
    def __init__(self):
        self.node_name = "SawppyLedDisplay"
        self.port = None
        self.msg_in = None
        self.msg = None
        self.direction = None
        self.state = 0

        rospy.init_node(self.node_name)

        connectparams = rospy.get_param('~SGVHAK_Rover')['led']['connect']

        self.t_next = rospy.Time.now()

        # Open serial port with parameters
        s = serial.Serial()
        s.baudrate = connectparams['baudrate']
        s.port = connectparams['port']
        s.timeout = connectparams['timeout']
        s.open()

        if s.is_open:
            self.port = s

        r = rospy.Rate(10)

        rospy.Subscriber('/display/msg', String, self.msg_cb, queue_size=1)
        rospy.Subscriber('/display/direction', Int8, self.direction_cb, queue_size=1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

        while not rospy.is_shutdown() and self.port:
            now = rospy.Time.now()
            # print("mw t1: %s" % (str(now)))
            self.port.read(100)

            try:
                if (self.msg_in != None or self.direction != None):
                    if self.direction != None:
                        if self.direction < 0:
                            self.port.write("el\r")
                        elif self.direction > 0:
                            self.port.write("er\r")
                        else:
                            self.port.write("es\r")
                        self.direction = None
                    elif self.msg_in != None:
                        self.msg = self.msg_in
                        self.port.write("t3 %s\r" % self.msg)
                    self.state  = 0
                    self.t_next = now + rospy.Duration(1.0)

                if now > self.t_next:
                    t_delta = 0.0

                    if self.state == 0:
                        if self.msg != None:
                            self.port.write("t3 %s\r" % self.msg)
                            t_delta = 120.0
                        self.state += 1
                    elif self.state == 1:
                        self.port.write("ff\r")
                        t_delta = 10.0
                        self.state += 1
                    elif self.state == 2:
                        self.port.write("fh\r")
                        t_delta = 30.0
                        self.state += 1
                    elif self.state == 3:
                        self.port.write("ewl\r")
                        t_delta = 5.0
                        self.state += 1
                    elif self.state >= 4:
                        self.port.write("fh\r")
                        t_delta = 30.0
                        self.state = 0

                    self.t_next = now + rospy.Duration(t_delta)

            except serial.serialutil.SerialException:
                rospy.loginfo("closed serial")
                self.port = None

            r.sleep()

    def msg_cb(self, msg):
        self.msg_in = msg.data
        self.state = 0

    def direction_cb(self, msg):
        self.direction = msg.data
        self.state = 0

def main(args):
    try:
        LedDisplay()

    except KeyboardInterrupt:
        rospy.loginfo("close")

if __name__ == '__main__':
    main(sys.argv)


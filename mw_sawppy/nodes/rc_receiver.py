#!/usr/bin/env python

import re
import rospy
import serial
import sys
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import Joy

class Receiver():
    def __init__(self):
        self.node_name = "RC_Receiver"
        self.port = None

        rospy.init_node(self.node_name)

        connectparams = rospy.get_param('~SGVHAK_Rover')['rc_receiver']['connect']
        limitparams = rospy.get_param('~SGVHAK_Rover')['rc_receiver']['limits']
        last_time = rospy.Time.now()

        # Open serial port with parameters
        s = serial.Serial()
        s.baudrate = connectparams['baudrate']
        s.port = connectparams['port']
        s.timeout = connectparams['timeout']
        s.open()

        if s.is_open:
            self.port = s

            i = 0                              # Just read some lines
            line = self.port.readline()
            while line and i < 10:
                line = self.port.readline()
                i += 1

        joy_pub = rospy.Publisher("/joy", Joy, queue_size = 1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

        send_stop = 0
        last_rc = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        rc = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        my_joy = Joy()
        my_joy.axes = [ 0.0, 0.0, 0.0, 0.0 ]
        my_joy.buttons = [ 0, 0, 0, 0 ]

        while not rospy.is_shutdown() and self.port:
            try:
                line = self.port.readline()

                m = re.match('I ([-\d.]+) +([-\d.]+) +([-\d.]+) +([-\d.]+) +([-\d.]+) +([-\d.]+) +([-\d.]+) +([-\d.]+)', line)
                if m != None:
                    t = rospy.Time.now()

                    if (t - last_time).to_sec() > 0.05:
                        last_time = t
                        my_joy.header.stamp = t
                        rc = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
                        ok = True

                        for i in range(0, 8):
                            rc[i] = float(m.group(i + 1))

                        rc_use = rc[5] - (limitparams[5]['min'] + limitparams[5]['max']) / 2
                        if rc_use > -450 and rc_use < 450:
                            rospy.logerr("%s: rc_receiver.run() messed up last: %f, %f, %f, %f, %f, %f, %f, %f  now %f, %f, %f, %f, %f, %f, %f, %f" %
                                (t.strftime("%H%M%S.%f"),
                                 last_rc[0], last_rc[1], last_rc[2], last_rc[3], last_rc[4], last_rc[5], last_rc[6], last_rc[7],
                                 rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]))
                            ok = False

                        for i in range(0, 8):
                            last_rc[i] = rc[i]

                        axes = []
                        buttons = []

                        for i in range(0, 4):
                            my_joy.axes[i] = (rc[i] - (limitparams[i]['min'] + limitparams[i]['max']) / 2.0) / (limitparams[i]['max'] - limitparams[i]['min']) * 2.0
                            # print("%d: (%f - %f / 2.0) / %f * 2.0 -> %f" % (i, rc[i], (limitparams[i]['min'] + limitparams[i]['max']), (limitparams[i]['max'] - limitparams[i]['min']), my_joy.axes[i]))

                        for i in range(4, 8):
                            my_joy.buttons[i - 4] = (rc[i] - (limitparams[i]['min'] + limitparams[i]['max']) / 2.0) > 0.0
                            # print("%d: (%f - %f / 2.0) > 0.0 -> %d" % (i, rc[i], (limitparams[i]['min'] + limitparams[i]['max']), my_joy.buttons[i - 4]))
  
                        joy_pub.publish(my_joy)

            except serial.serialutil.SerialException:
                rospy.loginfo("closed serial")
                self.port = None

def main(args):
    try:
        Receiver()

    except KeyboardInterrupt:
        rospy.loginfo("close")

if __name__ == '__main__':
    main(sys.argv)


#!/usr/bin/env python

import datetime
from serial.serialutil import SerialException
import serial
from struct import *
import thread

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy 
import os
import sys
import traceback

class teensy_motors_wrapper:
    """
    Class that implements the rover motor control methods for serial bus
    based on a Teensy 3.5 running a modified ROS Arduino Bridge
    """
    def __init__(self):
        self.port = None
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        # Keep things thread safe
        self.mutex = thread.allocate_lock()

    def check_sp(self):
        """ Raises error if we haven't opened serial port yet. """
        if self.port == None:
            raise ValueError("Teensy serial communication is not available.")

    def connect(self):
        """
        Read serial port connection parameters and open the port.
        """

        connectparams = rospy.get_param('~SGVHAK_Rover')['teensy_motors']['connect']

        # Open serial port with parameters
        s = serial.Serial()
        s.baudrate = connectparams['baudrate']
        s.port = connectparams['port']
        s.timeout = connectparams['timeout']
        s.open()

        if s.is_open:
            self.port = s
            self.timeout = s.timeout
            self.interCharTimeout = self.timeout / 60.

    def close(self):
        """
        Closes down the serial port
        """
        if self.port.is_open:
            self.port.close()
            self.port = None

    def version(self, id):
        """ Identifier string for this motor controller """
        return "Teensy Motors"

    @staticmethod
    def check_id(id):
        """ Verifies servo ID is within range and inverted status is boolean"""
        if not isinstance(id, (tuple,list)):
            raise ValueError("Teensy Motors identifier must be a tuple")

        if not isinstance(id[0], int):
            raise ValueError("Teensy Motors address must be an integer")

        if id[0] < 0 or id[0] > 5:
            raise ValueError("Teensy Motors address {} outside of valid range 0-5".format(id[0]))

        if not isinstance(id[1], bool):
            raise ValueError("Inverted status must be a boolean")

        return tuple(id)

    def recv(self, timeout = 0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')
        return value

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * 6).split()
            return map(int, values)
        except:
            return []

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []

        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0

        self.mutex.release()
        return ack == 'OK'

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) == 2:
            return values
        elif len(values) == 6:
            # Rover motor encoders
            #   1   0
            #   3   2
            #   5   4
            self.isRover = True
            return [ values[3], values[2] ]
        else:
            print "Encoder count was not 2: " + str(values)
            raise SerialException
            return None

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def set_max_current(self, id, current):
        sid, center, inverted = self.check_id(id)
        self.check_sp()
        # Does nothing

    def init_velocity(self, id):
        sid, inverted = self.check_id(id)
        self.check_sp()
        # Does nothing

    def velocity(self, id, velocity):
        """ Runs motor at specified +/- ticks/period """
        mid, inverted = self.check_id(id)

        ticks = int(velocity)

        if inverted:
            ticks = ticks * -1

        if abs(ticks) < 20:
            ticks = 0

        self.execute_ack("n %d %d\r" % (mid, ticks))

    def init_angle(self, id):
        pass

    def maxangle(self, id):
        return 0

    def angle(self, id, angle):
        pass

    def steer_setzero(self, id):
        pass

    def input_voltage(self, id):
        return 0


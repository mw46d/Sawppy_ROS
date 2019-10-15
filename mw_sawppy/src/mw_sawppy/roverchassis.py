#!/usr/bin/env python
"""
MIT License

Copyright (c) 2018 Roger Cheng

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import math
import lewansoul_wrapper
import teensy_motors_wrapper

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os
import sys
import thread
import traceback

from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

# Python 2 does not have a constant for infinity. (Python 3 added math.inf.)
infinity = float("inf")

class roverwheel:
  """
  Rover wheel class tracks information specific to a particular wheel on
  the chassis.
  """
  def __init__(self, name, x = 0, y = 0, rollingcontrol = None, rollingparam = None,
    steeringcontrol = None, steeringparam = None):

    # String is used to identify this wheel in various operations. Bonus if
    # the words make sense to a human reader ('front_left') but not required.
    self.name = name

    # X,Y coordinate of this wheel on the rover chassis, relative to center.
    self.x = x
    self.y = y

    # Rolling velocity motor (optional): a reference to the control object and
    # the parameters to identify this wheel to the control.
    self.rollingcontrol = rollingcontrol
    self.rollingparam = rollingparam

    # Steering angle motor (optiona): similar to the above, but for steering.
    self.steeringcontrol = steeringcontrol
    self.steeringparam = steeringparam

    # The most recently commanded steering angle and rolling velocity
    self.angle = 0
    self.velocity = 0     # now in m/s
    self.des_v = 0        # now in encoder ticks/period
    self.v = 0            # now in encoder ticks/period
    self.last_adj = 0.001 # Just a reprensentation of zero

    # If we were given a rolling velocity control, run any initialization we
    # need and obtain its label string to show to user.
    if self.rollingcontrol:
      self.rollingcontrol.init_velocity(self.rollingparam)
      try:
        self.rollinglabel = self.rollingcontrol.version(self.rollingparam)
      except ValueError as ve:
        self.rollinglabel = "(No Response)"

    # Repeat the above, this time for steering angle control.
    if self.steeringcontrol:
      self.steeringcontrol.init_angle(self.steeringparam)
      try:
        self.steeringlabel = self.steeringcontrol.version(self.steeringparam)
      except ValueError as ve:
        self.steeringlabel = "(No Response)"

  def poweroff(self):
    """
    Instructs the motor controller to stop rolling, stop holding position,
    whatever is the least-effort situation. (If applicable)
    """
    self.velocity = 0
    if self.rollingcontrol:
      self.rollingcontrol.power_percent(self.rollingparam, 0)

    # Killing the power leaves the angle wherever it was last (except as
    # moved by external forces) so leave self.angle alone.
    if self.steeringcontrol:
      self.steeringcontrol.power_percent(self.steeringparam, 0)

  def anglevelocity(self):
    """
    Send the dictated angle and velocity to their respective controls
    """
    if self.rollingcontrol:
      self.rollingcontrol.velocity(self.rollingparam, self.v)

    if self.steeringcontrol:
      self.steeringcontrol.angle(self.steeringparam, self.angle)

  def steerto(self, angle):
    """
    Steer this wheel to the specified angle. Caller is responsible for
    validation of all parameters.
    """
    self.steeringcontrol.angle(self.steeringparam, angle)

  def steersetzero(self):
    """
    Set the current steering angle of this wheel as the new zero. Caller is
    responsible for validation of all parameters
    """
    self.steeringcontrol.steer_setzero(self.steeringparam)

  def motor_voltage(self):
    """
    Query the rolling and steering motor controllers for their current input
    voltage levels
    """
    voltages = dict()

    if self.rollingcontrol:
      voltages["Rolling"] = self.rollingcontrol.input_voltage(self.rollingparam)
    else:
      voltages["Rolling"] = "Not Applicable"

    if self.steeringcontrol:
      voltages["Steering"] = self.steeringcontrol.input_voltage(self.steeringparam)
    else:
      voltages["Steering"] = "Not Applicable"

    return voltages

class chassis:
    """
    Rover chassis class tracks the physical geometry of the chassis and uses
    that informaton to calculate Ackerman steering angles and relative
    velocity for wheel travel
    """

    def __init__(self, base_frame):
        # List of wheels
        # Each wheel is a dictionary mapping name of a wheel to its specific info.
        self.wheels = dict()

        # When turning radius grows beyond this point, the wheel angles are so
        #   miniscule it is indistinguishable from straight line travel. This hard
        #   coded default can be updated based on chassis config by calling
        #   calculate_radius_min_max()
        self.maxRadius = 250

        # Radius representing the tightest turn this chassis can make. Minimum
        #   value of zero indicates chassis is capable of turning in place.
        #   This hard coded default value can be updated based on chassis config by
        #   calling calculate_radius_min_max()
        self.minRadius = 17.75

        # The current (velocity, radius) that dictated wheel angle and velocity.
        #   Velocity unit is up to the caller, math works regardless of units
        #     inches, metric, quadrature pulses, etc.
        #   Radius unit must match those used to specify wheel coordinates.
        self.currentMotion = (0, infinity)

        # A dictionary mapping a name string identifying a motor controller type
        #   to an instance of the motor controller.
        self.motorcontrollers = dict()

        # ROS
        self.base_frame = base_frame
        self.stopped = False

        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = float(rospy.get_param("~base_controller_timeout", 1.0))
        self.accel_limit = float(rospy.get_param('~accel_limit', 0.1))
        self.motors_reversed = float(rospy.get_param("~motors_reversed", False))
        self.max_velocity = float(rospy.get_param("~max_velocity", 0.6))  # m/s

        self.ensureready()

        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "")
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "")
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Kp'] = float(rospy.get_param("~Kp", 20))
        pid_params['Kd'] = float(rospy.get_param("~Kd", 12))
        pid_params['Ki'] = float(rospy.get_param("~Ki", 0))
        pid_params['Ko'] = float(rospy.get_param("~Ko", 50))

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * math.pi)

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        now = rospy.Time.now()
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("~cmd_vel", Twist, self.cmdVelCallback)

        # Clear any old odometry info
        self.motorcontrollers['teensy'].reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size = 5)
        self.odomBroadcaster = TransformBroadcaster()

        # Reserve a thread lock
        self.mutex = thread.allocate_lock()

        rospy.loginfo("Started Rover Chassis controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True

        if missing_params:
            os._exit(1)

        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']

        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']

        self.motorcontrollers['teensy'].update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                # Just the two middle wheels, they represent a differential drive system;-)
                left_enc, right_enc = self.motorcontrollers['teensy'].get_encoder_counts()
            except Exception as e:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count) + "  now: " + e.__class__.__name__)
                rospy.logerr("mw stack trace: " + traceback.format_exc())
                return

            dt = now - self.then
            self.then = now
            dt = dt.to_sec()

            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
                self.enc_left = 0
                self.enc_right = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            print("Odom dt= %f, dleft= %f  dright= %f, last_enc=(%d, %d), enc=(%d, %d)" % (dt, dleft, dright, self.enc_left, self.enc_right, left_enc, right_enc))
            self.enc_right = right_enc
            self.enc_left = left_enc

            dx = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track

            x = math.cos(dth) * dx
            y = -math.sin(dth) * dx
            self.x += (math.cos(self.th) * x - math.sin(self.th) * y)
            self.y += (math.sin(self.th) * x + math.cos(self.th) * y)
            self.th += dth

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = math.sin(self.th / 2.0)
            quaternion.w = math.cos(self.th / 2.0)

            odom = Odometry(header = rospy.Header(frame_id = "odom"), child_frame_id = self.base_frame)
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = dx/dt
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = dth/dt

            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                now,
                self.base_frame,
                "odom"
                )

            self.odomPub.publish(odom)

            self.mutex.acquire()
 
            # XXX Now send the latest info to the servos & motors
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                for wheel in self.wheels.values():
                    wheel.angle = 0
                    wheel.des_v = 0
            
            for wheel in self.wheels.values():
                if wheel.v < wheel.des_v:
                    wheel.v += self.max_accel
                if wheel.v > wheel.des_v:
                    wheel.v = wheel.des_v
                else:
                    wheel.v -= self.max_accel
                    if wheel.v < wheel.des_v:
                        wheel.v = wheel.des_v

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                for wheel in self.wheels.values():
                    wheel.anglevelocity()
                 
            self.mutex.release()

            self.t_next = now + self.t_delta

    def init_motorcontrollers(self):
        """
        Creates the dictionary where a name in the configuration file can be
        matched with its corresponding motor controller.
        """
        try:
            lws = lewansoul_wrapper.lewansoul_wrapper()
            lws.connect()
            self.motorcontrollers['lewansoul'] = lws
        except StandardError as se:
            rospy.logerr("%s: Unable to initialize LewanSoul Servo Library: %s" % (__name__, str(se)))
            os._exit(1)

        try:
            teensy = teensy_motors_wrapper.teensy_motors_wrapper()
            teensy.connect()
            self.motorcontrollers['teensy'] = teensy
        except StandardError as se:
            rospy.logerr("%s: Unable to initialize Teensy Motors Library: %s" % (__name__, str(se)))
            os._exit(1)

    def ensureready(self):
        """
        Makes sure this chassis class is ready for work by ensuring the required
        information is loaded and ready.
        """
        if len(self.wheels) > 0:
            return

        # Initialize motor controller dictionary.
        self.init_motorcontrollers()
        # print(self.motorcontrollers)

        # Load configuration from JSON.
        wheel_config = rospy.get_param('~SGVHAK_Rover')['roverchassis']

        # Using the data in configuration JSON file, create a wheel object.
        for wheel in wheel_config:
            # Retrieve name and verify uniqueness.
            name = wheel['name']
            if name in self.wheels:
                raise ValueError("Duplicate wheel name {} encountered.".format(name))

            # Initialize all optional motor control values to None
            steeringcontrol = None
            steeringparam = None
            rollingcontrol = None
            rollingparam = None

            # Fill in any rolling velocity motor control and associated parameters
            rolling = wheel['rolling']
            if rolling and isinstance(rolling, list):
                rollingtype = rolling[0]
                if len(rolling) == 2:
                    rollingparam = rolling[1]
                else:
                    rollingparam = rolling[1:]
                if rollingtype in self.motorcontrollers:
                    rollingcontrol = self.motorcontrollers[rollingtype]
                else:
                    raise ValueError("Unknown motor control type")

            # Fill in any steering angle motor control and associated parameters
            steering = wheel['steering']
            if steering and isinstance(steering, list):
                steeringtype = steering[0]
                if len(steering) == 2:
                    steeringparam = steering[1]
                else:
                    steeringparam = steering[1:]
                if steeringtype in self.motorcontrollers:
                    steeringcontrol = self.motorcontrollers[steeringtype]
                else:
                    raise ValueError("Unknown motor control type %s steering -> %s" % (name, steeringtype))

            # Add the newly created roverwheel object to wheels dictionary.
            self.wheels[name] = roverwheel(name, wheel['x'], wheel['y'],
                rollingcontrol, rollingparam, steeringcontrol, steeringparam)

        # Update radius min/max based on the rover chassis configuration info
        self.calculate_radius_min_max()

    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()

        velocity = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        # print("cmdVelCallback(" + str(req) + ")")

        radius = infinity

        # deadband
        if abs(velocity) < 0.02:
            velocity = 0.0
            if abs(th) < 0.2:
                th = 0.0
        if abs(th) < 0.03:
            th = 0.0

        if th != 0.0:
            radius = velocity / th
        print("cmdVelCallback(%f, %f, %f) maxRadius= %f" % (velocity, th, radius, self.maxRadius))

        """
        Given the desired velocity and turning radius, update the angle and
        velocity required for each wheel to perform the desired motion.

        Velocity and radius is given relative to rover center.

        Radius of zero indicates a turn-in-place movement. (Not yet implemented)
        Radius of infinity indicates movement in a straight line.
        """
        # if abs(radius) < self.minRadius:
        #     # This chassis configuration could not make that tight of a turn.
        #     raise ValueError("Radius below minimum")

        self.currentMotion = (velocity, radius)

        self.mutex.acquire()
        if radius > self.maxRadius:
            # Straight line travel
            for wheel in self.wheels.values():
                wheel.angle = 0
                wheel.velocity = velocity
                wheel.last_adj = 0.001
        else:
            # Calculate angle and velocity for each wheel
            for wheel in self.wheels.values():
                # Dimensions of triangle representing the wheel. Used for calculations
                # in form of opposite, adjacent, and hypotenuse
                opp = wheel.y
                adj = radius - wheel.x
                hyp = math.sqrt(pow(opp, 2) + pow(adj, 2))

                # Calculate wheel steering angle to execute the commanded motion.
                if adj == 0:
                    if wheel.last_adj > 0.0:
                        adj = 0.001
                    else:
                        adj = -0.001

                wheel.angle = math.degrees(math.atan(float(opp) / float(adj)))
                wheel.last_adj = adj

                # Calculate wheel rolling velocity to execute the commanded motion.
                if abs(radius) < 0.01:
                    wheel.velocity = 0 # TODO: Velocity calculation for spin-in-place where radius is zero
                    wheel.velocity = velocity * hyp     # XXX Should be normalized below?!
                else:
                    wheel.velocity = velocity * hyp / abs(radius)

                # If center of rotation is within the wheel track, and between the
                # wheel and the origin, then this wheel will need to turn in the
                # opposite direction so the rover body can turn about the center.
                if (radius < 0 and wheel.x < 0 and wheel.x < radius) or (radius > 0 and wheel.x > 0 and wheel.x > radius):
                    wheel.velocity = -wheel.velocity

        # Go back and normalize al the wheel roll rate magnitude so they are at or
        # below target velocity while maintaining relative ratios between their rates.
        maxCalculated = 0

        for wheel in self.wheels.values():
            if abs(wheel.velocity) > maxCalculated:
                maxCalculated = abs(wheel.velocity)

        if maxCalculated > self.max_velocity:
            # At least one wheel exceeded specified maxVelocity, calculate
            # normalization ratio and apply to every wheel.
            reductionRatio = self.max_velocity / float(maxCalculated)
            for wheel in self.wheels.values():
                wheel.velocity = wheel.velocity * reductionRatio

        for wheel in self.wheels.values():
            wheel.des_v = int(wheel.velocity * self.ticks_per_meter / self.motorcontrollers['teensy'].PID_RATE)
        self.mutex.release()

    def calculate_radius_min_max(self):
        """
        Once the wheel configuraton has been loaded, read maximum turning ability
        of the wheels and calculate the minimum turning radius. Also look at the
        radius when the wheels are turned a single degree and use that as maximum
        turning radius.

        Initial values are established by first finding the wheel with the
        greatest X distance from center. Minimum radius is 1% of its distance, and
        maximum is 10 times (1000%) of the distance.
        """
        wheel_x_max = 0
        for wheel in self.wheels.values():
            if abs(wheel.x) > wheel_x_max:
                wheel_x_max = abs(wheel.x)

        if wheel_x_max > 0:
            limit_min = wheel_x_max * 0.01
            limit_max = wheel_x_max * 10

        # Initial values established, now let's look at each wheel and calculate
        # its relative min/max and compare against initial values.
        for wheel in self.wheels.values():
            if wheel.steeringcontrol:
                angle_max = wheel.steeringcontrol.maxangle(wheel.steeringparam)
                if angle_max < 90:
                    limit_radius = wheel.x + (wheel.y/math.tan(math.radians(angle_max)))
                    if limit_radius > limit_min:
                        limit_min = limit_radius
                # If the rover uses steering mechanism that can be more precise than
                # +/- one degree, decrease the value accordingly.
                limit_radius = wheel.x + (wheel.y/math.tan(math.radians(1)))
                if abs(limit_radius) < limit_max:
                    limit_max = abs(limit_radius)

        self.minRadius = limit_min
        self.maxRadius = limit_max

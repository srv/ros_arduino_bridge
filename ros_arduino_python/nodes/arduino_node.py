#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import os, time
import thread

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.INFO)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyUSB1")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

	# Define lights and laser pins
        self.pin_light_left  = 2;
        self.pin_light_right = 3;
        self.pin_laser  = 5;
        
	# Lights services (on/off)
        rospy.Service('lights_on', Empty, self.LightsOnHandler)
        rospy.Service('lights_off', Empty, self.LightsOffHandler)

        # Laser services (on/off)
        rospy.Service('laser_on', Empty, self.LaserOnHandler)
        rospy.Service('laser_off', Empty, self.LaserOffHandler)

	# Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            r.sleep()
    
    def LightsOnHandler(self, req):
    	# Set directions
    	self.controller.pin_mode(self.pin_light_left,  1)
    	self.controller.pin_mode(self.pin_light_right, 1)
    	# Set on
    	self.controller.digital_write(self.pin_light_left,  0)
    	self.controller.digital_write(self.pin_light_right, 0)
	return EmptyResponse()

    def LightsOffHandler(self, req):
    	# Set directions
    	self.controller.pin_mode(self.pin_light_left,  1)
    	self.controller.pin_mode(self.pin_light_right, 1)
    	# Set off
    	self.controller.digital_write(self.pin_light_left,  1)
    	self.controller.digital_write(self.pin_light_right, 1)
	return EmptyResponse()

    def LaserOnHandler(self, req):
        # Set directions
        self.controller.pin_mode(self.pin_laser,  1)
        # Set on
        self.controller.digital_write(self.pin_laser,  0)
	return EmptyResponse()

    def LaserOffHandler(self, req):
        # Set directions
        self.controller.pin_mode(self.pin_laser,  1)
        # Set off
        self.controller.digital_write(self.pin_laser,  1)
	return EmptyResponse()
    
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")
        
if __name__ == '__main__':
    myArduino = ArduinoROS()

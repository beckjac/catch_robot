#!/usr/bin/env python

from math import atan2, pi
from time import sleep

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Constants
ENABLE = 11
DIRECTION = 13
PULSE = 15

DELAY = 0.002
INCREMENT = pi/400
TOLERANCE = pi/100

TARGET_ID = 0

class Stepper:
    def __init__(self, name='stepper'):
        # Member variables
        self.angle = pi/2;
        
        # Configure IO
        gpio.setup(ENABLE, gpio.OUT)
        gpio.output(ENABLE, gpio.HIGH)
        
        gpio.setup(PULSE, gpio.OUT)
        gpio.setup(DIRECTION, gpio.OUT)
        
        # Configure ROS
        rospy.init_node(name)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_pose, queue_size=1)
        
        return
    
    def get_pose(self, msg):
        # Check the message contents
        pos = None
        for marker in msg.markers:
            if marker.id == TARGET_ID:
                pos = marker.pose.pose.position
                break
        
        if pos == None:
            return
        
        # Calculate new angle
        azimuth = atan2(pos.z, pos.x)
        if abs(azimuth - self.angle) > TOLERANCE:
            self.rotate_to(azimuth)
        
        return
    
    def rotate_to(self, angle):
        while self.angle < angle:
            gpio.output(DIRECTION, gpio.HIGH)
            
            gpio.output(PULSE, gpio.HIGH)
            sleep(DELAY)
            gpio.output(PULSE, gpio.LOW)
            sleep(DELAY)
            
            self.angle += INCREMENT
        
        while self.angle > angle:
            gpio.output(DIRECTION, gpio.LOW)
            
            gpio.output(PULSE, gpio.HIGH)
            sleep(DELAY)
            gpio.output(PULSE, gpio.LOW)
            sleep(DELAY)
            
            self.angle -= INCREMENT
        
        return
    
    def run(self):
        rospy.spin()
        return
    
if __name__ == '__main__':
    stepper = Stepper()
    stepper.run()

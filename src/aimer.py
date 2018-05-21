#!/usr/bin/env python

from math import atan2, pi
from time import sleep

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Constants
AZIMUTH_ENABLE = 11
AZIMUTH_DIRECTION = 13
AZIMUTH_PULSE = 15

ELEVATION_ENABLE = 11
ELEVATION_DIRECTION = 13
ELEVATION_PULSE = 15

DELAY = 0.002
INCREMENT = pi/400
TOLERANCE = pi/200

class BaseStepper:
    def __init__(self, name='stepper'):
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
        
        # TODO: implement antagonist behaviour
        # Calculate new angle
        azimuth = atan2(pos.z, pos.x) - pi/2
        if abs(azimuth) > TOLERANCE:
            if azimuth > 0:
                self.rotate_by(INCREMENT)
            else:
                self.rotate_by(-INCREMENT)
        
        return
    
    def rotate_by(self, angle):
        # TODO: Implement acceleration
        steps = int(abs(angle)/INCREMENT);
        if angle > 0:
            gpio.output(DIRECTION, gpio.HIGH)
        else:
            gpio.output(DIRECTION, gpio.LOW)
        
        for step in xrange(steps):
            gpio.output(PULSE, gpio.HIGH)
            sleep(DELAY)
            gpio.output(PULSE, gpio.LOW)
            sleep(DELAY)
        
        return
    
    def run(self):
        rospy.spin()
        return
    
if __name__ == '__main__':
    stepper = Stepper()
    stepper.run()

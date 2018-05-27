#!/usr/bin/env python

from math import atan2, pi
from time import sleep

import rospy
from geometry_msgs.msg import Point

import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Constants
AZIMUTH_ENABLE = 19
AZIMUTH_DIRECTION = 21
AZIMUTH_PULSE = 23

ELEVATION_ENABLE = 11
ELEVATION_DIRECTION = 13
ELEVATION_PULSE = 15

DELAY = 0.02
INCREMENT = 2*pi/1600
TOLERANCE = 2*INCREMENT

class Stepper():
    def __init__(self, enable_pin, direction_pin, pulse_pin):
        self.enable = enable_pin
        self.direction = direction_pin
        self.pulse = pulse_pin
        
        gpio.setup(self.enable, gpio.OUT)
        gpio.setup(self.direction, gpio.OUT)
        gpio.setup(self.pulse, gpio.OUT)
        
        gpio.output(self.enable, gpio.HIGH)
        
        self.steps = 0
        
        return
    
    def rotate_by(self, angle):
        # TODO: Implement acceleration
        steps = int(abs(angle)/INCREMENT);
        if steps == float('NaN'):
            rospy.logwarn("Got NaN as angle. Ignoring.")
            return
            
        if angle > 0:
            gpio.output(self.direction, gpio.HIGH)
            self.steps += steps
        else:
            gpio.output(self.direction, gpio.LOW)
            self.steps -= steps
        
        for step in xrange(steps):
            gpio.output(self.pulse, gpio.HIGH)
            sleep(DELAY)
            gpio.output(self.pulse, gpio.LOW)
            sleep(DELAY)
        
        return
    
    def rotate_to(self, angle):
        diff = angle - self.steps*INCREMENT
        self.rotate_by(diff)
        return

azimuth = Stepper(AZIMUTH_ENABLE, AZIMUTH_DIRECTION, AZIMUTH_PULSE)
elevation = Stepper(ELEVATION_ENABLE, ELEVATION_DIRECTION, ELEVATION_PULSE)

def aim(msg):
    # Lay in azimuth
    azimuth.rotate_to(msg.x)
    
    # Lay in elevation
    elevation.rotate_to(msg.y)
    
    return

def main():
    # Create callback
    rospy.init_node('aimer')
    
    rospy.Subscriber('firing_solution', Point, aim, queue_size=1)
    
    # Keep alive
    rospy.spin()
    
    return
    
    
if __name__ == '__main__':
    main()

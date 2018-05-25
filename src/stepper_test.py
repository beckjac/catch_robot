#!/usr/bin/env python

from math import pi
from time import sleep
import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)

# Constants
ENABLE = 11
DIRECTION = 13
PULSE = 15

DELAY = 0.0005
INCREMENT = pi/400

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
    
if __name__ == '__main__':
    stepper = Stepper()
    
    stepper.rotate_to(0)
    print "at 0"
    stepper.rotate_to(pi)
    print "at pi"
    stepper.rotate_to(pi/2)
    print "at pi/2"

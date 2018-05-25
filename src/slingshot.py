#!/usr/bin/env python

from math import pi
from time import sleep

import rospy
from std_msgs.msg import Empty

import RPi.GPIO as gpio

# Constants
FIRE_ENABLE = 3
FIRE_DIRECTION = 5
FIRE_PULSE = 7

DELAY = 0.01
INCREMENT = pi/200

def fire(msg):
    # Spin the firing stepper one rotation
    angle = 2*pi
    steps = int(angle/INCREMENT);
    
    for step in xrange(steps):
        gpio.output(FIRE_PULSE, gpio.HIGH)
        sleep(DELAY)
        gpio.output(FIRE_PULSE, gpio.LOW)
        sleep(DELAY)
    
    return

def main():
    # Setup gpio
    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    
    gpio.setup(FIRE_ENABLE, gpio.OUT)
    gpio.setup(FIRE_DIRECTION, gpio.OUT)
    gpio.setup(FIRE_PULSE, gpio.OUT)
    
    gpio.output(FIRE_ENABLE, gpio.HIGH)
    gpio.output(FIRE_DIRECTION, gpio.LOW)
    
    # Create callback
    rospy.init_node('slingshot')
    rospy.Subscriber('fire', Empty, fire, queue_size=1)
    
    # Keep alive
    rospy.spin()

if __name__ == '__main__':
    main()

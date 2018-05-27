#!/usr/bin/env python

from math import pi
from time import sleep

import rospy
from std_msgs.msg import Empty

import RPi.GPIO as gpio

# Constants
FIRE = 5

DELAY = 0.05

def fire(msg):
    # Activate valve
    gpio.output(FIRE, gpio.HIGH)
    sleep(DELAY)
    gpio.output(FIRE, gpio.LOW)
    
    return

def main():
    # Setup gpio
    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    
    gpio.setup(FIRE, gpio.OUT)
    gpio.output(FIRE, gpio.LOW)
    
    # Create callback
    rospy.init_node('cannon')
    rospy.Subscriber('fire', Empty, fire, queue_size=1)
    
    # Keep alive
    rospy.spin()

if __name__ == '__main__':
    main()

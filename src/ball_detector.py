#!/usr/bin/env python

from time import time, sleep

import rospy
from std_msgs.msg import Float32

import RPi.GPIO as gpio

# Constants
TRIG = 10
ECHO = 12

THRESHOLD = 10# cm
RATE = 10 # Hz

def sense(pub):
    # Send trigger pulse
    gpio.output(TRIG, gpio.HIGH)
    sleep(0.00001)
    gpio.output(TRIG, gpio.LOW)
    
    # Listen for echo pulse
    start = time()
    while gpio.input(ECHO) == gpio.LOW:
        start = time()
    
    stop = time()
    while gpio.input(ECHO) == gpio.HIGH:
        stop = time()
    
    # Calculate range
    dist = (stop - start)*34300/2 # Speed of sound 34300 cm/s
    
    # Send
    if dist < THRESHOLD:
        pub.publish(dist)
    
    return

def main():
    # Setup gpio
    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    
    gpio.setup(ECHO, gpio.IN)
    gpio.setup(TRIG, gpio.OUT)
    
    gpio.output(TRIG, gpio.LOW)
    
    # Create publisher
    rospy.init_node('ball_detector')
    pub = rospy.Publisher('ball_detected', Float32, queue_size=1)
    
    # Keep alive
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        sense(pub)
        rate.sleep()

if __name__ == '__main__':
    main()

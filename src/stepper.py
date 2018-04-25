#!/usr/bin/env python

from math import atan2, pi

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)

# Constants
PULSE = 13
DIRECTION = 15
DELAY = 0.020
INCREMENT = pi/4000

class Stepper:
    def __init__(self, name='stepper'):
        # Member variables
        self.angle = pi/2;
        
        # Configure IO
        gpio.setup(PULSE, gpio.OUT)
        gpio.setup(DIRECTION, gpio.OUT)
        
        # Configure ROS
        rospy.init_node(name)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_pose, queue_size=1)
        
        return
    
    def get_pose(self, msg):
        # Check the message is not empty
        if len(msg.markers) == 0:
            return
        
        # Get pose data
        marker = msg.markers[0]
        pos = marker.pose.pose.position
        
        # Calculate new angle
        azimuth = atan2(pos.z, pos.x)
        self.rotate_to(azimuth)
        
        return
    
    def rotate_to(self, angle):
        while self.angle < angle:
            gpio.output(DIRECTION, gpio.HIGH)
            
            gpio.output(PULSE, gpio.HIGH)
            rospy.sleep(DELAY)
            gpio.output(PULSE, gpio.LOW)
            rospy.sleep(DELAY)
            
            self.angle += INCREMENT
        
        while self.angle > angle:
            gpio.output(DIRECTION, gpio.LOW)
            
            gpio.output(PULSE, gpio.HIGH)
            rospy.sleep(DELAY)
            gpio.output(PULSE, gpio.LOW)
            rospy.sleep(DELAY)
            
            self.angle -= INCREMENT
        
        return
    
    def run(self):
        rospy.spin()
        return
    
if __name__ == '__main__':
    stepper = Stepper()
    stepper.run()

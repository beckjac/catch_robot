#!/usr/bin/env python

from math import pi, atan2
from time import sleep

import rospy
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers

# Constants
BALL_THRESH = 10# cm

AIM_TIME = 5
FIRE_TIME = 10

FRIENDLY_ID = 0
ENEMY_ID = 1

class Logic():
    def __init__(self):
        # Members
        self.ball_ready = False
        self.player_pose = None
        self.player_id = None
        
        # Init node
        rospy.init_node('logic')
        
        # Create publishers/subscribers
        self.aim_pub = rospy.Publisher('firing_solution', Point, queue_size=1)
        self.fire_pub = rospy.Publisher('fire', Empty, queue_size=1)
        
        self.ball_sub = rospy.Subscriber('ball_detected', Float32, self.check_ball, queue_size=1)
        self.tag_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_pose, queue_size=1)
        
        return

    def check_ball(self, msg):
        if msg.data < BALL_THRESH:
            self.ball_ready = True
        return

    def get_pose(self, msg):
        for marker in msg.markers:
            if (marker.id == FRIENDLY_ID) or (marker.id == ENEMY_ID):
                self.player_pose = marker.pose.pose.position
                self.player_id = marker.id
                break
        return
    
    def calculate_elevation(self, target_range):
        return pi/4
    
    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            # Notification of ball
            rospy.loginfo("Waiting for ball")
            self.ball_ready = False
            while not self.ball_ready:
                sleep(0.1)

            # Detect AR tag
            rospy.loginfo("Waiting for tag")
            self.player_pose = None
            while self.player_pose == None:
                sleep(0.1)

            # Turn towards/away from tag
            rospy.loginfo("Aiming...")
            if self.player_id == FRIENDLY_ID:
                # Aim at
                azimuth = atan2(self.player_pose.z, self.player_pose.x) - pi/2
                elevation = self.calculate_elevation(self.player_pose.z)
            elif player_id == ENEMY_ID:
                # Aim away
                azimuth = 0 # Away
                elevation = pi/4
            else:
                # This shouldn't happen, since the callback filters
                rospy.logerr("Invalid tag ID: {}".format(self.player_id))
                continue
            
            msg = Point()
            msg.x = azimuth
            msg.y = elevation
            
            self.aim_pub.publish(msg)
            sleep(AIM_TIME)

            # Fire
            rospy.loginfo("Firing!")
            self.fire_pub.publish()
            sleep(FIRE_TIME)
            
            # Return to catching pose
            rospy.loginfo("Returning to catch position")
            msg = Point()
            msg.x = 0
            msg.y = 0
            
            self.aim_pub.publish(msg)
            sleep(AIM_TIME)
        
        return

def main():
    logic = Logic()
    logic.run()

if __name__ == '__main__':
    main()

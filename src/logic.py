#!/usr/bin/env python

from math import pi

import rospy
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers

# Constants
AIM_TIME = 10

FRIENDLY_ID = 0
ENEMY_ID = 1

# Global vars
ball_ready = False
player_pose = None
player_id = None

def check_ball(msg):
    ball_ready = True
    return

def get_pose(msg):
    for marker in msg.markers:
        if (marker.id == FRIENDLY_ID) or (marker.id == ENEMY_ID):
            player_pose = marker.pose.pose.position
            player_id = marker.id
            break
    return

def calculate_elevation(target_range):
    
    return pi/4

def main():
    # Init node
    rospy.init_node('logic')
    
    # Create publishers/subscribers
    aim_pub = rospy.Publisher('firing_solution', Point, queue_size=1)
    fire_pub = rospy.Publisher('fire', Empty, queue_size=1)
    
    ball_sub = rospy.Subscriber('', Float32, check_ball, queue_size=1)
    tag_sub = rospy.Subscriber('', AlvarMarkers, get_pose, queue_size=1)
    
    # Main loop
    while not rospy.is_shutdown():
        # Notification of ball
        ball_ready = False
        while not ball_ready:
            sleep(0.1)

        # Detect AR tag
        player_pose = None
        while player_pose == None:
            sleep(0.1)

        # Turn towards/away from tag
        if player_id == FRIENDLY_ID:
            # Aim at
            azimuth = atan2(player_pose.z, player_pose.x) - pi/2
            elevation = calculate_elevation(player_pose.z)
        elif player_id == ENEMY_ID:
            # Aim away
            azimuth = 0 # Away
            elevation = pi/4
        else:
            # This shouldn't happen, since the callback filters
            rospy.logerr("Invalid tag ID: {}".format(player_id))
            continue
        
        msg = Point()
        msg.x = azimuth
        msg.y = elevation
        
        aim_pub.publish(msg)
        sleep(AIM_TIME)

        # Fire
        fire_pub.publish()
        
        # Return to catching pose
        msg = Point()
        msg.x = 0
        msg.y = 0
        
        aim_pub.publish(msg)
        sleep(AIM_TIME)

if __name__ == '__main__':
    main()

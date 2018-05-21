#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

# Constants
FRIENDLY_ID = 0
ENEMY_ID = 1

def main():
    aim_pub = rospy.Publisher('firing_solution', Point, queue_size=1)
    fire_pub = rospy.Publisher('fire', Empty, queue_size=1)
    
    # Notification of ball
    while not ball_ready:
        sleep(0.1)

    # Detect AR tag

    # Turn towards/away from tag
    if FRIENDLY_ID:
        # Aim at
    elif ENEMY_ID:
        # Aim away
    else:
        # Retry

    # Fire
    fire_pub.publish()
    
    # Return to catching pose
    aim_pub.publish()
    

if __name__ == '__main__':
    main()

#!/usr/bin/env python

"""
Starter script for lab1.
Author: Ivan Xia, Richard Cai, Adarsh Karnati, Chris Correa
"""
import rospy
import numpy as np
import argparse
import tf
from geometry_msgs.msg import Point,PoseArray
    

def hand_pos_pub(start_pos):
    """
    publish the hand position with respect to the start pos

    from geometry_msgs.msg import Point
    tag_pos = rospy.wait_for_message('tag_talk', Point)
    OR
    rospy.Subscriber('tag_talk', Point, callback_fn)

    Parameters
    ----------
    tag_number : int
        AR tag number
    """
    print("hand_pos called")
    rospy.init_node('hand_pub', anonymous=True)
    pub = rospy.Publisher('hand_pub', Point, queue_size=10)
    r = rospy.Rate(100) 

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    from_frame = 'base'
    to_frame = 'left_hand'
    pos, quat = None, None
    
    while not rospy.is_shutdown():
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            pos, quat = listener.lookupTransform(from_frame, to_frame, t)
            pos = np.array(pos) - start_pos
            pub.publish(Point(pos))
        except:
            print ('Could not hand position')
            continue
        r.sleep()

if __name__ == '__main__':
    if not rospy.has_param("/hand_pub/start_pos"):
        raise ValueError("start_pos not found on parameter server")    
    start_pos = rospy.get_param("/hand_pub/start_pos")
    start_pos = np.array(start_pos)
    hand_pos_pub(start_pos)
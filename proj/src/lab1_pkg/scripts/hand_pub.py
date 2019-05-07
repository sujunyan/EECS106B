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
    

def hand_pos_pub():
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
    to_frame = '%s_hand'%(arm)
    pos, quat = None, None
    center_pos = [(final_pos[i]-start_pos[i])/2+start_pos[i] for i in range(3)] 
    center_pos[2] = 0;
    while not rospy.is_shutdown():
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            pos, quat = listener.lookupTransform(from_frame, to_frame, t)       
            pos = [pos[i]-center_pos[i] for i in range(3)]
            #print("Relative hand position get (%f %f %f)"%tuple(pos))
            pub.publish(Point(pos[0],pos[1],pos[2]))
        except Exception as e:
            print ('Could not hand position from %s to %s'%(from_frame,to_frame))
            #print(e)
            #pub.publish(Point(0, 0, 0))
            continue
        r.sleep()
start_pos = None
arm = None
final_pos = None
def get_param():
    global start_pos,arm,final_pos
    if not rospy.has_param("/hand_pub/start_pos"):
        raise ValueError("start_pos not found on parameter server")    
    start_pos = rospy.get_param("/hand_pub/start_pos")
    start_pos = eval(start_pos)
    if not rospy.has_param("/hand_pub/final_pos"):
        raise ValueError("final_pos not found on parameter server")    
    final_pos = rospy.get_param("/hand_pub/final_pos")
    final_pos = eval(final_pos)
    if not rospy.has_param("/hand_pub/arm"):
        raise ValueError("start_pos not found on parameter server")    
    arm = rospy.get_param("/hand_pub/arm")

if __name__ == '__main__':
    get_param()
    #print(start_pos,arm)
    hand_pos_pub()
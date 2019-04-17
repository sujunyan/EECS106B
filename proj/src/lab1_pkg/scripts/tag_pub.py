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
def tag_pub(tags):
    """
    Separate script that constantly searches for the AR tag position and publishes
    it to a separate topic.  If you don't use this and the transform listener cannot
    find the AR tag, then your program will hang until the transform listener finds
    the tag.  To get the latest tag position, you'll have to subscribe to the
    tag_talk topic using the following:

    from geometry_msgs.msg import Point
    tag_pos = rospy.wait_for_message('tag_talk', Point)
    OR
    rospy.Subscriber('tag_talk', Point, callback_fn)

    Parameters
    ----------
    tag_number : int
        AR tag number
    """
    print("tag_pub called")
    rospy.init_node('tag_pub', anonymous=True)
    pub = rospy.Publisher('tag_talk', Point, queue_size=10)
    r = rospy.Rate(100) # 10hz

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    from_frame = 'base'
    pos, quat = None, None
    tags = [tags]
    while not rospy.is_shutdown():
        print(tags)
        for i in range(len(tags)):
            to_frame = 'ar_marker_{}'.format(tags[i])
            try:
                t = listener.getLatestCommonTime(from_frame, to_frame)
                pos, quat = listener.lookupTransform(from_frame, to_frame, t)
                print 'Found marker {}'.format(tags[i])
            except:
                print 'Could not find marker {}'.format(tags[i])
                continue
        if pos is not None:
            pub.publish(*pos)
            #br.sendTransform(pos, quat, rospy.Time.now(), from_frame, to_frame)
        r.sleep()

if __name__ == '__main__':
    # How to run: python src/tag_pub.py -tag 4 6 3
    #  python src/tag_pub.py -tag 4
    parser = argparse.ArgumentParser()
    parser.add_argument('-tags', nargs='+', required=True, help='the tag number')
    args = parser.parse_args()
    print(args.tags)
    try:
        tag_pub(args.tags[0])
    except rospy.ROSInterruptException:
        print("call tag_pub failed!")

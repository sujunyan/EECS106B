#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import sys
import argparse

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg

from lab3.planners import SinusoidPlanner

class Exectutor(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(100)
        self.state = BicycleStateMsg()

    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """
        for t, cmd, state in plan:
            self.cmd(cmd)
            self.rate.sleep()
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : :obj:`BicycleCommandMsg`
        """
        self.pub.publish(msg)

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = msg

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, help='How far to move in the x direction')
    parser.add_argument('-y', type=float, help='How far to move in the y direction')
    parser.add_argument('-theta', type=float, help='How far to turn the turtlebot')
    parser.add_argument('-phi', type=float, help='How far to turn the wheels')
    return parser.parse_args()

if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    args = parse_args()

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()
    
    ex = Exectutor()

    p = SinusoidPlanner(0.3)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    plan = p.plan_to_pose(ex.state, goalState, 0.01, 2)

    ex.execute(plan)






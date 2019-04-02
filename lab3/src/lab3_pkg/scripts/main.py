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
import matplotlib.pyplot as plt
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
        self.stablize_flag = True # if we need to implement the paper about stablization
        rospy.on_shutdown(self.shutdown)

    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """

        # store the data for plotting
        self.desired_state_list = [] # 4xn list (x,y,theta,phi)
        self.true_state_list = [] # 4xn list (x,y,theta,phi)
        self.t_list = [] # 1xn list

        if len(plan) == 0:
            return

        for (t, cmd, state) in plan:
            if self.stablize_flag:
                pass
            self.cmd(cmd)
            self.rate.sleep()
            # store the data for plotting
            self.t_list.append(t)
            self.desired_state_list.append([state.x,state.y,state.theta,state.phi])
            self.true_state_list.append([self.state.x, self.state.y, self.state.theta, self.state.phi])

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

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd(BicycleCommandMsg())

    def plot(self):
        """
        plot the desired and true trajectories
        """
        # x plot
        desired, true = [a[0] for a in self.desired_state_list],[a[0] for a in self.true_state_list]
        plt.figure(1)
        plt.subplot(221)
        plt.plot(self.t_list,desired)
        plt.plot(self.t_list,true)
        plt.ylabel('x')
        plt.xlabel('t')
        plt.legend(["desired_x","true_x"])
        # y plot
        plt.subplot(222)
        desired, true = [a[1] for a in self.desired_state_list],[a[1] for a in self.true_state_list]
        plt.plot(self.t_list,desired)
        plt.plot(self.t_list,true)
        plt.ylabel('y')
        plt.xlabel('t')
        plt.legend(["desired_y","true_y"])
 
        # theta plot 
        plt.subplot(223)
        desired, true = [a[2] for a in self.desired_state_list],[a[2] for a in self.true_state_list]
        plt.plot(self.t_list,desired)
        plt.plot(self.t_list,true)
        plt.ylabel('theta')
        plt.xlabel('t')
        plt.legend(["desired_theta","true_theta"])

        # phi
        plt.subplot(224)
        desired, true = [a[3] for a in self.desired_state_list],[a[3] for a in self.true_state_list]
        plt.plot(self.t_list,desired)
        plt.plot(self.t_list,true)
        plt.ylabel('phi')
        plt.xlabel('t')
        plt.legend(["desired_phi","true_phi"])
  
        plt.figure(2)
        desired_x, true_x = [a[0] for a in self.desired_state_list],[a[0] for a in self.true_state_list]
        desired_y, true_y = [a[1] for a in self.desired_state_list],[a[1] for a in self.true_state_list]
        plt.plot(desired_x,desired_y)
        plt.plot(true_x,true_y)
        plt.ylabel('y')
        plt.xlabel('x')
        plt.legend(["desired","true"])

        plt.show()



def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0.0, help='Desired position in x')
    parser.add_argument('-y', type=float, default=0.0, help='Desired position in y')
    parser.add_argument('-theta', type=float, default=0.0, help='Desired turtlebot angle')
    parser.add_argument('-phi', type=float, default=0.0, help='Desired angle of the (imaginary) steering wheel')
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

    print "Initial State"
    print ex.state

    p = SinusoidPlanner(0.3, 0.3, 2, 3)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    plan = p.plan_to_pose(ex.state, goalState, 0.01, 2)
    
    print "Predicted Initial State"
    print plan[0][2]
    print "Predicted Final State"
    print plan[-1][2]

    ex.execute(plan)
    print "Final State"
    print ex.state
    ex.plot()



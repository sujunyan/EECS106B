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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
ros_rate = 100
delta_t = 5
class Exectutor(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.sub_vel = rospy.Subscriber('/odom', Odometry, self.subscribe_vel)
        self.rate = rospy.Rate(ros_rate)
        self.state = BicycleStateMsg()
        self.real_vel = Twist()
        self.stablize_flag = False # if we need to implement the paper about stablization
        self.length = 0.3
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
        self.true_w = [] # true angular velocity 
        self.desired_w = [] # desired angular velocity

        if len(plan) == 0:
            return

        gamma = 0.5
        for (t, cmd, state,fdk) in plan:
            if self.stablize_flag:
                (u1,u2) = cmd.linear_velocity,cmd.steering_rate
                x0 = self.state2array(state)
                x = self.state2array(self.state)
                err = np.matrix((x - x0))
                #print(err.transpose())
                u =  gamma * fdk * err.transpose()
                print(fdk)
                u1,u2 = u1 + u[0] , u2 + u[1]
                #print(u)
                cmd = BicycleCommandMsg(u1,u2)
            self.cmd(cmd) 
            # store the data for plotting
            self.t_list.append(t)
            self.desired_state_list.append(self.state2array(state))
            self.true_state_list.append(self.state2array(self.state))

            # store the angular velocity
            self.true_w.append(self.real_vel.angular.z)
            desired_w = (1.0/self.length)*np.tan(self.state.phi)*cmd.linear_velocity
            self.desired_w.append(desired_w)

            self.rate.sleep()
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())
    def state2array(self,state):
        return np.array([state.x,state.y,state.theta,state.phi])
    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim
c
        Parameters
        ----------
        msg : :obj:`BicycleCommandMsg`
        """
        self.pub.publish(msg)

    def subscribe_vel(self,msg):
        self.real_vel = msg.twist.twist

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

    def plot_vel(self):
        plt.figure(3)
        desired, true = self.desired_w, self.true_w
        plt.plot(self.t_list,desired)
        plt.plot(self.t_list,true)
        plt.legend(["desired_vel","true_vel"])

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

        #self.plot_vel()
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
    plan = p.plan_to_pose(ex.state, goalState, 1.0/ros_rate, delta_t)
    
    print "Predicted Initial State"
    print plan[0][2]
    print "Predicted Final State"
    print plan[-1][2]

    ex.execute(plan)
    print "Final State"
    print ex.state
    ex.plot()



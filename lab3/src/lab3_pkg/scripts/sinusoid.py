#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import numpy as np
import tf2_ros
import tf
import sys

class Sinusoid():
    """docstring for BangBang"""
    def __init__(self):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(100)
        self.state = BicycleStateMsg()

    def run(self, a1, a2, w1, w2):
        # for i in range(10):
            # self.turn(1, -1)
            # self.strafe()
        self.sin_command(a1, a2, w1, w2)

    def sin_command(self, a1, a2, w1, w2):
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()
            if t > (2*np.pi/min(w1, w2)):
                break
            u1 = a1*np.sin(w1*t)
            u2 = a2*np.cos(w2*t)
            self.cmd_v(u1, u2)
            self.rate.sleep()
        self.cmd(0,0)

    def state2v(self):
        return (self.state.x, self.state.phi, np.sin(self.state.theta), self.state.y)

    def cmd_v(self, v1, v2):
        u1 = v1/np.cos(self.state.theta)
        u2 = v2
        self.cmd(u1, u2)

    def cmd(self, u1, u2):
        self.pub.publish(BicycleCommandMsg(u1, u2))

    def subscribe(self, msg):
        self.state = msg

    def shutdown(self):
        rospy.loginfo("shutting down")
        self.cmd(0,0)


if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    
    s = Sinusoid()
    s.run(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))





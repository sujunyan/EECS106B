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
from math import cos
import matplotlib.pyplot as plt
from std_srvs.srv import Empty as EmptySrv
ros_rate = 10
class BangBang(object):
    """docstring for BangBang"""
    """ For the control on the Lie Algebra part, look at bangbang.py. g3 is turn, and g4 is strafe """
    def __init__(self):
        self.rate = rospy.Rate(ros_rate)
        self.state = BicycleStateMsg()
        self.state_init = BicycleStateMsg()
        self.init = True
        self.state_error = BicycleStateMsg()
         
        self.t = 0
        self.t_list = [0]
        self.current_state_plot = [[0],[0],[0],[0]]   # x , phi, theta, y 
        self.desired_state_plot = [[0,0],[0,0],[0,0],[0,0]]   # x , phi , theta ,y 
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)

    def subscribe(self, msg):
        if self.init:
            self.state_init = msg
            self.init = False
        self.state = msg
    
    def sleep_plot(self):
        self.rate.sleep()
        self.t = self.t + 0.1
        print('self.t', self.t)
        self.t_list.append(self.t)
        self.current_state_plot[0].append(self.state.x)
        self.current_state_plot[1].append(self.state.phi)
        self.current_state_plot[2].append(self.state.theta)
        self.current_state_plot[3].append(self.state.y)

    def plot(self):
        

        # x plot 
        plt.figure(1)
        plt.subplot(221)
        plt.plot(self.t_list, self.current_state_plot[0])
        plt.plot([self.t_list[0],self.t_list[-1]], self.desired_state_plot[0])
        plt.ylim((-0.5,1.2))
        plt.yticks(np.arange(-0.5, 1.2, step =0.1))
        plt.ylabel('x')
        plt.xlabel('t')
        plt.legend(["true_x", "desired_x"])
        plt.grid(True, linestyle = '-.')
        
        # y plot 
        plt.subplot(222)
        plt.plot(self.t_list, self.current_state_plot[3])
        plt.plot([self.t_list[0],self.t_list[-1]], self.desired_state_plot[3])
        plt.ylim((-0.2,1.0))
        plt.yticks(np.arange(-0.2, 1.0, step =0.1))
        plt.ylabel('y')
        plt.xlabel('t')
        plt.legend(["true_y", "desired_y"])
        plt.grid(True, linestyle = '-.')
        
        # phi plot 
        plt.subplot(223)
        plt.plot(self.t_list, self.current_state_plot[1])
        plt.plot([self.t_list[0],self.t_list[-1]], self.desired_state_plot[1])
        plt.ylim((-0.2,0.5))
        plt.yticks(np.arange(-0.2, 0.5, step = 0.1))
        plt.ylabel('phi')
        plt.xlabel('t')
        plt.legend(["true_phi", "desired_phi"])
        plt.grid(True, linestyle = '-.')


        # theta plot  
        plt.subplot(224)
        plt.plot(self.t_list, self.current_state_plot[2])
        plt.plot([self.t_list[0],self.t_list[-1]], self.desired_state_plot[2])
        plt.ylim((-1.5,1.0))
        plt.yticks(np.arange(-1.5, 1.0, step = 0.1))
        plt.ylabel('theta')
        plt.xlabel('t')
        plt.legend(["true_theta", "desired_theta"])
        plt.grid(True, linestyle = '-.')
        
        plt.figure(2)
        plt.plot(self.desired_state_plot[0], self.desired_state_plot[3])
        plt.plot(self.current_state_plot[0], self.current_state_plot[3])
        plt.ylim((-0.2,0.9))
        plt.yticks(np.arange(-0.2, 0.9, step = 0.1))
        plt.ylabel('y')
        plt.xlabel('x')
        plt.legend(["desired","true"])
        plt.grid(True, linestyle = '-.')


        plt.show()


    def run(self):
        self.cmd(0, 0)
        print('init state:',self.state)       
 
        ## parallel park 
        # self.parallel(0.5)

        ## go straight
        # self.straight(1)
        
        ## u turn
        # self.u_turn()

        ## s turn
        self.s_turn(0.5,0.5)

        print('final state:', self.state)
 

    def straight(self, mag):
        x_init = self.state.x
        y_init = self.state.y

        self.desired_state_plot[0][1] = mag

       
        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            print('correct phi')
            self.sleep_plot()
        


        while (self.state.x - x_init)*(self.state.x - x_init) +  (self.state.y - y_init)*(self.state.y - y_init) < mag*mag:

            phi_current = self.state.phi
            phi_error = 0 - phi_current
            self.cmd(0.2, 5*phi_error)

            self.sleep_plot()
            print('straight move')

        self.cmd(0,0)
        self.sleep_plot()
        self.sleep_plot()


    def u_turn(self):
        

        self.desired_state_plot[2][1] = 3.14

        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            self.sleep_plot()

        while abs(self.state.theta - 3.14/2) > 0.05:
            turning = 1.5*(-self.state.theta + 3.14/2)
            print('first turning','theta', self.state.theta)
            self.cmd(0.2,turning)
            self.sleep_plot()

        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            self.sleep_plot()

        while abs(self.state.theta - 3.14) > 0.02:
            turning = 1.5*(-self.state.theta + 3.14)
            print('second turning','theta', self.state.theta)
            self.cmd(-0.2,-turning)
            self.sleep_plot()

        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            self.sleep_plot()

        while self.state.x > 0:
            
            phi_current = self.state.phi
            phi_error = 0 - phi_current
            theta_current = self.state.theta
            theta_error = 3.14 - abs(theta_current)
            y_current = self.state.y
            y_error = abs(y_current)
            self.cmd(0.2, - 0.3 * abs(theta_error))
            self.sleep_plot()
        
        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            self.sleep_plot()

        
        '''
        while self.state.x < 0:
            phi_current = self.state.phi
            phi_error = 0 - phi_current
            self.cmd(-0.2, 5*phi_error)
            self.sleep_plot()
        '''
        '''
        self.cmd(0,0)
        self.sleep_plot()
        self.sleep_plot()
        '''

    def s_turn(self, x_end, y_end):
        

        self.parallel(y_end)
        self.straight(x_end)





    def parallel(self,mag):
        x_init = self.state.x
        y_init = self.state.y
        phi_init = self.state.phi
        theta_init = self.state.theta
        print('start parallel')
        
        self.desired_state_plot[3][1] = mag

        while self.state.x - 1.0 < 0:
            # self.straight(0.1,self.state.x - x_init)
            phi_current = self.state.phi
            phi_error = 0 - phi_current
            self.cmd(0.2, 5*phi_error)
            self.sleep_plot()


        y1 = self.state.y
        while abs(abs(self.state.theta) - (3.14/4)) > 0.03:
            turning = 1.5*(self.state.theta + 3.14/4)
            print('turning back', 'theta',self.state.theta,'turning',turning)
            self.cmd(-0.1,turning)
            self.sleep_plot()
        y2 = self.state.y
        

        print('y for turn: ', y2 - y1)
        self.cmd(0,0)
        self.rate.sleep()
        self.rate.sleep()

        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            print('correct phi')
            self.sleep_plot()
                                               

        while (mag + abs(y2-y1)) - abs(self.state.y) > 0.05:
            self.cmd(-0.2, 0)
            print('move to y',mag+abs(y2-y1) - abs(self.state.y))
            self.sleep_plot()


        print('phi is: ', self.state.phi)

        y3 = self.state.y

        while abs(self.state.theta - 0) > 0.03:
            turning = 1.5*(-self.state.theta + 0)
            print('turning forward','theta', self.state.theta,'turning',turning)
            self.cmd(0.1,turning)
            self.sleep_plot()

        y4 = self.state.y
        print('y for turn: ', y3 - y4)

        self.cmd(0,0)
        self.sleep_plot()
        self.sleep_plot()

        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            self.sleep_plot()

        while self.state.x - x_init < 0:
            # self.straight(0.1,self.state.x - x_init)
            print('straight move')
            self.cmd(0.2, 0)
            self.sleep_plot()

        while self.state.x - x_init > 0:
            print('straight move')
            self.cmd(-0.2, 0)
            self.sleep_plot()




        while abs(self.state.phi - 0) > 0.0001:
            phi_error_cur = 0 - self.state.phi
            self.cmd(0.0001, 5*phi_error_cur)
            print('correct phi')
            self.sleep_plot()

        self.cmd(0,0)
        self.sleep_plot()
        self.sleep_plot()

    def cmd(self, u1, u2):
        # BicycleCommandMsg:
        # The commands to a bicycle model robot (v, phi): linear_velocity & steering_rate
        self.pub.publish(BicycleCommandMsg(u1, u2))


if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()
    


    b = BangBang()
    b.run()
    b.plot()




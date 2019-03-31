#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
from math import *
from scipy.integrate import quad,quadrature,ode,odeint
from scipy import optimize
import sys
from copy import copy

import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import tf2_ros
import tf

class SinusoidPlanner():
    def __init__(self, l, max_phi, max_u1, max_u2):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose

        Parameters
        ----------
        l : float
            length of car
        """
        self.l = l
        self.max_phi = max_phi
        self.max_u1 = max_u1
        self.max_u2 = max_u2
        self.max_y = 0.5
        self.theta_limit = 0.03

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You 
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi 

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))

        theta_path = []
        start_t = 0
        start_state_x = start_state
        # steer the theta first to escape the singularity. Bugs exist since we do not know 
        # if it will hit the singularity point when the system goes.
        #if (max_abs_angle >= np.pi/2) and (min_abs_angle <= np.pi/2):
        delta_theta = abs(goal_state.theta - start_state.theta)
        if delta_theta > self.theta_limit:
            print("\nGenerating theta_path...") 
            theta_path = self.steer_theta(start_state, goal_state,0,dt, delta_t)
            start_t = theta_path[-1][0] + dt
            start_state_x = theta_path[-1][2]
            print(start_state_x)
            #raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")

        if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )
        print("\nGenerating x_path...") 
        x_path =        self.steer_x(
                            start_state_x, 
                            goal_state, 
                            start_t, 
                            dt, 
                            delta_t
                        )
        print("\nGenerating phi_path...") 
        phi_path =      self.steer_phi(
                            x_path[-1][2], 
                            goal_state, 
                            x_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        print("\nGenerating alpha...") 
        alpha_path =    self.steer_alpha(
                            phi_path[-1][2], 
                            goal_state, 
                            phi_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        print("\nGenerating y_path...") 
        goal_state_y  = goal_state
        start_state_y = alpha_path[-1][2]
        n = abs(goal_state_y.y - start_state_y.y) // self.max_y + 1
        n = int(n)
        delta_y = (goal_state_y.y - start_state_y.y) / n
        print("For steer_y n=%d delta_y=%f"%(n,delta_y))
        print("goal",goal_state_y)
        print("start",start_state_y)
        goal_state_y.y = start_state_y.y + delta_y
        y_start_t = alpha_path[-1][0] + dt
        y_path = []
        for i in range(n):
            y_path_tmp = self.steer_y(start_state_y, goal_state_y, y_start_t,dt, delta_t)     
            y_path.extend(y_path_tmp)
            y_start_t = y_path_tmp[-1][0] + dt
            start_state_y.y = goal_state_y.y
            goal_state_y.y += delta_y

        path = []
        for p in [theta_path, x_path, phi_path, alpha_path, y_path]:
            path.extend(p)
        #print(theta_path)
        return path

    def steer_theta(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the theta direction
        Only called when singularity points exist.

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """


        delta_theta =  goal_state.theta - start_state.theta

        # steer phi to max so that it can rotate
        start_state_phi = start_state
        goal_state_phi = copy(start_state)
        goal_state_phi.phi = np.sign(delta_theta) * self.max_phi
        print("In steer_theta start phi is %f goal phi is %f"%(start_state_phi.phi,goal_state_phi.phi))
        path = self.steer_phi(start_state_phi,goal_state_phi,t0,dt,delta_t)
        #print(path)

        # steer u1 to make it rotate
        curr_state = copy(goal_state_phi)
        t = path[-1][0] + dt
        while not rospy.is_shutdown():
            u1,u2 = self.max_u1, 0
            cmd_u = BicycleCommandMsg(u1, u2)
            path.append([t,cmd_u,curr_state])
            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )
            t += dt
            delta_theta = abs(curr_state.theta - goal_state.theta)
            if delta_theta < self.theta_limit:
                break

        #t0 = path[-1][0] + dt
#        start_state_phi = path[-1][2]
#        goal_state_phi = start_state_phi
#        goal_state_phi.phi = start_state.phi
#        path2 = self.steer_phi(start_state_phi,goal_state_phi,t0,dt,delta_t)
#        path.extend(path2)
        return path
        

    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_phi(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the phi direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # copy from ster_x and modify it
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_phi = goal_state_v[1] - start_state_v[1]

        v1 = 0
        v2 = delta_phi/delta_t

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_alpha(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the alpha direction.  
        Remember dot{alpha} = f(phi(t))*u_1(t) = f(frac{a_2}{omega}*sin(omega*t))*a_1*sin(omega*t)
        also, f(phi) = frac{1}{l}tan(phi)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_alpha = goal_state_v[2] - start_state_v[2]

        omega = 2*np.pi / delta_t

        a2 = min(0.9, self.phi_dist * omega)
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
        phi_fn = lambda t: (a2/omega)*np.sin(omega*t) + start_state_v[1]
        integrand = lambda t: f(phi_fn(t))*np.sin(omega*t) # The integrand to find beta
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t,full_output=0)[0]

        a1 = (delta_alpha*omega)/(np.pi*beta1)

              
        print("In steer_alpha a1 %f a2 %f"%(a1,a2))
        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)


    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the y direction. 
        Remember, dot{y} = g(alpha(t))*v1 = frac{alpha(t)}{sqrt{1-alpha(t)^2}}*a_1*sin(omega*t)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_y = goal_state_v[3] - start_state_v[3]
        #print("\n")
        #print("In steer, start ",start_state,"goal",goal_state)
        #print("\n")

        omega = 2 * np.pi / delta_t

        max_a1 = self.max_u1 
        max_a2 = self.max_u2 
        # the gross func to find the root: gross_func = 0
        def gross_func(x):
            a1 = x[0]
            a2 = x[1]
            if abs(a1) > max_a1 or abs(a2) > max_a2:
                r = abs(a1) + abs(a2) # set the constrain
                return [r , r]
            tol = 1e-4
            limit = 10
            def alpha_t(t):
                # alpha = sin(theta)
                # we can calculate theta first to escape the domain check of alpha
                def integrand(t):
                    f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
                    def phi_fn(t):
                        phi = (a2/2/omega)*np.sin(2*omega*t) + start_state_v[1]
                        return phi if abs(phi) < self.max_phi else np.pi/2
                    return f(phi_fn(t)) * np.sin(omega * t) * a1
                result = quad(integrand,0,t,full_output=0,epsabs=tol,epsrel=tol,limit=limit)[0]
                #result = quadrature(integrand,0,t,vec_func=False)[0]
                return result
            def g(a):
                #print(a)
                if abs(a) >1:
                    return np.inf
                return a/sqrt(1-a*a)
            def integrand(t):
                return g(alpha_t(t)) * sin(omega * t)
            int_val = abs(quad(integrand,0,delta_t,full_output=0,epsabs=tol,epsrel=tol,limit=limit)[0])
            #int_val = abs(quadrature(integrand,0,delta_t,vec_func=False)[0])
            return [ (int_val * a1 - delta_y ), 0 ]

        def gross_func_ode(x):
            # try to use ode to handle the itegration
            # v1 = a1 sin wt
            # v2 = a2 cos(2wt)
            (a1,a2) = x
            def my_ode(z,t):
                #ode in original form
                (x,y,theta,phi) = z
                inf_result = [np.inf,np.inf,np.inf,np.inf]
                if x == np.inf:
                    return inf_result
                if cos(theta) == 0 :
                    return inf_result
                u1 = a1 * sin(omega * t) / cos(theta)
                u2 = a2 * cos(2 * omega * t) 
                flag = self.check_limit(u1,u2,phi)
                result = [np.cos(theta)*u1, np.sin(theta)*u1, 1/self.l*tan(phi)*u1, u2]
                return result if flag else inf_result
            z0 = self.state2u(start_state)
            t = np.array([0,delta_t])
            sol = odeint(my_ode,z0,t,printmessg=False)
            y = sol[-1][1] # the final state of y
            return [y - goal_state_v[3],0]

        def find_root(func1):
            # try to find the root of equation func = 0
            init_guess = np.array([delta_y*2, delta_y*2])
            while not rospy.is_shutdown():
                sol = optimize.root(func1,init_guess,method='hybr')
                #print("sol.success\n",sol.success,sol.message)
                if (sol.success):
                    break
                else:
                    init_guess[0] = max_a1 * np.random.rand()
                    init_guess[1] = max_a2 * np.random.rand()
                    print("Find root failed, because %s "%(sol.message))
                    print("change initial guess to (%f,%f) and try again..."%(init_guess[0],init_guess[1]))
                    #print(sol.x)
            return sol.x


        # generate the path              
        while not rospy.is_shutdown():
            (a1,a2) = find_root(gross_func_ode)
            print("In steer_y a1=%f a2=%f delta_y=%f"%(a1,a2,delta_y))
            v1 = lambda t: a1*np.sin(omega*(t))
            v2 = lambda t: a2*np.cos(2*omega*(t))

            path, t = [], t0
            while t < t0 + delta_t:
                path.append([t, v1(t-t0), v2(t-t0)])
                t = t + dt

            u_path = self.v_path_to_u_path(path, start_state, dt)
            if not self.limit_flag:
                break
            else:
                print("Generated y path reached the limit, reduce the max a1 %f a2 %f and try again..."%(max_a1,max_a2))
                mul = 0.99
                max_a1 = max_a1 * mul
                max_a2 = max_a2 * mul
        return u_path
    
    def check_limit(self,u1,u2,phi):
        return abs(u1) <= self.max_u1 and abs(u2) <= self.max_u2 and abs(phi) <= self.max_phi

    def state2u(self,state):
        return np.array([state.x, state.y, state.theta, state.phi])

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray` 
            x, phi, alpha, y
        """
        return np.array([state.x, state.phi, np.sin(state.theta), state.y])

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)
            list of (time, v1, v2) commands
        start_state : :obj:`BicycleStateMsg`
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        self.limit_flag = False # limit flag to indicate if it reaches the limit
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state.theta)
            u2 = v2
            if abs(u1) > self.max_u1:
                print("The limit is reached. u1 %f max %f"%(u1,self.max_u1))
                self.limit_flag = True
            if abs(u2) > self.max_u2:
                print("The limit is reached. u2 %f max %f"%(u2,self.max_u2))
                self.limit_flag = True
            phi = state.phi
            if abs(phi) > self.max_phi:
                print("The limit is reached. phi %f max %f"%(phi,self.max_phi))
                self.limit_flag = True
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]
            # TODO should add limitation to u
            if self.limit_flag:
                return path
            
            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        print path[-1][2]
        return path
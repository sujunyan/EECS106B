#!/usr/bin/env python

"""
Starter script for lab1.
Author: Chris Correa
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from utils.utils import *

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point

class MotionPath(object):
    def __init__(self, limb, kin, total_time,ar_marker_num):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.total_time = total_time
        self.ar_marker_num = ar_marker_num
        self.delta_t =  0.01

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        t = time
        delta_t = self.delta_t
        pos_t_1 = self.target_position(t-delta_t)
        pos_t   = self.target_position(t)
        return (pos_t - pos_t_1) / delta_t

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        t = time
        delta_t = self.delta_t
        pos_t_2 = self.target_position(t-2*delta_t)
        pos_t_1 = self.target_position(t-delta_t)
        pos_t   = self.target_position(t)
        return (pos_t - 2*pos_t_1 + pos_t_2) / (2*delta_t)

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])

        plt.figure()
        plt.subplot(3,2,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,2,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")

        plt.subplot(3,2,3)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,2,4)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")

        plt.subplot(3,2,5)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,2,6)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.show()

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()
        points = []
        for t in np.linspace(0, self.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace)
            points.append(point)

        # We want to make a final point at the end of the trajectory so that the
        # controller has time to converge to the final point.
        extra_point = self.trajectory_point(self.total_time, jointspace)
        extra_point.time_from_start = rospy.Duration.from_sec(self.total_time + 1)
        points.append(extra_point)

        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a
        RobotTrajectory.

        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`
        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            x_t, x_t_1, x_t_2 = None, None, None
            ik_attempts = 0
            theta_t_2 = self.get_ik(self.target_position(t-2*delta_t))
            theta_t_1 = self.get_ik(self.target_position(t-delta_t))
            theta_t   = self.get_ik(self.target_position(t))

            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here?
            point.positions = theta_t
            point.velocities = (theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (2*delta_t)
        else:
            point.positions = self.target_position(t)
            point.velocities = self.target_velocity(t)
            point.accelerations = self.target_acceleration(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point

    def get_ik(self, x, max_ik_attempts=10):
        """
        gets ik

        Parameters
        ----------
        x : 3x' :obj:`numpy.ndarray`
            workspace position of the end effector
        max_ik_attempts : int
            number of attempts before short circuiting

        Returns
        -------
        7x' :obj:`numpy.ndarray`
            joint values to achieve the passed in workspace position
        """
        ik_attempts, theta = 0, None
        while theta is None and not rospy.is_shutdown():
            theta = self.kin.inverse_kinematics(
                position=x,
                orientation=[0, 1, 0, 0]
            )
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                print("\n IK failed with point")
                print(x)
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )
        return theta

    def get_tag_pos(self):
        """
        set time out to 10s
        if specified timeout is exceeded, raise an exception
        store self.tag as 3x np.array vector
        """
        p = rospy.wait_for_message('tag_talk', Point, timeout = 10)
        self.tag_pos = np.array([p.x,p.y,p.z])

    def lookup_tag(tag_number):
        """
        Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
        You can use either this function or try starting the scripts/tag_pub.py script.  More info
        about that script is in that file.

        Parameters
        ----------
        tag_number : int

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            tag position
        """
        listener = tf.TransformListener()
        from_frame = 'base'
        to_frame = 'ar_marker_{}'.format(tag_number)

        r = rospy.Rate(200)
        while (
            not listener.frameExists(from_frame) or not listener.frameExists(to_frame)
        ) and (
            not rospy.is_shutdown()
        ):
            print 'Cannot find AR marker {}, retrying'.format(tag_number)
            r.sleep()

        t = listener.getLatestCommonTime(from_frame, to_frame)
        tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
        return vec(tag_pos)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        ratio = time / self.total_time
        return self._start_pos + ratio *  self._path_diff

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        t = time
        delta_t = self.delta_t
        pos_t_1 = self.target_position(t-delta_t)
        pos_t   = self.target_position(t)
        return (pos_t - pos_t_1) / delta_t

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        t = time
        delta_t = self.delta_t
        pos_t_2 = self.target_position(t-2*delta_t)
        pos_t_1 = self.target_position(t-delta_t)
        pos_t   = self.target_position(t)
        return (pos_t - 2*pos_t_1 + pos_t_2) / (2*delta_t)

class ScanPath(MotionPath):
    def __init__(self, limb, kin, total_time, ar_marker_num,
                start_pos,final_pos, delta_xyz = (0.05,0.05,0.01)):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        start_pos : 3x1 np.array
        final_pos : 3x1 np.array
        """
        super(ScanPath,self).__init__(limb,kin,total_time,ar_marker_num)
        self.start_pos = start_pos
        self.final_pos = final_pos
        
        self.delta_x, self.delta_y, self.delta_z = delta_xyz
        self.cur_xy = np.array([0,0])
        self.duration_up = 2 # the duration of one up motion
        self.duration_down = 2 # the duration of one down motion
        self.duration_xy = 2 # the duration of motion in xy plane
        self.duration = self.duration_down + self.duration_up + self.duration_xy

        delta_pos = final_pos - start_pos
        self.num_x = int(delta_pos[0] / self.delta_x)
        self.num_y = int(delta_pos[1] / self.delta_y)
        self.total_time = self.duration * (self.num_x * self.num_y)


    def target_position(self, time:float):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z position in workspace coordinates of the end effector
        """
        t0 = time % self.duration
        def is_xy_motion():
            return (t0 < self.duration_xy)
        def is_down_motion():
            return (t0 < self.duration_xy + self.duration_down and t0 >= self.duration_xy)
        def is_up_motion():
            return (t0 < self.duration and t0 >= self.duration_down + self.duration_xy)

        if (is_xy_motion()):
            self.next_xy0 = self.next_xy()
            target_xy = t0/float(self.duration) * (self.next_xy0 - self.cur_xy) \
                        + self.cur_xy
            return np.array([target_xy[0], target_xy[1], start_pos[2] ])
        elif (is_down_motion()):
            self.cur_xy = self.next_xy0
        elif (is_up_motion()):
            pass
        else:
            raise ValueError("Should not reach here!")
        

        return super().target_position(time)

    def next_xy(self):
        """
        returns the next [x,y] based on current position
        """
        x,y = self.cur_xy
        # if x is at the end
        if (x == self.num_x and y == self.num_y):
            return self.cur_xy
        elif (y == self.num_y):
            return np.array([x+1,0]) # if y is about to excess
        else:
            return np.array([x,y+1])

        

    def to_robot_trajectory(self, num_waypoints:int=100, jointspace=True):
       return super().to_robot_trajectory(num_waypoints * num_x * num_y,jointspace)
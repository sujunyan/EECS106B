#!/usr/bin/env python

"""
Starter script for lab1.
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib.pyplot as plt
import traceback
import inspect
# Lab imports
from utils.utils import *
import time
import math
# ROS imports
import tf
import rospy
import baxter_interface
import intera_interface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
import moveit_ik
from paths.paths import LinearPath
NUM_JOINTS = 7

class Controller:

    def __init__(self, limb, kin, ik_srv = None):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin
        self.name = 'Controller'
        group = 'right_arm'
        self.ik_srv = moveit_ik.MoveitIK(group)
        print self.ik_srv

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position
        according to the input path and the current time. Each Controller below extends this
        class, and implements this accordingly.

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray`
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray`
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray`
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray`
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray`
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray`
            desired accelerations
        current_index : int
            waypoint index at which search was terminated
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)

    def plot_results(
        self,
        times,
        actual_positions,
        actual_velocities,
        target_positions,
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            actual_workspace_positions[i] = \
                self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
            actual_workspace_velocities[i] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
        # check if joint space
        if target_positions.shape[1] > 3:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i] = \
                    self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
                target_workspace_velocities[i] = \
                    self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])

            # Plot joint space
            plt.figure()
            # print len(times), actual_positions.shape()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Position Error")

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")

            print "Close the plot window to continue"
            plt.show() 

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")
            #plt.legend()

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_workspace_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")
            #plt.legend()
        print "Close the plot window to continue"
        plt.show()




    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)
        start_time = time.time()

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position,
                target_velocity,
                target_acceleration,
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break
        time_used = time.time() - start_time
        print("In controller, time used %f s."%(time_used,))
        if log:
            self.plot_results(
                times,
                actual_positions,
                actual_velocities,
                target_positions,
                target_velocities
            )
        return True

    def follow_ar_tag(self, tag, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You
        should look at execute_path() for inspiration on how to write this.

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """
                # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()


        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)
        start_time = time.time()
        total_time = 1
        path = None
        last_path_time = 0
        
        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                break
                #return False
            if t - last_path_time >= total_time:
                path = None
                last_path_time += 1

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            tag_pos = rospy.wait_for_message('tag_talk', Point)
           
            if not path:
                start_pos = self.get_current_position()[:3]
                h_offset = 0.1
                final_pos = np.array([tag_pos.x, tag_pos.y , tag_pos.z + h_offset])
                path = LinearPath(self._limb,self._kin,total_time,None,start_pos,final_pos)
            if self.name == 'workspace':
                #p = path.trajectory_point(t - last_path_time,0)
                (target_position,
                target_velocity,
                target_acceleration) = self.get_ar_target(tag_pos)
            else:
                p = path.trajectory_point(t - last_path_time,1)
                (target_position,
                target_velocity,
                target_acceleration) = (p.positions ,p.velocities, p.accelerations)

            
            """
            """

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()
            """
            if current_index >= max_index:
                self.stop_moving()
                break
            """
        time_used = time.time() - start_time
        print("In controller, time used %f s."%(time_used,))
        if log:
            self.plot_results(
                times,
                actual_positions,
                actual_velocities,
                target_positions,
                target_velocities
            )
        return True

    def get_ar_target(self,tag_pos):
        """
        get target position, velocity and acceleration for the follow_ar_tag task.

        Parameters
        ----------
        tag_pos : :obj:`Point`

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray`
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray`
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray`
            desired accelerations
        """

        J_inv = self._kin.jacobian_pseudo_inverse()
        J = self._kin.jacobian()
        h_offset = 0.1
        target_position = np.array([tag_pos.x,tag_pos.y,tag_pos.z + h_offset ,0,0,0]) #6x np array
        current_position = self.get_current_position()
        speed = 0.2
        target_velocity = (target_position - current_position ) * speed
        #target_position = current_position + target_velocity # take a small step ahead
        target_acceleration = np.zeros(6) # might not be correct

        if self.name != 'workspace':
            pos = target_position[:3]
            target_position = self.get_ik(pos)
            """
            delta_t = 0.01
            print("pos ",target_position)
            print("vel ",target_velocity)

            pos1 = target_position[:3] + delta_t*target_velocity[:3]
            pos2 = target_position[:3] + 2*delta_t*target_velocity[:3]
            print("pos1 ",pos1)
            print("pos2 ",pos2)
            target_position = self.get_ik(target_position[:3])
            target_position_delta1 = self.get_ik(pos1)
            target_position_delta2 = self.get_ik(pos2)

            target_velocity = (target_position_delta1 - target_position)/delta_t
            target_acceleration = (target_position_delta1 - target_position) - (target_position_delta2 - target_position_delta1)
            target_acceleration /= delta_t*delta_t
            """
            #print(target_position)
            #target_velocity = J_inv.dot(target_velocity).A1
            #target_acceleration = J_inv.dot(target_acceleration).A1
        else : # This might be redundent
            target_position = target_position[:3]
            target_velocity = target_velocity[:3]
            target_acceleration = target_acceleration[:3]


        return (target_position,target_velocity,target_acceleration)

    def get_current_position(self):
        """
        returns
        --------
        current_position: 6x numpy.ndarray ([3x position, 3x rotation])
        """
        current_position = self._kin.forward_position_kinematics() # 7x vector
        pos = current_position[0:3]
        rot_in_quat = current_position[3:7]
        rot = tf.transformations.euler_from_quaternion(rot_in_quat)
        current_position = np.array([pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]])
        return current_position

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
            joint values to achieve the passed in workspace position (in radians)
        """
        ik_attempts, theta = 0, None
        orien = [0,1,0,0]
        #self.ik_srv = None
        while theta is None and not rospy.is_shutdown():
            if self.ik_srv:
                pose = create_pose_stamped_from_pos_quat(x, orien,"base")
                #print("Hello from get_ik")
                res = self.ik_srv.get_ik(pose) # TODO
                if res.error_code.val == 1:
                    #print res.solution
                    theta = res.solution.joint_state.position[-8:-1] # right hand
                else:
                    print("IK failed with error code %s"%(res.error_code.val))
                #print(res)
                #print(theta)
            else:
                theta = self._kin.inverse_kinematics(
                    position=x,
                    orientation=orien
                    )
            #print(x)
            #traceback.print_exc()
            #print(inspect.stack())
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                print("\n IK failed with point")
                print(x)
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )
        """
        if theta.any() != None:
            print(theta)
            for i in range(len(theta)):
                theta[i] %= 2*math.pi
        """
        return theta


class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_positions(joint_array_to_dict(target_position, self._limb))
        #self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class PDWorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controller should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.name = 'workspace'
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position
        according to the input path and the current time. Each Controller below extends this
        class, and implements this accordingly. This method should call
        self._kin.forward_psition_kinematics() and self._kin.forward_velocity_kinematics() to get
        the current workspace position and velocity and self._limb.set_joint_velocities() to set
        the joint velocity to something.  you may have to look at
        http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html to convert the
        output of forward_velocity_kinematics() to a numpy array.  You may find joint_array_to_dict()
        in utils.py useful

        MAKE SURE TO CONVERT QUATERNIONS TO EULER IN forward_position_kinematics().
        you can use tf.transformations.euler_from_quaternion()

        your target orientation should be (0,0,0) in euler angles and (0,1,0,0) as a quaternion.

        Parameters
        ----------
        target_position: 3x' ndarray of desired positions
        target_velocity: 3x' ndarray of desired velocities
        target_acceleration: 3x' ndarray of desired accelerations
        """
        Kp = self.Kp
        Kv = self.Kv

        """
        current_velocity = self._kin.forward_velocity_kinematics() # PyKDL.Twist
        (vel,rot) = (current_velocity.vel,current_velocity.rot)
        print "current_velocity from origin", current_velocity
        current_velocity = np.array([vel[0],vel[1],vel[2], rot[0],rot[1],rot[2]] )
        """
        J = self._kin.jacobian()
        joint_vel = get_joint_velocities(self._limb)
        current_velocity = J.dot(joint_vel)
        current_velocity = current_velocity.A1
        #print "current_velocity from origin", current_velocity

        current_position = self.get_current_position()

        """
        current_position = self._kin.forward_position_kinematics() # 7x vector
        pos = current_position[0:3]
        rot_in_quat = current_position[3:7]
        rot = tf.transformations.euler_from_quaternion(rot_in_quat)
        current_position = np.array([pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]])

        print "target_pos",target_position
        print "current pos",current_position
        print "target vel",target_velocity
        """
        target_position = np.append(target_position,np.zeros(3)) # add orientation
        target_velocity = np.append(target_velocity,np.zeros(3)) # add angular velocity

        err = target_position - current_position
        err_d = target_velocity - current_velocity

        print "pos err,", err
        print "vel err",err_d
        #print "target vel",target_velocity
        #print "cur vel",current_velocity
        print "\n"

        output_vel = target_velocity + Kp.dot(err) + Kv.dot(err_d) # Note that Kp, Kv are 6x6 diagnol matrixes
        J_inv = self._kin.jacobian_pseudo_inverse()

        output_vel = J_inv.dot(output_vel) # change from workspace vel to joint space
        output_vel = output_vel.A1 # change from matrix to a vector
        #print "\n",output_vel,"\n",target_velocity
        #print joint_array_to_dict(output_vel, self._limb), "\n"
        self._limb.set_joint_velocities(joint_array_to_dict(output_vel, self._limb))


class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv, ik_srv = None):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        ik_srv : obj:` MoveitIk
        """
        Controller.__init__(self, limb, kin, ik_srv = ik_srv)
        self.name = 'jointspace'
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position
        according to the input path and the current time. Each Controller below extends this
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        Kp = self.Kp
        Kv = self.Kv

        current_position = np.array([self._limb.joint_angle(joint) for joint in self._limb.joint_names()])
        current_velocity = np.array([self._limb.joint_velocity(joint) for joint in self._limb.joint_names()])

        err = target_position - current_position
        err_d = target_velocity - current_velocity

        output_vel = target_velocity + Kp.dot(err) + Kv.dot(err_d) # Note that Kp, Kv are 7x7 diagnol matrixes

        print "\n",output_vel
        print joint_array_to_dict(output_vel, self._limb), "\n"
        self._limb.set_joint_velocities(joint_array_to_dict(output_vel, self._limb))


class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv, ik_srv = None):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        ik_srv : obj:` MoveitIk
        """
        Controller.__init__(self, limb, kin, ik_srv = ik_srv)
        self.name = 'torque'
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position
        according to the input path and the current time. Each Controller below extends this
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Look in section 4.5 of MLS.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        Kp = self.Kp
        Kv = self.Kv

        current_position = np.array([self._limb.joint_angle(joint) for joint in self._limb.joint_names()])
        current_velocity = np.array([self._limb.joint_velocity(joint) for joint in self._limb.joint_names()])
        
        err = target_position - current_position
        err_d = target_velocity - current_velocity


        
        
        

        # G = J_T * M_cart * [0,0,0.981,0,0,0]
        J_T = self._kin.jacobian_transpose()
        M_cart = self._kin.cart_inertia()
        g = np.matrix([0,0,-.981,0,0,0]).transpose() 
        G = J_T * M_cart * g 
        G = G.A1
        #G = G.dot(g).A1
        #G = J_T * M_cart * [0,0,0.981,0,0,0]
        # M
        M = self._kin.inertia()
        tau_d =  M.dot(target_acceleration).A1
        #M = M.dot(target_acceleration)
        #C = self._kin.coriolis()

        """
        print("G",G)
        print("M",M)
        print("C",C)
        """

        #output_vel = M + G + Kp.dot(err) + Kv.dot(err_d) 
        #output_torque =  M + G + Kp.dot(err) + Kv.dot(err_d) 
        output_torque = tau_d + Kp.dot(err) + Kv.dot(err_d)  
        print "tau_d",tau_d
        print "target_acceleration",target_acceleration
        print "output",output_torque
        #output_torque = output_torque.A1
        """
        output_torque = np.ones(7)
        out = joint_array_to_dict(output_torque, self._limb)
        cur_effort = self._limb.joint_efforts()

        
        err = [out[key]-cur_effort[key] for key,value in out.iteritems()]
        print "\noutput_torque\n",out
        print "joint_efforts\n",cur_effort
        print "effort error is\n",err
        """
        #print "J_T\n",J_T
        #print "M_cart\n",M_cart
        #print joint_array_to_dict(output_torque, self._limb), "\n"
        #self._limb.set_joint_torques(cur_effort)
        self._limb.set_joint_torques(joint_array_to_dict(output_torque, self._limb))

## comments on gravity
#   
# 





######################
# GRAD STUDENTS ONLY #
######################

class WorkspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """

        Parameters
        ----------
        target_position: 3x' ndarray of desired positions
        target_velocity: 3x' ndarray of desired velocities
        target_acceleration: 3x' ndarray of desired accelerations
        """
        raise NotImplementedError

class JointspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        raise NotImplementedError

#!/usr/bin/env python
"""
Starter script for lab1.
Author: Chris Correa
"""
import copy
import sys
import argparse
import time
import numpy as np
import signal
import time
from paths.paths import LinearPath, CircularPath, MultiplePaths
from controllers.controllers import (
    PDWorkspaceVelocityController,
    PDJointVelocityController,
    PDJointTorqueController,
    FeedforwardJointVelocityController
)
from utils.utils import *
from path_planner import PathPlanner


import rospy
import tf
import baxter_interface
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState
from baxter_pykdl import baxter_kinematics
#print 'Couldn\'t import ROS, I assume you\'re working on just the paths on your own computer'

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
    while (True):
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
            break
        except:
            continue
    return vec(tag_pos)

def get_trajectory(task, ar_marker_num, num_way, controller_name):
    """
    Returns an appropriate robot trajectory for the specified task.  You should
    be implementing the path functions in paths.py and call them here

    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    ar_marker_num : nx1' : list of ar_marker numbers ;

    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    if task == 'line':
        # left_hand
        #start_pos = np.array([0.6, 0.147, 0])
        #final_pos = np.array([0.8, 0.30 , 0])
        #right
        start_pos = np.array([0.555, -0.37, 0])
        final_pos = np.array([0.76, -0.1 , 0])

        path = LinearPath(limb,kin,total_time,ar_marker_num,start_pos,final_pos) # ar_marker_num might be redundent
    elif task == 'circle':
        """
        center_pos = np.array([0.730, 0.253, 0.140])
        path = CircularPath(limb,kin,total_time,ar_marker_num,center_pos)
        h_offset = 0.1
        center_pos = lookup_tag(ar_marker_num[0])
        center_pos = center_pos[0]
        print(center_pos)
        center_pos[2] += h_offset
        """
        center_pos = np.array([0.583, -0.16, -0.1]) # right_hand
        #center_pos = np.array([0.7, 0.23, 0]) # left_hand
        r = 0.1
        path = CircularPath(limb,kin,total_time,ar_marker_num,center_pos,r)
    elif task == 'square':
        """
        tag_pos = [lookup_tag(num) for num in ar_marker_num]
        h_offset = 0.1
        print(tag_pos)
        for i in range(len(tag_pos)):
            tag_pos[i] = tag_pos[i][0]
            tag_pos[i][2] += h_offset
            """
        corners = [np.array([0.73,0.47,0]), np.array([0.73,0.25,0.1])
                ,np.array([0.55, 0.25, 0]), np.array([0.53, 0.47, 0.1])]
        #corners = [np.array([0.73,0.47,0]), np.array([0.73,0.25,-0.1])
                #,np.array([0.55, 0.25, 0]), np.array([0.53, 0.47, 0.1])]
        #corners = tag_pos
        length = len(corners)
        paths = [LinearPath(limb,kin,total_time,ar_marker_num,corners[i],corners[(i+1)%length]) for i in range(length)]
        path = MultiplePaths(paths)
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, controller_name!='workspace')

def get_controller(controller_name):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'workspace':
        ## for circle
        Kp = np.array([4 , 3 , 4,0,0,0]) # 6x array
        Kv = np.array([0, 0.02 , 0,0,0,0])
        #for line
        #Kp = np.array([1, 1 , 4,0,0,0]) # 6x array
        #Kv = np.array([0, 0 , 0,0,0,0])
        ## for square
        #Kp = np.array([1 , 1 , 4,0,0,0]) # 6x array
        #Kv = np.array([0, 0.0 , 0,0,0,0])
        controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'jointspace':
        Kp = np.array([0,4,1,-0.1,5,1,-0.1])   # 7x array
        Kv = np.array([0,0.2,0,0,0,0,0])
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'torque':
        #Line
        #Kp = np.array([100 ,50 ,50 ,20 ,20 ,20, 10])      # 7x array 20 8 28 20 15 0 20
        #Kv = np.array([3,3,2,1,0,1,1])   #3 4 2 4 3 3 4     3 5 2 4 3 3 4
        Kp = np.array([32 ,32 ,32,12 ,20 ,12, 3])      # 7x array 20 8 28 20 15 0 20
        Kv = np.array([0,0,0.1,0,0,0,0])   #3 4 2 4 3 3 4     3 5 2 4 3 3 4
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    elif controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t 1 -ar 1 -c workspace -a left --log
    python scripts/main.py -t 2 -ar 2 -c jointspace -a left --log
    python scripts/main.py -t 3 -ar 3 -c torque -a right --log
    python scripts/main.py -t 1 -ar 4 5 --path_only --log

    You can also change the rate, timeout if you want
    """
    start = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, square.  Default: line'
    )
    ## We may not use it since we are using tag_pub.py
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='workspace',
        help='Options: workspace, jointspace, torque or open_loop.  Default: workspace'
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on
        the real robot"""
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()

    


    rospy.init_node('moveit_node')
    # this is used for sending commands (velocity, torque, etc) to the robot
    limb = baxter_interface.Limb(args.arm)
    # this is used to get the dynamics (inertia matrix, manipulator jacobian, etc) from the robot
    # in the current position, UNLESS you specify other joint angles.  see the source code
    # https://github.com/valmik/baxter_pykdl/blob/master/src/baxter_pykdl/baxter_pykdl.py
    # for info on how to use each method
    kin = baxter_kinematics(args.arm)
    total_time = 5 # seconds


    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.

    planner = PathPlanner('{}_arm'.format(args.arm))
    robot_trajectory = get_trajectory(args.task, args.ar_marker, args.num_way, args.controller_name)
    #print(robot_trajectory)
    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectoryrget_veloci

    if args.controller_name == "workspace":
        pose = create_pose_stamped_from_pos_quat(
            robot_trajectory.joint_trajectory.points[0].positions,
            [0, 1, 0, 0],
            'base'
        )
        plan = planner.plan_to_pose(pose)
    else:
        plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan)

    if args.moveit:
        # LAB 1 PART A
        # by publishing the trajectory to the move_group/display_planned_path topic, you should
        # be able to view it in RViz.  You will have to click the "loop animation" setting in
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        disp_traj = DisplayTrajectory()
        #print(robot_trajectory)
        disp_traj.trajectory.append(robot_trajectory)
        # disp_traj.trajectory_start = planner._group.get_current_joint_values()
        disp_traj.trajectory_start = RobotState()
        pub.publish(disp_traj)

        try:
            raw_input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
        # the lines above will display the trajectory in RViz
        start_time = time.time()
        planner.execute_plan(robot_trajectory)
        time_used = time.time() - start_time
        print("In Moveit, total_time used is %f s\n."%(time_used,))
    else:
        # LAB 1 PART B
        controller = get_controller(args.controller_name)
        try:
            raw_input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory,
            rate=args.rate,
            timeout=args.timeout,
            log=args.log
        )
        if not done:
            print 'Failed to move to position'
            sys.exit(0)

    end = time.time()
    alltime = end - start
    print( "The executed time is")
    print(alltime)

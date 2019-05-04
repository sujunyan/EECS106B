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
from paths.paths import ScanPath
from utils.utils import *
from path_planner import PathPlanner
import json
import yaml

import rospy
import tf
import baxter_interface
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from baxter_pykdl import baxter_kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point


def get_trajectory(task, num_way, saved_file):
    """
    Returns an appropriate robot trajectory for the specified task.  You should
    be implementing the path functions in paths.py and call them here

    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square

    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    def str2floatList(s):
        l = s.split()
        return map(float,l)
    ar_marker_num = None    
    if (saved_file):
        with open(saved_file,'r+') as f:
            s = f.read()
            y = yaml.load(s)
            traj = JointTrajectory()
            traj.joint_names = y['joint_trajectory']['joint_names']
            points = []
            for p in y['joint_trajectory']['points']:
                point = JointTrajectoryPoint()
                point.positions = str2floatList(p['positions'][0])
                point.velocities = str2floatList(p['velocities'][0])
                point.accelerations = str2floatList(p['accelerations'][0])
                point.time_from_start.secs = float(p['time_from_start']['secs'])
                point.time_from_start.nsecs = float(p['time_from_start']['nsecs'])
                points.append(point)
            traj.points = points
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    if task == 'scan':
        #start_pos = np.array([0.474, - 0.4 , -0.04])
        final_pos = np.array([0.82, - 0.6 , -0.04])
        path = ScanPath(limb,kin,total_time,ar_marker_num,\
                        start_pos,final_pos,delta_xyz = (0.05,0.05,0.02)) # ar_marker_num might be redundent
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, True)


def args_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='scan', help=
        'Options: scan,  Default: scan'
    )

    parser.add_argument('-arm', '-a', type=str, default='right', help=
        'Options: left, right.  Default: right'
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
    parser.add_argument('-num_way', type=int, default=100, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on
        the real robot"""
    )
    parser.add_argument('--store', action='store_true', help='store the trajectory')
    parser.add_argument('--saved', type=str, default=None, help=
        'use the saved trajectory'
    )
    return parser.parse_args()

start_pos = None
arm = None
def get_param():
    global start_pos,arm
    if not rospy.has_param("/hand_pub/start_pos"):
        raise ValueError("start_pos not found on parameter server")    
    start_pos = rospy.get_param("/hand_pub/start_pos")
    if not rospy.has_param("/hand_pub/arm"):
        raise ValueError("start_pos not found on parameter server")    
    arm = rospy.get_param("/hand_pub/arm")



if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages

    """
    args = args_parse()
    rospy.init_node('Unnamed_node')

    #args.arm = arm

    limb = baxter_interface.Limb(args.arm)
    kin = baxter_kinematics(args.arm)
    total_time = 5 # seconds


    planner = PathPlanner('{}_arm'.format(args.arm))

    robot_trajectory = get_trajectory(args.task, args.num_way, args.saved)
    if args.store:
        print("storing the robot_trajectory")
        with open("scan","w+") as f:
            def msg2json(msg):
                y = yaml.load(str(msg))
                return json.dumps(y,indent=4)
            print >>f, str(robot_trajectory)
        #print("robot_trajectory")
    # Execute to the start point
    
    print(robot_trajectory.joint_trajectory.points[0].positions)
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan)

    # by publishing the trajectory to the move_group/display_planned_path topic, you should
    # be able to view it in RViz.  You will have to click the "loop animation" setting in
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)
    try:
        raw_input('Press <Enter> to execute the trajectory using MOVEIT')
    except KeyboardInterrupt:
        sys.exit()
    # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
    # the lines above will display the trajectory in RViz
    planner.execute_plan(robot_trajectory)
    

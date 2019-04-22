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


import rospy
import tf
import baxter_interface
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState
from baxter_pykdl import baxter_kinematics

def get_trajectory(task, num_way):
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
    ar_marker_num = None    
    if task == 'scan':
        start_pos = np.array([0.474, -0.4 , -0.04])
        final_pos = np.array([0.82, -0.6 , -0.04])
        path = ScanPath(limb,kin,total_time,ar_marker_num,\
                        start_pos,final_pos,delta_xyz = (0.05,0.05,0.02)) # ar_marker_num might be redundent
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, True)


def args_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='scan', help=
        'Options: scan.  Default: scan'
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
    return parser.parse_args()
    
if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages

    """
    args = args_parse()
    rospy.init_node('Unnamed_node')
    limb = baxter_interface.Limb(args.arm)
    kin = baxter_kinematics(args.arm)
    total_time = 5 # seconds


    planner = PathPlanner('{}_arm'.format(args.arm))

    robot_trajectory = get_trajectory(args.task, args.num_way)

    with open("robot_trajectory","w+") as f:
        print >>f, robot_trajectory
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


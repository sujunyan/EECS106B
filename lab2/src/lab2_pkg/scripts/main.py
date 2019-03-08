#!/usr/bin/env python
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import scipy
import sys
import argparse

# AutoLab imports
from autolab_core import RigidTransform
import trimesh

# 106B lab imports
from lab2.policies import GraspingPolicy
from lab2.utils import *

try:
    import rospy
    import tf
    from baxter_interface import gripper as baxter_gripper
    from path_planner import PathPlanner
    from intera_interface import gripper as sawyer_gripper
    ros_enabled = True
except:
    print 'Couldn\'t import ROS.  I assume you\'re running this on your laptop'
    ros_enabled = False

ros_enabled = True
GRIPPER_CONNECTED = True

def translate_g(g,p):
    """
    Parameters
    ----------
    g : 4x4 : :obj:`numpy.ndarray`
    p : 3x1 : :obj:`numpy.ndarray`

    Returns
    ---------------
    g : 4x4 : :obj:`numpy.ndarray`. New g translated by p
    """
    g[0:3,3] = g[0:3,3] + p
    return g

def create_rotation_from_RPY(a,b,c):
    """
    takes roll, pitch yaw and return a rotation matrix

    Parameters
    ------------
    a,b,c: roll,pitch,yaw

    Returns
    ------------
    R 3x3
    """
    result = np.array([ [cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c) + sin(a)*sin(c)],
                      [sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c)],
                      [-sin(b)      , cos(b)*sin(c)                     , cos(b)*cos(c)]
                    ])
    return result

def lookup_transform(to_frame, from_frame='base'):
    """
    Returns the AR tag position in world coordinates

    Parameters
    ----------
    to_frame : string
        examples are: ar_marker_7, gearbox, pawn, ar_marker_3, etc
    from_frame : string
        lets be real, you're probably only going to use 'base'

    Returns
    -------
    :obj:`autolab_core.RigidTransform` AR tag position or object in world coordinates
    """
    if not ros_enabled:
        print 'I am the lookup transform function!  ' \
            + 'You\'re not using ROS, so I\'m returning the Identity Matrix.'
        return RigidTransform(to_frame=from_frame, from_frame=to_frame)

    #if BAXTER_CONNECTED:
        #rospy.init_node('moveit_node')



    listener = tf.TransformListener()
    attempts, max_attempts, rate = 0, 10, rospy.Rate(1.0)
    while attempts < max_attempts and not rospy.is_shutdown():
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
            print("Find transform!")
            break;
        except:
            rate.sleep()
            print("Try to find transform failed...")
            attempts += 1
    rot = RigidTransform.rotation_from_quaternion(tag_rot)
    return RigidTransform(rot, tag_pos, to_frame=from_frame, from_frame=to_frame)

def execute_grasp(T_grasp_world, planner, gripper):
    """
    takes in the desired hand position relative to the object, finds the desired
    hand position in world coordinates.  Then moves the gripper from its starting
    orientation to some distance BEHIND the object, then move to the  hand pose
    in world coordinates, closes the gripper, then moves up.

    Parameters
    ----------
    T_grasp_world : :obj:`autolab_core.RigidTransform`
        desired position of gripper relative to the world frame
    """
    def close_gripper():
        """closes the gripper"""
        if gripper:
            gripper.close()
        rospy.sleep(1.0)

    def open_gripper():
        """opens the gripper"""
        if gripper:
            gripper.open()
        rospy.sleep(1.0)

    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return

    open_gripper()
    g = T_grasp_world.matrix
    # move an approxiamate pos to avoid collision
    offest_p = - g[0:3,3] * 0.1 # move backwords with some distance
    #offest_p = np.array([-0.05,0.05,0.05])
    g = translate_g(g,offest_p)
    target_pose = create_pose_from_rigid_transform(g)
    plan = planner.plan_to_pose(target_pose)
    planner.execute_plan(plan)
    rospy.sleep(1.0)

    g = translate_g(g,-offest_p)
    target_pose = create_pose_from_rigid_transform(g)
    plan = planner.plan_to_pose(target_pose)
    planner.execute_plan(plan)
    rospy.sleep(1.0)
    close_gripper()

    # move to new position
    offest_p[2] = 0
    g = translate_g(g,offest_p)
    target_pose = create_pose_from_rigid_transform(g)
    plan = planner.plan_to_pose(target_pose)
    planner.execute_plan(plan)
    open_gripper()

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-obj', type=str, default='gearbox', help=
        """Which Object you\'re trying to pick up.  Options: gearbox, nozzle, pawn.
        Default: gearbox"""
    )
    parser.add_argument('-n_vert', type=int, default=1000, help=
        'How many vertices you want to sample on the object surface.  Default: 1000'
    )
    parser.add_argument('-n_facets', type=int, default=32, help=
        """You will approximate the friction cone as a set of n_facets vectors along
        the surface.  This way, to check if a vector is within the friction cone, all
        you have to do is check if that vector can be represented by a POSITIVE
        linear combination of the n_facets vectors.  Default: 32"""
    )
    parser.add_argument('-n_grasps', type=int, default=500, help=
        'How many grasps you want to sample.  Default: 500')
    parser.add_argument('-n_execute', type=int, default=5, help=
        'How many grasps you want to execute.  Default: 5')
    parser.add_argument('-metric', '-m', type=str, default='compute_force_closure', help=
        """Which grasp metric in grasp_metrics.py to use.
        Options: compute_force_closure, compute_gravity_resistance, compute_custom_metric"""
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('--baxter', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is
        so you can run this on your laptop"""
    )
    parser.add_argument('--sawyer', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is
        so you can run this on your laptop"""
    )
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

if __name__ == '__main__':

    args = parse_args()

    rospy.init_node('grasp_main_node')

    if args.debug:
        np.random.seed(0)

    # Mesh loading and pre-processing
    mesh = trimesh.load('~/EECS106B/lab2/src/lab2_pkg/objects/{}.obj'.format(args.obj))

    T_obj_world = lookup_transform(args.obj)
    mesh.apply_transform(T_obj_world.matrix)
    print("Transform applied")
    mesh.fix_normals()
    print("Normals fixed")
    # This policy takes a mesh and returns the best actions to execute on the robot
    grasping_policy = GraspingPolicy(
        args.n_vert,
        args.n_grasps,
        args.n_execute,
        args.n_facets,
        args.metric
    )
    print("grasping_policy got")
    # Each grasp is represented by T_grasp_world, a RigidTransform defining the
    # position of the end effector
    T_grasp_worlds = grasping_policy.top_n_actions(mesh, args.obj,vis=False )
    print("T_grasp_worlds got")
    # Execute each grasp on the baxter / sawyer
    if args.baxter or args.sawyer:
        if args.baxter:
            gripper = baxter_gripper.Gripper(args.arm)
            planner = PathPlanner('{}_arm'.format(arm))
        elif args.sawyer:
            if GRIPPER_CONNECTED:
                gripper = sawyer_gripper.Gripper('right')
            else:
                gripper = None
            planner = PathPlanner('right_arm')

        # add the obstacle table
        g = np.eye(4)
        size = np.array([1,1,1])
        g[0:3,3] = T_obj_world.translation
        g[2,3] = grasping_policy.ground_z - size[2]
        sPose = PoseStamped()
        sPose.pose = create_pose_from_rigid_transform(g)
        planner.add_box_obstacle( size, 'table', sPose)
        """
        rot = create_rotation_from_RPY(-3.141, 0.002, 1.382)
        T_grasp_world = RigidTransform(translation=np.array([ 0.672, 0.421, -0.112]),
                                        rotation=rot)
        execute_grasp(T_grasp_world, planner, gripper)
        """
        #print(T_obj_world)
        for T_grasp_world in T_grasp_worlds:
            T_grasp_world, best_vertices, quality = T_grasp_world
            print('\nExecute the grasp with vertices')
            print(best_vertices)
            repeat = True
            while repeat and not rospy.is_shutdown():
                execute_grasp(T_grasp_world, planner, gripper)
                repeat = raw_input("repeat? [y|n] ") == 'y'

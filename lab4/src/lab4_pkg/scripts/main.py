#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import sys
import argparse

# # AutoLab imports
# from autolab_core import RigidTransform
# import trimesh

# 106B lab imports
# from lab4.policies import GraspingPolicy
from lab4_pkg.msg import SoftGripperState, SoftGripperCmd

try:
    import rospy
    ros_enabled = True
except:
    print 'Couldn\'t import ROS.  I assume you\'re running this on your laptop'
    ros_enabled = False
    
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
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    if args.debug:
        np.random.seed(0)

    # # Mesh loading and pre-processing
    # mesh = trimesh.load_mesh('objects/{}.obj'.format(args.obj))
    # T_obj_world = lookup_transform(args.obj)
    # mesh.apply_transform(T_obj_world.matrix)
    # mesh.fix_normals()

    # # This policy takes a mesh and returns the best actions to execute on the robot
    # grasping_policy = GraspingPolicy(
    #     args.n_vert, 
    #     args.n_grasps, 
    #     args.n_execute, 
    #     args.n_facets, 
    #     args.metric
    # )
    # # Each grasp is represented by T_grasp_world, a RigidTransform defining the 
    # # position of the end effector
    # T_grasp_worlds = grasping_policy.top_n_actions(mesh, args.obj)

    # # Execute each grasp on the baxter / sawyer
    # if args.baxter:
    #     gripper = baxter_gripper.Gripper(args.arm)
    #     planner = PathPlanner('{}_arm'.format(arm))

    #     for T_grasp_world in T_grasp_worlds:
    #         repeat = True
    #         while repeat:
    #             execute_grasp(T_grasp_world, planner, gripper)
    #             repeat = raw_input("repeat? [y|n] ") == 'y'
    rospy.init_node('main', anonymous=True)
    pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
    rospy.sleep(1)
    print 'a'
    msg = SoftGripperCmd(220,0)
    # msg = SoftGripperCmd(0,0)
    pub.publish(msg)
    print 'b'
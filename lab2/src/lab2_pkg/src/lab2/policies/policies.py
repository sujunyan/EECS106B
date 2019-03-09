#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Grasping Policy for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import random
# Autolab imports
from autolab_core import RigidTransform
import trimesh
from visualization import Visualizer3D as vis3d
import heapq
import rospy
# 106B lab imports
from lab2.metrics import (
    compute_force_closure,
    compute_gravity_resistance,
    compute_custom_metric
)
from lab2.utils import *

# YOUR CODE HERE
# probably don't need to change these (BUT confirm that they're correct)
MAX_HAND_DISTANCE = .04
MIN_HAND_DISTANCE = .01
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1
MIN_DIS_TO_TABLE = 0.03
# TODO
OBJECT_MASS = {'gearbox': .25, 'nozzle': .25, 'pawn': .25}

def vet_distance(vert1, vert2):
    """
    Parameter:
    -----------------------------------

       vert1:  1x3 :obj:`numpy.ndarray`
       vert2:  1x3 :obj:`numpy.ndarray`

    Return :
       obj : distance : double

    """
    return np.linalg.norm(vert1.reshape((3,1)) - vert2.reshape((3,1)))





class GraspingPolicy():
    def __init__(self, n_vert, n_grasps, n_execute, n_facets, metric_name):
        """
        Parameters
        ----------
        n_vert : int
            We are sampling vertices on the surface of the object, and will use pairs of
            these vertices as grasp candidates
        n_grasps : int
            how many grasps to sample.  Each grasp is a pair of vertices
        n_execute : int
            how many grasps to return in policy.action()
        n_facets : int
            how many facets should be used to approximate the friction cone between the
            finger and the object
        metric_name : string
            name of one of the function in src/lab2/metrics/metrics.py
        """
        self.n_vert = n_vert
        self.n_grasps = n_grasps
        self.n_facets = n_facets
        # This is a function, one of the functions in src/lab2/metrics/metrics.py
        self.metric = eval(metric_name)
        self.approach_direction = [0, 0, 1] # TODO
        self.approach_direction = np.array(approach_direction)

    def vertices_to_baxter_hand_pose(self,grasp_vertices, approach_direction):
        """
        takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
        BE CAREFUL ABOUT THE FROM FRAME AND TO FRAME.  the RigidTransform class' frames are
        weird.

        Parameters
        ----------
        grasp_vertices : 2x3 :obj:`numpy.ndarray`
            position of the fingers in object frame
        approach_direction : 3x' :obj:`numpy.ndarray`
            there are multiple grasps that go through contact1 and contact2.  This describes which
            orientation the hand should be in

        Returns
        -------
        :obj:`autolab_core:RigidTransform` Hand pose in the object frame
        """
        v1, v2 = grasp_vertices[0] , grasp_vertices[1]
        center = (v1+v2) /2

        # borrow the idea from look_at_general()
        x = normalize(v1 - v2)
        y = normalize(np.cross(x, approach_direction) )
        z = np.cross(x,y)


        R = np.eye(3)
        R[0:3,0] = x
        R[0:3,1] = y
        R[0:3,2] = z

        print('z direction',z)
        #print('R', R)
        #print(R,np.linalg.det(R))
        handpose = RigidTransform(R, center, from_frame = 'gripper',to_frame = 'object')

        return handpose



    def sample_grasps(self, vertices, normals):
        """
        Samples a bunch of candidate grasps.  You should randomly choose pairs of vertices and throw out
        pairs which are too big for the gripper, or too close too the table.  You should throw out vertices
        which are lower than ~3cm of the table.  You may want to change this.  Returns the pairs of
        grasp vertices and grasp normals (the normals at the grasp vertices)

        Parameters
        ----------
        vertices : nx3 :obj:`numpy.ndarray`
            mesh vertices
        normals : nx3 :obj:`numpy.ndarray`
            mesh normals
        T_ar_object : :obj:`autolab_core.RigidTransform`
            transform from the AR tag on the paper to the object

        Returns
        -------
        n_graspsx2x3 :obj:`numpy.ndarray`
            grasps vertices.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector and there are n_grasps of them, hence the shape n_graspsx2x3
        n_graspsx2x3 :obj:`numpy.ndarray`
            grasps normals.  Each grasp containts two contact points.  Each vertex normal
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3
        """
        vertices_candidates = []
        normals_candidates = []
        factor = 5
        count = 0
        l = vertices.shape[0]

        # the z value of the bottom of the object
        vertices_z = [vertices[i][2] for i in range(l)]
        ground_z = min(vertices_z)
        self.ground_z = ground_z
        print(vertices[:][2])
        #print(ground_z)
        while not rospy.is_shutdown():
            i = random.randrange(0,l)
            j = random.randrange(0,l)
            v1,v2 = vertices[i], vertices[j]
            n1,n2 = normals[i], normals[j]
            #print(v1,v2)
            #print(n1,n2)
            #print(count)
            #print('\n\n')
            #print("vertices candidate",v1,v2)
            #print(vertices_z)
            #print(vertices_z.shape)
            if v1[2] - ground_z < MIN_DIS_TO_TABLE or v2[2] - ground_z < MIN_DIS_TO_TABLE:
                #print("sample too close to the table",ground_z,v1,v2)
                continue
            if n1.dot(n2) > 0:
                #print("")
                continue
            if np.array_equal(v1,v2):
                continue

            distance = vet_distance(v1,v2)
            if distance < MIN_HAND_DISTANCE or distance > MAX_HAND_DISTANCE:
                #print("hand too close")
                continue

            count += 1
            vertices_candidates.append([v1,v2])
            normals_candidates.append([n1,n2])

            if count == self.n_grasps:
                break

        grasp_vertices = np.array(vertices_candidates)
        grasp_normals = np.array(normals_candidates)

        return grasp_vertices , grasp_normals


    def score_grasps(self, grasp_vertices, grasp_normals, object_mass):
        """
        takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality

        Parameters
        ----------
        grasp_vertices : n_graspsx2x3 :obj:`numpy.ndarray`
            grasps.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3
        grasp_normals : mx2x3 :obj:`numpy.ndarray`
            grasps normals.  Each grasp containts two contact points.  Each vertex normal
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3

        Returns
        -------
        :obj:`list` of int
            grasp quality for each
        """
        quality = []
        for i in range(len(grasp_vertices)):
            vertices = grasp_vertices[i]
            normals = grasp_normals[i]
            num_facets = self.n_facets
            mu = CONTACT_MU
            gamma = CONTACT_GAMMA
            quality.append(self.metric(vertices, normals, num_facets, mu, gamma, object_mass))
        return quality

    def vis(self, mesh, grasp_vertices, grasp_qualities):
        """
        Pass in any grasp and its associated grasp quality.  this function will plot
        each grasp on the object and plot the grasps as a bar between the points, with
        colored dots on the line endpoints representing the grasp quality associated
        with each grasp

        Parameters
        ----------
        mesh : :obj:`Trimesh`
        grasp_vertices : mx2x3 :obj:`numpy.ndarray`
            m grasps.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector, hence the shape mx2x3
        grasp_qualities : mx' :obj:`numpy.ndarray`
            vector of grasp qualities for each grasp
        """
        vis3d.mesh(mesh)

        dirs = normalize(grasp_vertices[:,0] - grasp_vertices[:,1], axis=1)
        midpoints = (grasp_vertices[:,0] + grasp_vertices[:,1]) / 2
        grasp_endpoints = np.zeros(grasp_vertices.shape)
        grasp_vertices[:,0] = midpoints + dirs*MAX_HAND_DISTANCE/2
        grasp_vertices[:,1] = midpoints - dirs*MAX_HAND_DISTANCE/2

        for grasp, quality in zip(grasp_vertices, grasp_qualities):
            color = [min(1, 2*(1-quality)), min(1, 2*quality), 0, 1]
            vis3d.plot3d(grasp, color=color, tube_radius=.001)
            g = self.vertices_to_baxter_hand_pose(grasp,self.approach_direction)
            vis3d.plot3d(g[0:3,2], tube_radius=.001) # plot the z-direction of the grasp
        vis3d.show()

    def top_n_actions(self, mesh, obj_name, vis=True):
        """
        Takes in a mesh, samples a bunch of grasps on the mesh, evaluates them using the
        metric given in the constructor, and returns the best grasps for the mesh.  SHOULD
        RETURN GRASPS IN ORDER OF THEIR GRASP QUALITY.

        You should try to use mesh.mass to get the mass of the object.  You should check the
        output of this, because from this
        https://github.com/BerkeleyAutomation/trimesh/blob/master/trimesh/base.py#L2203
        it would appear that the mass is approximated using the volume of the object.  If it
        is not returning reasonable results, you can manually weight the objects and store
        them in the dictionary at the top of the file.

        Parameters
        ----------
        mesh : :obj:`Trimesh`
        vis : bool
            Whether or not to visualize the top grasps

        Returns
        -------
        :obj:`list` of : [`autolab_core.RigidTransform`, best_vertices 2x3 np.array, grasp_quality]
            the matrices T_grasp_world, which represents the hand poses of the baxter / sawyer
            which would result in the fingers being placed at the vertices of the best grasps
        """
        # Some objects have vertices in odd places, so you should sample evenly across
        # the mesh to get nicer candidate grasp points using trimesh.sample.sample_surface_even()
        print("top_n_actions called")
        vertice_candidates , face_index = trimesh.sample.sample_surface_even(mesh,self.n_vert)
        normal = []
        for x in range(len(face_index)):
             normal.append(list(mesh.face_normals[face_index[x]]))
        normal_candidates = np.array(normal)
        print("normal_candidates got")
        grasp_vertices, grasp_normals = self.sample_grasps(vertice_candidates,normal_candidates)
        print('samples got')
        object_mass = OBJECT_MASS[obj_name]
        grasp_qualities = self.score_grasps(grasp_vertices, grasp_normals, object_mass)

        #print('grasp_qualities got')

        rel = sorted(range(len(grasp_qualities)),key = lambda x:grasp_qualities[x])
        #print('rel', rel)
        n_execute = 10
        rel = rel[-n_execute:] # sort is from small to large
        rel.reverse()

        best_vertices = []
        best_grasp_qualities = []
        for i in rel:
            best_vertices.append(list(grasp_vertices[i]))
            best_grasp_qualities.append(grasp_qualities[i])

        best_vertices = np.array(best_vertices)

        print('best_vertices',best_vertices)
        print('qualities',best_grasp_qualities)
        if (vis):
            self.vis(mesh, best_vertices, best_grasp_qualities)
            #self.vis(mesh, grasp_vertices, grasp_qualities)


        T_grasp_worlds = []
        #print(best_vertices[0])


        for x in range(best_vertices.shape[0]):
            T_grasp_worlds.append([self.vertices_to_baxter_hand_pose(best_vertices[x],self.approach_direction),
                                    best_vertices[x], grasp_qualities[x]])

        #print(T_grasp_worlds)
        return T_grasp_worlds

#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Grasp Metrics for EE106B grasp planning lab
Author: Chris Correa
"""
# may need more imports
import numpy as np
import scipy
from lab2.utils import vec, adj, look_at_general, length

from math import *
#import cvxpy as cvx


def vert_angle(vert1, vert2):
    """
    vert : 1*3 : obj : 'numpy.ndarray'

    """
    lx, ly = length(vert1), length(vert2)
    angle = vert1.dot(vert2) / (lx * ly)
    if angle > np.pi /2:
        return np,pi - angle
    else:
        return angle



def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Compute the force closure of some object at contacts, with normal vectors
    stored in normals You can use the line method described in HW2.  if you do you
    will not need num_facets

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors
        will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    pointVector = vertices[0] - vertices[1]
    theta1 = vert_angle(pointVector, normals[0])
    theta2 = vert_angle(-pointVector,normals[1])
    muangle = np.arctan(mu)
    if theta1< muangle and theta2 < muangle:
        return 1
    return 0

def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """
    defined in the book on page 219.  Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors
        will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    grasp map  6x2(num_facets+1):obj:`numpy.ndarray` 
    """

    n = num_facets
    wrench_basis = []
    grasp_map = []
    # maybe need to normalize the unit value
    for i in range(n):
        fi = [cos(i/n*pi*2), mu * sin(i/n*pi*2), 1,0,0,0]
        wrench_basis.append(fi)
    wrench_basis.append([0,0,0,0,0,1])
    wrench_basis = np.array(wrench_basis).transpose()

    v1, v2 = vertices[0], vertices[1]
    n1, n2 = normals[0],  normals[1] 
                              
    #TODO might be inverse(g)                          
    g1      = look_at_general(v1, n1)
    adj1T = adj(g1).transpose()
    G1 = adj1T.dot(wrench_basis)
    g2      = look_at_general(v2, n2)
    adj2T = adj(g2).transpose()
    G2 = adj2T.dot(wrench_basis)

    grasp_map = np.append(G1.transpose(),G2.transpose(),axis=0)
    grasp_map = grasp_map.transpose()
    #print(G1.shape,G2.shape)
    #print(grasp_map.shape)
    return grasp_map
    

def contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute whether the given grasp (at contacts with surface normals) can produce
    the desired_wrench.  will be used for gravity resistance.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors
        will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    bool : whether contact forces can produce the desired_wrench on the object
    """
    n = num_facets
    lb = np.zeros(2*n + 2)
    up = np.array([np.inf for i in range(2*n + 2)]) 
    #up[n] = gamma
    #up[2*n +1] = gamma

    G = get_grasp_map(vertices,normals,num_facets,mu,gamma)
    result = scipy.optimize.lsq_linear(G, desired_wrench,bounds=(lb,up),verbose=0,lsmr_tol = 'auto')

    cost = result.cost
    return (result.success)
                           

def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Gravity produces some wrench on your object.  Computes whether the grasp can
    produce and equal and opposite wrench

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will
        be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    gravity_wrench = [0,0,-object_mass * .981, 0, 0,0]
    gravity_wrench = np.array(gravity_wrench)

    return contact_forces_exist(vertices, normals, num_facets, mu, gamma, gravity_wrench)
    
    



def compute_custom_metric(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    I suggest Ferrari Canny, but feel free to do anything other metric you find.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will
        be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    scores = np.zeros((vertices.shape[0],1))
    

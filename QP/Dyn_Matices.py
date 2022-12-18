#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  3 16:37:13 2017

@author: user
"""
import rbdl
import numpy as np

def CalcM(model, q):
    q = np.array(q, dtype=float)
    M = np.zeros ((model.q_size, model.q_size))
    rbdl.CompositeRigidBodyAlgorithm(model, q, M, True)
    return M
        
def Calch(model, q, qdot):
    damping = -2
    q = np.array(q, dtype=float)
    qdot = np.array(qdot, dtype=float)
    h = np.zeros(model.q_size)
    rbdl.InverseDynamics(model, q, qdot, np.zeros(model.qdot_size), h)
    D = np.eye(len(q))*damping
#    D[-1, -1] = D[-1, -1]/damping*damping2
    return h - np.dot(D, qdot)
    
def CalcJ(model, q, point):
    bodyid = 3
    q = np.array(q, dtype=float)
    Jc = np.zeros((3, model.dof_count))
 
    rbdl.CalcPointJacobian (model, q, bodyid, point, Jc)
    
    Jc = np.vstack((Jc[0, :], Jc[1, :], [1.]*3))       
    return Jc
#    rbdl.CalcPointVelocity6D(model_a, np.zeros(3), 3, tip_a, Jc)
def CalcdJdq(model, q, qdot, point):
    bodyid = 3
    q = np.array(q, dtype=float)
    qdot = np.array(qdot, dtype=float)
    body_accel = rbdl.CalcPointAcceleration(model, q, qdot, \
    np.zeros(model.dof_count), bodyid, point)
    return np.array([body_accel[0], body_accel[1], 0.])
    
def CalcBody2Base(model, q, point):
    bodyid = 3
    q = np.array(q, dtype=float)
    pose = rbdl.CalcBodyToBaseCoordinates(model, q, bodyid, point)
    x = np.array([pose[0], pose[1], np.array(q).sum()])       
    return x
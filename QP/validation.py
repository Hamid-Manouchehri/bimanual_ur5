#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 17:51:13 2017

@author: user
"""
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from numpy import cos
from numpy import deg2rad as d2r
from numpy import rad2deg as r2d
from numpy.linalg import inv, pinv
from scipy.linalg import qr
#from cvxopt import solvers, matrix
import rbdl
from Utils import Anim_bimanual
from scipy.linalg import sqrtm
from numpy.linalg import matrix_rank as mrank

#### This is just to avoid warnings:
from Dyn_Matices import CalcM, Calch, CalcJ, CalcdJdq, CalcBody2Base
####

execfile('Dyn_Matices.py')


def Validation(q, dq, ddq, tau_a, tau_b, F_a, F_b, \
                   model_a, model_b, tip_a, tip_b, \
                   M_o, I_o, h_o, CalcPqr, \
                   q_des, dq_des, ddq_des):
    qdd = ddq[:4]
    zdd = ddq[4:7]
    lambda_a = F_a
    q_a = q[:2]
    q_b = q[2:4]
    dq_a = dq[:2]
    dq_b = dq[2:4]
    
    M_a = CalcM(model_a, q_a)
    M_b = CalcM(model_b, q_b)    
    h_a = Calch(model_a, q_a, dq_a)
    h_b = Calch(model_b, q_b, dq_b)
    
    JJ_a = CalcJ(model_a, q_a, tip_a)
    JJ_b = CalcJ(model_b, q_b, tip_b)
    J_a = np.hstack((JJ_a, np.zeros((2, 2))))
    J_b = np.hstack((np.zeros((2, 2)), JJ_b))
    
    x_a = CalcBody2Base(model_a, q_a, tip_a)
    x_b = CalcBody2Base(model_b, q_b, tip_b)
    
    x_o = q[4:6]
    r_o_a = x_o - x_a
    r_o_b = x_o - x_b
        
    J_oa = np.array([[1, 0,   r_o_a[1]], [0, 1, - r_o_a[0]]])
    J_ob = np.array([[1, 0,   r_o_b[1]], [0, 1, - r_o_b[0]]])
    
    M = np.zeros((4, 4))
    M[:2, :2] = M_a
    M[2:, 2:] = M_b

    h = np.vstack((h_a.reshape(2, 1), h_b.reshape(2, 1))).flatten()
    
    MM_o = np.eye(3)
    MM_o[:2, :2] = M_o
    MM_o[2, 2] = I_o
    hh_o = np.array([h_o[0], h_o[1], 0])
    
    Mqh = np.dot(M, qdd) + h
    Mzh = np.dot(MM_o, zdd) + hh_o
    
    J_a = np.hstack((JJ_a, np.zeros((2, 2))))
    J_b = np.hstack((np.zeros((2, 2)), JJ_b))
    
    tau = np.vstack((tau_a.reshape(2, 1), tau_b.reshape(2, 1))).flatten() 
    
    lhs = np.dot(J_b.T, np.dot(pinv(J_ob.T), Mzh)) + Mqh
    J_rel = J_a - np.dot(J_oa, np.dot(pinv(J_ob), J_b))
    
    
    rhs = tau + np.dot(J_rel.T, lambda_a)
    
    lhs = np.array(lhs.flatten(), dtype=float)
    rhs = np.array(rhs.flatten(), dtype=float)
#    return np.allclose(lhs, rhs)

#    S = np.eye(4)
    
    k, n = J_rel.shape
    
#    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
#    Su = np.hstack((np.zeros((n, n)), np.eye(n)))
    P, qr_Q, qr_R = CalcPqr(J_rel, Su)
    
#    print '*******', J_rel
    
    plhs = np.dot(P, lhs)
    ptau = np.dot(P, tau)
    ptau = np.array(ptau.flatten(), dtype=float)
    
    pinv_P = np.dot(P.T, inv(np.dot(P, P.T)))
    tau2 = np.dot(pinv_P, plhs)
    
    ptau2 = np.dot(P, tau2)
    
#    print ptau2 - plhs
#    print ptau
#    print ">>>", tau2
    
    rhs2 = tau2 + np.dot(J_rel.T, lambda_a)
    
#    print ">>", tau2 - tau
#    print plhs, ptau2
    
#    
#    print 'r>>>>', rhs
#    print 'l ====', lhs
        
#    return np.allclose(plhs, ptau2)
    return dq[4:7] - np.dot(pinv(J_ob), np.dot(J_b, dq[:4]))
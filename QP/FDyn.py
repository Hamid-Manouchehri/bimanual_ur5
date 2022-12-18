#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 16:44:38 2017

@author: user
"""


def dyn(x, t, tau_a, tau_b):
    """
    .dyn  evaluates system dynamics
    """
    x = x.flatten()
    q_a = x[:3]
    q_b = x[3:6]
#    z_o = x[6:9]
#    th_o = x[8]
    dq_a = x[9:12]
    dq_b = x[12:15]
    dz_o = x[15:]
    dth_o = dz_o[2]
    
    M_a = CalcM(model_a, q_a)
    M_b = CalcM(model_b, q_b)    
    h_a = Calch(model_a, q_a, dq_a)
    h_b = Calch(model_b, q_b, dq_b)    
    
    J_a = CalcJ(model_a, q_a, tip_a)
    J_b = CalcJ(model_b, q_b, tip_b)
    dJ_adq_a = CalcdJdq(model_a, q_a, dq_a, tip_a)
    dJ_bdq_b = CalcdJdq(model_b, q_b, dq_b, tip_b)
    
    x_a = CalcBody2Base(model_a, q_a, tip_a)
    x_b = CalcBody2Base(model_b, q_b, tip_b)
    
#    r_a_b = x_a - x_b
    r_o_a = x_o[:2] - x_a[:2]
    r_o_b = x_o[:2] - x_b[:2]
    
    
    J_oa = np.array([[1, 0,   r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
    J_ob = np.array([[1, 0,   r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])
    
    
    sol = ForwardDynamics(M_a, M_b, M_o, \
                    h_a, h_b, h_o, \
                    J_a, J_b, J_oa, J_ob, \
                    r_o_a, r_o_b, \
                    tau_a, tau_b, \
                    dJ_adq_a, dJ_bdq_b, \
                    dth_o) 
    
    save.lambda_a = sol[9:12]
    save.lambda_b = sol[12:]
    save.qdd = sol[:9]
    
    J_o = np.vstack((J_oa, J_ob))
    
    W = np.eye(6)
    invw = inv(W)
    S = np.eye(6)
    if False:       
        aux1 = np.dot(invw, np.dot(S, J_o))
        aux21 = np.dot(J_o.T, np.dot(S.T, invw))
        aux22 = np.dot(S, J_o)
        aux2 = np.dot(aux21, aux22)
        winv = np.dot(aux1, pinv(aux2))
    else:
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
        aux = pinv(np.dot(J_o.T, np.dot(S.T, w_m_s)))
        winv = np.dot(w_m_s, aux)
    
#    winv = 1/2*np.vstack((inv(J_oa.T), inv(J_ob.T)))  
  
    JJ = np.dot(winv, J_o.T)
    N = np.eye(JJ.shape[0]) - JJ
    save.lambda_motion = np.dot(JJ, np.concatenate((save.lambda_a, save.lambda_b)))
    save.lambda_squeeze = np.dot(N, np.concatenate((save.lambda_a, save.lambda_b)))
    
    
    dx = np.concatenate((dq_a, dq_b, dz_o, sol[:9]))

    return dx
    

def ForwardDynamics(M_a, M_b, M_o, \
                    h_a, h_b, h_o, \
                    J_a, J_b, J_oa, J_ob, \
                    r_o_a, r_o_b, \
                    tau_a, tau_b, \
                    dJ_adq_a, dJ_bdq_b, \
                    dth_o):
    
  
    B1 = tau_a - h_a
    B2 = tau_b - h_b
    B3 = -h_o

    roa = np.array([r_o_a[0], r_o_a[1], 0])
    rob = np.array([r_o_b[0], r_o_b[1], 0])
    dJ_oadZ_o = np.cross([0, 0, dth_o], np.cross(roa, [0, 0, dth_o]))
    dJ_obdZ_o = np.cross([0, 0, dth_o], np.cross(rob, [0, 0, dth_o]))

    B4 =  dJ_oadZ_o - dJ_adq_a
    B5 =  dJ_obdZ_o - dJ_bdq_b
    
    B = np.vstack((B1.reshape(3, 1), B2.reshape(3, 1), B3.reshape(3, 1),\
                   B4.reshape(3, 1), B5.reshape(3, 1)))
    
    
    A = np.zeros((15, 15))    
    A[:3, :3] = M_a
    A[:3, 9:12] = -J_a.T
    
    A[3:6, 3:6] = M_b
    A[3:6, 12:] = -J_b.T
    
    A[6:9, 6:9] = M_o
    A[6:9, 9:12] = J_oa.T
    A[6:9, 12:] = J_ob.T

    A[9:12, :3] = J_a
    A[9:12, 6:9] = - J_oa
    
    A[12:, 3:6] = J_b
    A[12:, 6:9] = - J_ob
    
    
    
    
    
        
    return np.dot(np.linalg.inv(A), B).flatten()
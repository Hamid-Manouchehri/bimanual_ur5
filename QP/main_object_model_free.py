# -*- coding: utf-8 -*-
"""
Created on Tue Oct 25 12:23:33 2016

@author: Mohammad Shahbazi
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
from cvxopt import solvers, matrix

import sys
sys.path.append('/home/mshahbazi/projects/rbdl/build/python/')
import rbdl

from Utils import Anim_bimanual
from scipy.linalg import sqrtm
from numpy.linalg import matrix_rank as mrank

#### This is just to avoid warnings:
from Dyn_Matices import CalcM, Calch, CalcJ, CalcdJdq, CalcBody2Base
####

execfile('Dyn_Matices.py')

class GetLambda(object):
    lambda_a = 0
    lambda_b = 0
    qdd = 0
    q_des, qdot_des, qddot_des = 0, 0, 0
    lambda_a_p = 0
    M, h, QR, J_b = 0, 0, 0, 0
    
save = GetLambda()
    
   

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
    
def QRDecompose(J):
        
    JT = J.T
    
    m, n = JT.shape
    
    if m == 0 or n == 0:
        raise TypeError(\
        'Try to calculate QR decomposition, while there is no contact!')
        
    qr_Q, qr_R = qr(JT)
    
    qr_R = qr_R[:n, :]
    
    return qr_Q, qr_R
    
def CalcPqr(J, Su):    
    qr_Q, qr_R = QRDecompose(J)
    return np.dot(Su, qr_Q.T), qr_Q, qr_R
    
def Task2Joint(J_a, J_b, J_oa, J_ob, J_rel, dJ_adq_a, dJ_bdq_b, r_o_a, r_o_b, \
               x_o, dx_o, X_des, Xdot_des, Xddot_des):
    
#    dth_o_des = Xdot_des[2]
    dth_o = dx_o[2]
#    print dx_o
#    invJ_oa = inv(J_oa)
    invJ_ob = inv(J_ob)
    
    J_A = J_rel
    J_B = np.dot(invJ_ob, J_b)
    
#    r_a_o = - np.array([r_o_a[0], r_o_a[1], 0]) 
    dJ_oadz_o = np.cross([0, 0, dth_o], np.cross(r_o_a, [0, 0, dth_o]))
    dJ_obdz_o = np.cross([0, 0, dth_o], np.cross(r_o_b, [0, 0, dth_o]))
    
    dJ_A_dq = dJ_adq_a - dJ_oadz_o - np.dot(J_oa, np.dot(invJ_ob, \
                                                         dJ_bdq_b - dJ_obdz_o))
       
    dJ_B_dq = np.dot(invJ_ob, dJ_bdq_b - dJ_obdz_o)
    
    
    kp = 200
    kd = kp/5

    Xddot_des_A = np.zeros(3)
    Xddot_des_B = Xddot_des + kp*(X_des - x_o) + kd*(Xdot_des - dx_o)
    
    Xddot_des_A = np.vstack((Xddot_des_A.reshape(3, 1), Xddot_des_B.reshape(3, 1)))
    J_A = np.vstack((J_A, J_B))
    dJ_A_dq = np.vstack((dJ_A_dq.reshape(3, 1), dJ_B_dq.reshape(3, 1)))
    J_B = np.array([])

#    print '\n', Xddot_des_A
#    J_A = np.dot(invJ_oa, J_a)
#    dJ_A_dq = np.dot(invJ_oa, dJ_adq_a - dJ_oadz_o)
#    Xddot_des_A = Xddot_des_B
#    N_A = np.eye(J_A.shape[1]) - np.dot(pinv(J_A), J_A)

#    Xddot_des_B = dJ_obdz_o + np.dot(J_ob, Xddot_des_B)
#    J_B = J_b
#    dJ_B_dq = dJ_bdq_b

#    print Xddot_des_A
#    print 'B', Xddot_des_B
#    print "\n\n\nXddot_des_B", Xddot_des_B, '\n'
#    print X_des, x_o, '\n', Xdot_des, dx_o
    
#    Xddot_des_A = Xddot_des_B.copy()
#    print Xddot_des_A
         
    qddot_des = np.dot(pinv(J_A), Xddot_des_A - dJ_A_dq).flatten()
#    print qddot_des.shape
    
#    N_A = np.eye(J_A.shape[1]) - np.dot(pinv(J_A), J_A)
    
#    if J_B.shape[0]:
#        qddot_des += np.dot(np.dot(N_A, pinv(J_B)), Xddot_des_B - dJ_B_dq)
#    if self.J_E.shape[0]:
#        qddot += np.dot(np.dot(self.N_AB, pinv(self.J_E)), \
#        self.xddot_des[self.p3] - self.dJ_E_dq)
    
#    print np.dot(J_A, qddot_des) + dJ_A_dq.flatten() - Xddot_des_A.flatten()
#    print max(np.abs(np.dot(J_A, qddot_des) + dJ_A_dq.flatten() - Xddot_des_A.flatten()))
    
    
    q_des, qdot_des = Traj_Estimate(qddot_des)
    
    
    Xddot_des_after = Xddot_des_B #TODO
    dJ_obdz_o_des = dJ_obdz_o
#    dth_o_des = q_des[3:6].sum()
#    dJ_obdz_o_des = np.cross([0, 0, dth_o_des], np.cross(r_o_b, [0, 0, dth_o_des]))
#    print 'Xddot_des_after!!!!!!!!'
    return q_des, qdot_des, qddot_des, Xddot_des_after, dJ_obdz_o_des
    
def Traj_Estimate(qdd):
    #approximate q_t and qdot_t from qddot_t:
#    try: Dt = cr.t[-1] - cr.t[-2]
#    except: Dt = dt
    Dt = dt # See the above line!!!!
    qdot_des_now = dqctrl_des_prev + Dt*qdd
    q_des_now = qctrl_des_prev + Dt*qdot_des_now 
    
    q_des_now_filtered = []
    for i in q_des_now:
        i = np.mod(i, np.pi*2)
        if i > np.pi: i = i - np.pi*2
        q_des_now_filtered.append(i)
        
    return np.array(q_des_now_filtered), qdot_des_now

    
def traj_plan(t_start, t_end, z_start, z_end, traj_type = 'Quantic'):
        
    x, xd, xdd = z_start
    y, yd, ydd = z_end
    
    if traj_type == 'Quantic':
        f = lambda x: [x**5, x**4, x**3, x**2, x, 1]
        fd = lambda x: [5*x**4, 4*x**3, 3*x**2, 2*x, 1, 0]
        fdd = lambda x: [20*x**3, 12*x**2, 6*x, 2, 0, 0]
        
        
        A = np.array([f(t_start), fd(t_start), fdd(t_start),\
                      f(t_end), fd(t_end), fdd(t_end)])
                          
        traj, traj_d, traj_dd = [], [], []                 
        for i in range(len(x)):
            B = np.array([[x[i], xd[i], xdd[i], y[i], yd[i], ydd[i]]]).T           
            p = np.dot(np.linalg.inv(A), B)
            traj += list([lambda x, p=p: sum([p[0]*x**5, p[1]*x**4, p[2]*x**3, \
            p[3]*x**2, p[4]*x, p[5]])])              
            traj_d.append(lambda x, p=p: sum([p[0]*5*x**4, p[1]*4*x**3, \
            p[2]*3*x**2, p[3]*2*x, p[4]]))                
            traj_dd.append(lambda x, p=p: sum([p[0]*20*x**3, p[1]*12*x**2, \
            p[2]*6*x, p[3]*2]))
            
        return [traj, traj_d, traj_dd]
    
def Estimate_J_ob(x_a, x_b):
    x_o = (x_a + x_b)/2
    r_o_b = x_o - x_b
    return np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])
    

############### System Dynamic Functions #############
######################################################
######################################################
############### Controllers ##########################


        
def InverseDynamics_ObjectModelFree(q, dq, X_o, dX_o, X_des, Xdot_des, Xddot_des, \
                    ua_pre, ub_pre, qdd_pre, M_pre, h_pre, J_b_pre, Q_pre, R_pre,\
                    Lambda_a_pre, winv_pre,
                    W = None, tau_0 = None, kp = None, kd = None):
    
    q_a = q[:3]
    q_b = q[3:6]
    dq_a = dq[:3]
    dq_b = dq[3:6]
    
    qdd_pre = qdd_pre[:6]

    JJ_a = CalcJ(model_a, q_a, tip_a)
    JJ_b = CalcJ(model_b, q_b, tip_b)
    J_a = np.hstack((JJ_a, np.zeros((3, 3))))
    J_b = np.hstack((np.zeros((3, 3)), JJ_b))
    save.J_b = J_b
    
    dJ_adq_a = CalcdJdq(model_a, q_a, dq_a, tip_a)
    dJ_bdq_b = CalcdJdq(model_b, q_b, dq_b, tip_b)
    
    x_a = CalcBody2Base(model_a, q_a, tip_a)
    x_b = CalcBody2Base(model_b, q_b, tip_b)
    
#    x_o = X_o[:3]
    r_o_a = X_o - x_a
    r_o_b = X_o - x_b
        
    J_oa = np.array([[1, 0, r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
    J_ob = np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])
    
#    print J_ob
        
    #Calculating P based on qr decomposition
    J_rel = J_a - np.dot(J_oa, np.dot(inv(J_ob), J_b))
        
    # Plan Joint-space trajectories
    q_des, qdot_des, qddot_des, xddot_des_after, dJ_obdz_o = \
    Task2Joint(J_a, J_b, J_oa, J_ob, J_rel, dJ_adq_a, dJ_bdq_b, r_o_a, r_o_b, \
               X_o, dX_o, X_des, Xdot_des, Xddot_des)
    
    save.q_des, save.qdot_des, save.qddot_des = \
    q_des, qdot_des, qddot_des
    
        
    M_a = CalcM(model_a, q_a)
    M_b = CalcM(model_b, q_b)    
    h_a = Calch(model_a, q_a, dq_a)
    h_b = Calch(model_b, q_b, dq_b)
        
    M = np.zeros((6, 6))
    M[:3, :3] = M_a
    M[3:, 3:] = M_b
    
    save.M = M

    h = np.vstack((h_a.reshape(3, 1), h_b.reshape(3, 1))).flatten()
    
    save.h = h
    

#    Mzh = np.dot(M_o, xddot_des_after) + h_o.flatten()
    Mqh = np.dot(M, qddot_des) + h 
#    Mqh += np.dot(J_b.T, np.dot(inv(J_ob.T), Mzh))

    dJ_bdq_b_des = dJ_bdq_b
    dJ_obdz_o_des = dJ_obdz_o
    
#    kkp = 200
#    kkd = kkp/5
#    qddot_des_bar = qddot_des + kkp*(q_des - q[:6]) + kkd*(qdot_des - dq[:6])
    qddot_des_bar = qddot_des
    
    M_bar = M + np.dot(J_b.T, np.dot(inv(J_ob.T), \
                                     np.dot(M_o, np.dot(inv(J_ob), J_b))))
    C_barq_des = np.dot(J_b.T, np.dot(inv(J_ob.T), np.dot(M_o, np.dot(inv(J_ob), \
            dJ_bdq_b_des - dJ_obdz_o_des))))
    h_bar = h.flatten() + np.dot(J_b.T, np.dot(inv(J_ob.T), h_o)).flatten()
    
    Mqh_bar_des = np.dot(M_bar, qddot_des_bar) + C_barq_des + h_bar
#    print np.dot(M_bar, qddot_des)
#    print h_bar
    
    
    S = np.eye(6)
    
    k, n = J_rel.shape
    
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
#    Su = np.hstack((np.zeros((n, n)), np.eye(n)))
    P, qr_Q, qr_R = CalcPqr(J_rel, Su)
    save.QR = [qr_Q, qr_R]
#    print P.shape
#    P = np.eye(6) - np.dot(pinv(J_rel), J_rel)
    
    
#    aux_here = inv(np.dot(J_rel, np.dot(inv(M_bar), J_rel.T)))
#    P = np.eye(6) - np.dot(J_rel.T, np.dot(aux_here, np.dot(J_rel, inv(M_bar))))
#
#    P = np.dot(inv(M_bar), P)
    
    
    if W is None: W = np.eye(S.shape[0])
    if tau_0 is None: tau_0 = np.zeros(S.shape[0])
    if kp is None: kp = 0
    if kd is None: kd = 0
    
    invw = inv(W)
    
                
    if True:       
        aux1 = np.dot(invw, np.dot(S, P.T))
        aux21 = np.dot(P, np.dot(S.T, invw))
        aux22 = np.dot(S, P.T)
        aux2 = np.dot(aux21, aux22)
        winv = np.dot(aux1, pinv(aux2))
    else:
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
        aux = pinv(np.dot(P, np.dot(S.T, w_m_s)))
        winv = np.dot(w_m_s, aux)
    
    save.winv = winv
    
#    aux3 = np.dot(np.eye(S.shape[0]) - np.dot(winv, np.dot(P, S.T)), invw)
    
#    invdyn = np.dot(winv, np.dot(P, Mqh_bar_des)).flatten() + np.dot(aux3, tau_0)
    
#    invdyn = np.dot(pinv(P), np.dot(P, Mqh))
    
    
#    invdyn = np.dot(winv, np.dot(P, Mqh)).flatten()
#    invdyn = np.dot(pinv(np.dot(Su, np.dot(qr_Q.T, S.T))), \
#                    np.dot(Su, np.dot(qr_Q.T, Mqh)))
    P_pre = np.dot(Su, Q_pre.T)
    tau_pre = np.concatenate((ua_pre, ub_pre))
    J_ob_approx = Estimate_J_ob(x_a, x_b)
    P1_bar = np.dot(inv(J_ob_approx.T), np.dot(M_o_approx, inv(J_ob_approx)))
    aux10 = M + np.dot(J_b.T, np.dot(P1_bar, J_b))
    aux10_pre = M_pre + np.dot(J_b_pre.T, np.dot(P1_bar, J_b_pre))
    
#    print '\n==================' , aux10 - aux10_pre
    
#    tde_u = np.dot(winv, np.dot(P, np.dot(aux10, qddot_des) + h) \
#                   + np.dot(P_pre, np.dot(S.T, tau_pre) - \
#                            np.dot(aux10_pre, qdd_pre) + h_pre))
    
#    tde_u = np.dot(winv, np.dot(P, np.dot(aux10, qddot_des - qdd_pre) + h - h_pre)) + tau_pre
    
    tde_u = np.dot(winv, np.dot(P, np.dot(aux10, qddot_des) + h)) + \
    tau_pre - np.dot(winv_pre, np.dot(P_pre, np.dot(aux10_pre, qdd_pre) + h_pre))

    
#    tde_u = np.dot(winv, np.dot(P, np.dot(aux10, qddot_des) + h)) + tau_pre -\
#    np.dot(pinv(P_pre), np.dot(P_pre, np.dot(aux10_pre, qdd_pre) + h_pre))
    
#    aux_a = np.dot(inv(qr_R), np.dot(Sc, qr_Q.T))
#    Lambda_a = Lambda_a_pre + np.dot(aux_a, \
#                                     np.dot(aux10, qddot_des - qdd_pre) + h - h_pre\
#                                     - np.dot(S.T, tde_u) + np.dot(S.T, tau_pre))
#    
#    aux_a_pre = np.dot(inv(R_pre), np.dot(Sc, Q_pre.T))
#    Lambda_a = Lambda_a_pre + np.dot(aux_a, \
#                                     np.dot(aux10, qddot_des) + h\
#                                     - np.dot(S.T, tde_u)) - \
#                np.dot(aux_a_pre, \
#                                     np.dot(aux10_pre, qdd_pre) + h_pre\
#                                     - np.dot(S.T, tau_pre))
    
    J = np.vstack((J_a, J_b))
    Lambda = np.dot(inv(J.T), Mqh - np.dot(S.T, tde_u))
#    print mrank(J)
    
    
    
#    Lambda_b = Lambda_b_pre 

    
    
#    
##    Calc interaction force
#    Lambda_a = np.dot(inv(qr_R), np.dot(Sc, \
#                      np.dot(qr_Q.T, Mqh_bar_des - np.dot(S.T, invdyn)))) 
    save.lambda_a_p = Lambda[:3]
    
    
    
    
#    Lambda_a_2 = - np.dot(inv(qr_R), np.dot(Sc, np.dot(\
#                          np.eye(6) - np.dot(qr_Q.T, np.dot(winv, P)), np.dot(S.T, invdyn))))
#    
#    print Lambda_a - Lambda_a_2
#    print 'lambda from qr:', Lambda_a

    
    return np.dot(S.T, tde_u) + \
    kp * (q_des - q) + kd * (qdot_des - dq), \
    q_des, qdot_des
    
    
def QP(q, dq, X_o, dX_o, X_des, Xdot_des, Xddot_des):
    q_a = q[:3]
    q_b = q[3:6]
    dq_a = dq[:3]
    dq_b = dq[3:6]

    JJ_a = CalcJ(model_a, q_a, tip_a)
    JJ_b = CalcJ(model_b, q_b, tip_b)
    J_a = np.hstack((JJ_a, np.zeros((3, 3))))
    J_b = np.hstack((np.zeros((3, 3)), JJ_b))
    
    dJ_adq_a = CalcdJdq(model_a, q_a, dq_a, tip_a)
    dJ_bdq_b = CalcdJdq(model_b, q_b, dq_b, tip_b)
    
    x_a = CalcBody2Base(model_a, q_a, tip_a)
    x_b = CalcBody2Base(model_b, q_b, tip_b)
    
#    x_o = X_o[:3]
    r_o_a = X_o - x_a
    r_o_b = X_o - x_b
        
    J_oa = np.array([[1, 0, r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
    J_ob = np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])
        
    #Calculating P based on qr decomposition
    J_rel = J_a - np.dot(J_oa, np.dot(inv(J_ob), J_b))
    
        
    # Plan Joint-space trajectories
    q_des, qdot_des, qddot_des, xddot_des_after, dJ_obdz_o = \
    Task2Joint(J_a, J_b, J_oa, J_ob, J_rel, dJ_adq_a, dJ_bdq_b, r_o_a, r_o_b, \
               X_o, dX_o, X_des, Xdot_des, Xddot_des)
    
    save.q_des, save.qdot_des, save.qddot_des = \
    q_des, qdot_des, qddot_des
    
    M_a = CalcM(model_a, q_a)
    M_b = CalcM(model_b, q_b)    
    h_a = Calch(model_a, q_a, dq_a)
    h_b = Calch(model_b, q_b, dq_b)
        
    M = np.zeros((6, 6))
    M[:3, :3] = M_a
    M[3:, 3:] = M_b

    h = np.vstack((h_a.reshape(3, 1), h_b.reshape(3, 1))).flatten()
    

#    Mzh = np.dot(M_o, xddot_des_after) + h_o.flatten()
#    Mqh = np.dot(M, qddot_des) + h 
#    Mqh += np.dot(J_b.T, np.dot(inv(J_ob.T), Mzh))

    dJ_bdq_b_des = dJ_bdq_b
    dJ_obdz_o_des = dJ_obdz_o
    
#    kkp = 200
#    kkd = kkp/5
#    qddot_des_bar = qddot_des + kkp*(q_des - q[:6]) + kkd*(qdot_des - dq[:6])
    qddot_des_bar = qddot_des
    
    M_bar = M + np.dot(J_b.T, np.dot(inv(J_ob.T), \
                                     np.dot(M_o, np.dot(inv(J_ob), J_b))))
    C_barq_des = np.dot(J_b.T, np.dot(inv(J_ob.T), np.dot(M_o, np.dot(inv(J_ob), \
            dJ_bdq_b_des - dJ_obdz_o_des))))
    h_bar = h.flatten() + np.dot(J_b.T, np.dot(inv(J_ob.T), h_o)).flatten()
    
    Mqh_bar_des = np.dot(M_bar, qddot_des_bar) + C_barq_des + h_bar
#    print np.dot(M_bar, qddot_des)
#    print h_bar
    
    
    S = np.eye(6)
    
#    Su = np.hstack((np.zeros((n, n)), np.eye(n)))
    P_qr, qr_Q, qr_R = CalcPqr(J_rel, Su)
    
    n_m_k, n = P_qr.shape
    k = n - n_m_k
        
 
    ### cost function 
    W_t = np.eye(n)
    W_t[3, 3] = 1e3
    W_c = np.eye(n)*0
#    W_c[4, 4] = 1e-2
    
    b_t_tpose = np.zeros((1, n))
    b_c_tpose = np.zeros((1, n))
    
    ### constraints on torque A \tau <= a
    
    A_t, a_t = np.array([]), np.array([])    
    
    lim1 = 2.
    lim2 = 10.
    n_row = 2
    A_t = np.zeros((n_row, n))
    A_t[0, 0] = 1.
    A_t[1, 0] = -1.
    a_t = np.array([lim1, lim2], dtype=float).reshape(n_row, 1)
    
    ### constraints on \lambda_a: A_la \lambda_a <= a_la
#    A_lb, l_lb = np.array([]), np.array([]) 
    lim1 = 5.
    lim2 = 3.
    A_la = np.zeros((2, k))
    A_la[0, 0] = -1.
    A_la[1, 1] = 1.
    a_la = np.array([lim1, lim2], dtype=float).reshape(2, 1)
        
    ### constraints on \lambda_b: A_lb \lambda_b <= a_lb
#    A_lb, a_lb = np.array([]), np.array([]) 
    lim1 = 0.
    lim2 = 20.
    A_lb = np.zeros((2, k))
    A_lb[0, 0] = -1.
    A_lb[1, 1] = -1.
    a_lb = np.array([lim1, lim2], dtype=float).reshape(2, 1)
    
    
    
    ### Preparation for the QP:
    if A_la.any():
        A_la_hat = - np.dot(A_la, np.dot(inv(qr_R), np.dot(Sc, \
                                         np.dot(qr_Q.T, S.T))))
        a_la_hat = a_la - np.dot(A_la, np.dot(inv(qr_R), \
                   np.dot(Sc, np.dot(qr_Q.T, Mqh_bar_des)))).reshape(a_la.shape)
        
    if A_lb.any():
        A_lb_hat = np.dot(A_lb, np.dot(inv(J_ob.T),\
                   np.dot(J_oa.T, np.dot(inv(qr_R), \
                   np.dot(Sc, np.dot(qr_Q.T, S.T))))))
        aux1 = np.dot(J_oa.T, np.dot(inv(qr_R), np.dot(Sc, np.dot(qr_Q.T, Mqh_bar_des))))
        aux2 = np.dot(M_o, np.dot(inv(J_ob), np.dot(J_b, qddot_des_bar))) + h_o.flatten()
        aux3 = np.dot(M_o, np.dot(inv(J_ob), dJ_bdq_b - dJ_obdz_o))
        a_lb_hat = a_lb + np.dot(A_lb, \
                   np.dot(inv(J_ob.T), aux1 + aux2 + aux3)).reshape(a_lb.shape)
        
    
    W_hat = W_t + np.dot(S, np.dot(W_c, S.T))
    b_tpose_hat = b_t_tpose - np.dot((np.dot(M, qddot_des) + h).T, \
                                     np.dot(W_c, S.T)) - np.dot(b_c_tpose, S.T)
    
    if not A_t.any() and not A_la.any() and not A_lb.any():
        G = None; hc = None
    elif A_t.any():
        GG = A_t; hhc = a_t
        if A_la.any():
            GG = np.vstack((GG, A_la_hat))
            hhc = np.vstack((hhc, a_la_hat))
        if A_lb.any():
            GG = np.vstack((GG, A_lb_hat))
            hhc = np.vstack((hhc, a_lb_hat))
        G = matrix(GG)
        hc = matrix(hhc)
    elif A_la.any():
        GG = A_la_hat; hhc = a_la_hat
        if A_lb.any():
            GG = np.vstack((GG, A_lb_hat))
            hhc = np.vstack((hhc, a_lb_hat))
        G = matrix(GG)
        hc = matrix(hhc)
    else:
        GG = A_lb_hat; hhc = a_lb_hat
        G = matrix(GG)
        hc = matrix(hhc)
        
    AA = np.dot(P_qr, S.T)
    bb = np.dot(P_qr, Mqh_bar_des).reshape(n_m_k, 1)
    
    P = matrix(W_hat)
    q = matrix(b_tpose_hat.T)    
    A = matrix(AA)
    b = matrix(bb)
    
    sol=solvers.qp(P, q, A=A, b=b, G=G, h=hc)
    
    res = np.array(sol['x']).flatten()
    
#    if sol['status'] is not 'optimal' or ((np.dot(G, res) - hc) > 0).any(): 
#        raise ValueError('no optimal solution!')
    
    return np.dot(S.T, res), q_des, qdot_des


##################### Controllers-End ################
######################################################
######################################################
######################################################
###################### Constants and Desired #########
 
def MoI(m, d, w): return 1/12.*m*(d**2 + w**2)

#Lua Model Parameters:
#################
la1 = 0.4; lb1 = 0.15; la2 = 0.4; lb2 = 0.15; lo = .25; l0 = .5; w = .1; wo = .15
ma1 = 1.5; mb1 = 1.; ma2 = 1.5; mb2 = 1.; mo = 2.
ia1_xx = MoI(ma1, w, w); ia1_yy = MoI(ma1, la1, w); ia1_zz = MoI(ma1, la1, w)
ib1_xx = MoI(mb1, w, w); ib1_yy = MoI(mb1, lb1, w); ib1_zz = MoI(mb1, lb1, w)
ia2_xx = MoI(ma2, w, w); ia2_yy = MoI(ma2, la2, w); ia2_zz = MoI(ma2, la2, w) 
ib2_xx = MoI(mb2, w, w); ib2_yy = MoI(mb2, lb2, w); ib2_zz = MoI(mb2, lb2, w)
io_xx = MoI(mo, w, wo); io_yy = MoI(mo, l0, wo); io_zz = MoI(mo, lo, wo)
#################
t_end = 4.5


#Object params
g0 = 9.81*0
M_o = np.eye(3)*mo
M_o[2, 2] = io_zz
h_o = np.array([[0], [mo*g0], [0]])
M_o_approx = np.eye(3)
M_o_approx[2, 2] = .1

#Sa = np.array([[1, 0, 0], [0, 1, 0]])
#Sth = np.array([[0, 0, 1]])


#model = rbdl.loadModel('./Model2D.lua')
model_a = rbdl.loadModel('./RR_a.lua')
model_b = rbdl.loadModel('./RR_b.lua')

tip_a = np.array([lb1, 0., 0.])
tip_b = np.array([lb2, 0., 0.])


x_start = np.array([0, 0.9, 0])
x_end = np.array([-.5, .5, +np.pi/3])
z_start = [x_start, np.zeros(3), np.zeros(3)]
z_end = [x_end, np.zeros(3), np.zeros(3)]
x_des_t1, xdot_des_t1, xddot_des_t1 = traj_plan(0, t_end/3, z_start, z_end)

x_start = x_end.copy()
x_end = np.array([.5, .5, -np.pi/3])
z_start = [x_start, np.zeros(3), np.zeros(3)]
z_end = [x_end, np.zeros(3), np.zeros(3)]
x_des_t2, xdot_des_t2, xddot_des_t2 = traj_plan(0, t_end/3, z_start, z_end)

x_start = x_end.copy()
x_end = np.array([0, 0.9, 0])
z_start = [x_start, np.zeros(3), np.zeros(3)]
z_end = [x_end, np.zeros(3), np.zeros(3)]
x_des_t3, xdot_des_t3, xddot_des_t3 = traj_plan(0, t_end/3, z_start, z_end)

rlx = range(len(x_des_t1)) 

#x = [[d2r(0.), d2r(0.), d2r(0.), 0., 0., 0.]]
q = [[np.pi/3, np.pi/3, np.pi*(1/2 -1/3 - 1/3), \
      np.pi - np.pi/3, -np.pi/3, -np.pi*(1/2 -1/3 - 1/3), 0, 0.9, 0]]
dq = [[0.]*9]
ddq = [[0.]*9]

q_des = [q[-1][:6]]
dq_des = [[0.]*6]
ddq_des = [[0.]*6]


time = 0

dqctrl_des_prev = np.zeros(6) # TODO: we usually start from rest
qctrl_des_prev = q_des[-1][:6] 

M_0 = np.zeros((6, 6))
M_0[:3, :3] = CalcM(model_a, q[-1][:3])
M_0[3:, 3:] = CalcM(model_b, q[-1][3:6])

h_0 = np.zeros((6))
h_0[:3] = Calch(model_a, q[-1][:3], dq[-1][:3])
h_0[3:] = Calch(model_b, q[-1][3:6], dq[-1][3:6])


J_a_0 = np.hstack((CalcJ(model_a, q[-1][:3], tip_a), np.zeros((3, 3))))
J_b_0 = np.hstack((np.zeros((3, 3)), CalcJ(model_b, q[-1][3:6], tip_b)))

x_a = CalcBody2Base(model_a, q[-1][:3], tip_a)
x_b = CalcBody2Base(model_b, q[-1][3:6], tip_b)

r_o_a = q[-1][6:] - x_a
r_o_b = q[-1][6:] - x_b
    
J_oa_0 = np.array([[1, 0, r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
J_ob_0 = np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])

J_rel_0 = J_a_0 - np.dot(J_oa_0, np.dot(inv(J_ob_0), J_b_0))

k, n = J_rel_0.shape
    
Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

P_qr, Q_0, R_0 = CalcPqr(J_rel_0, Su)

S = np.eye(M_0.shape[0])
W = np.eye(S.shape[0])
invw = inv(W)
       
if True:       
    aux1 = np.dot(invw, np.dot(S, P_qr.T))
    aux21 = np.dot(P_qr, np.dot(S.T, invw))
    aux22 = np.dot(S, P_qr.T)
    aux2 = np.dot(aux21, aux22)
    winv_0 = np.dot(aux1, pinv(aux2))
else:
    w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
    aux = pinv(np.dot(P_qr, np.dot(S.T, w_m_s)))
    winv_0 = np.dot(w_m_s, aux)

M_list = [M_0]
h_list = [h_0]
J_b_list = [J_b_0]
QR_list = [[Q_0, R_0]]
winv_list = [winv_0]


t = [time]

tau_a = np.array([0, 0, 0])
tau_b = np.array([0, 0, 0])

u_a = [tau_a]
u_b = [tau_b]

F_a = [np.array([0, 0, 0])]
F_b = [np.zeros(3)]
       
F_a_p = [np.zeros(3)]

F_motion = [np.zeros(6)]
F_squeeze = [np.zeros(6)]

x_des = [q[-1][6:9]]
dx_des = [np.zeros(3)]
ddx_des = [np.zeros(3)]


dt = 0.005

def ChooseRef(time):
    if time <= t_end/3: 
        out = [x_des_t1, xdot_des_t1, xddot_des_t1, time]
    elif time <= 2*t_end/3: 
        out = [x_des_t2, xdot_des_t2, xddot_des_t2, time - t_end/3]
    else: 
        out = [x_des_t3, xdot_des_t3, xddot_des_t3 , time - t_end/3*2]
    return out


############### Constants and Desired ################
######################################################
######################################################
######################################################
############### Running Simulation ###################

while time < t_end:

    q_ctrl = q[-1][:6]
    dq_ctrl = dq[-1][:6]
    x_o = q[-1][6:9]
    dx_o = dq[-1][6:9]
    
    x_des_t, xdot_des_t, xddot_des_t, time_prime = ChooseRef(time)
    
    x_des_now = np.array([x_des_t[i](time_prime) for i in rlx]).flatten()
    xdot_des_now = np.array([xdot_des_t[i](time_prime) for i in rlx]).flatten()
    xddot_des_now = np.array([xddot_des_t[i](time_prime) for i in rlx]).flatten()
    
    x_des.append(x_des_now)
    dx_des.append(xdot_des_now)
    ddx_des.append(xddot_des_now)
    
    
#    u, qctrl_des_prev, dqctrl_des_prev = \
#    InverseDynamics(q_ctrl, dq_ctrl, x_o, dx_o, \
#                    x_des_now, xdot_des_now, xddot_des_now, \
#                    kp = 40*0, kd = 10*0)
    u, qctrl_des_prev, dqctrl_des_prev = \
    InverseDynamics_ObjectModelFree(q_ctrl, dq_ctrl, x_o, dx_o, \
                    x_des_now, xdot_des_now, xddot_des_now, \
                    u_a[-1], u_b[-1], np.array(ddq[-1]), M_list[-1], h_list[-1], \
                    J_b_list[-1], QR_list[-1][0], QR_list[-1][1], F_a_p[-1], \
                    winv_list[-1])
    
    u = np.array(u, dtype=float)
    tau_a, tau_b = u[:3], u[3:]  
        
    args = (tau_a, tau_b)
    res = odeint(dyn, np.concatenate((q[-1], dq[-1])),\
    [time, time + dt], args = args)
    
    
    
    
    for i in [0, 1, 2, 3, 4, 5, 8]:
        res[-1, i] = np.mod(res[-1, i], np.pi*2)
        if res[-1, i] > np.pi: res[-1, i] = res[-1, i] - np.pi*2
        
        
    plt.plot(time, x_des_now[0] - res[-1, 6], '*')
        
    
    q.append(res[-1, :9])
    dq.append(res[-1, 9:])
    u_a.append(tau_a)
    u_b.append(tau_b)
    F_a.append(save.lambda_a)
    F_b.append(save.lambda_b)
    ddq.append(save.qdd)
    q_des.append(save.q_des)
    dq_des.append(save.qdot_des)
    ddq_des.append(save.qddot_des)
    F_a_p.append(save.lambda_a_p)
    F_motion.append(save.lambda_motion)
    F_squeeze.append(save.lambda_squeeze)
    M_list.append(save.M)
    h_list.append(save.h)
    QR_list.append(save.QR)
    J_b_list.append(save.J_b)
    winv_list.append(save.winv)

    time += dt
    t.append(time)



############### Running Simulation ################
######################################################
######################################################
######################################################
############### Plot Results ###################


q = np.array(q, dtype=float) 
dq = np.array(dq, dtype=float)  
ddq = np.array(ddq, dtype=float)
q_des = np.array(q_des , dtype=float) 
dq_des  = np.array(dq_des , dtype=float)  
ddq_des  = np.array(ddq_des , dtype=float)
t = np.array(t)
u_a = np.array(u_a)
u_b = np.array(u_b)
F_a = np.array(F_a)
F_b = np.array(F_b)
F_a_p = np.array(F_a_p)
x_des = np.array(x_des)
dx_des = np.array(dx_des)
ddx_des = np.array(ddx_des)
F_motion = np.array(F_motion, dtype=float)
F_squeeze = np.array(F_squeeze, dtype=float)




robot_anim = Anim_bimanual([model_a, model_b], q, t, [tip_a, tip_b], l0)


#from validation import Validation

#for i in range(len(t)):
#    print i, Validation(q[i, :], dq[i, :], ddq[i, :], u_a[i, :], \
#                         u_b[i, :], F_a[i, :], F_b[i, :], \
#                       model_a, model_b, tip_a, tip_b, \
#                       M_o, I_o, h_o, CalcPqr, \
#                       q_des[i, :], dq_des[i, :], ddq_des[i, :])




#
#def dist(x, y): return np.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)
#
#xa = []
#xb = []
#for i in range(len(t)):
#    xa.append(CalcBody2Base(model_a, q[i, :3], tip_a))
#    xb.append(CalcBody2Base(model_b, q[i, 3:6], tip_b))
#
#xa = np.array(xa)
#xb = np.array(xb)
#x_a_o = xa - q[:, 6:9]
#x_b_o = xb - q[:, 6:9]
#
##plt.plot(t, np.sqrt((xa[:, 0] - xb[:, 0])**2 + (xa[:, 1] - xb[:, 1])**2))
#
#plt.plot(t, np.sqrt(x_a_o[:, 0]**2 + x_a_o[:, 1]**2))
#plt.plot(t, np.sqrt(x_b_o[:, 0]**2 + x_b_o[:, 1]**2))
#
#
#
#for i in range(len(t)):
#    J_a = CalcJ(model_a, q[i, :3], tip_a)
#    #JJ_b = CalcJ(model_b, q_b, tip_b)
##    J_a = np.hstack((JJ_a, np.zeros((2, 2))))
#    #J_b = np.hstack((np.zeros((2, 2)), JJ_b))
#    J_oa = np.array([[1, 0, - x_a_o[i, 1]], [0, 1, x_a_o[i, 0]], [0, 0, 1]])
#    vo = np.dot(pinv(J_oa), np.dot(J_a, dq[i, :3]))
#    
#    print np.dot(J_a, dq[i, :3]) - np.dot(J_oa, dq[i, 6:9])
##    print '>>>>', vo - dq[i, 4:7]
#    
#    plt.plot(t[i], vo[0], '*')
#    plt.plot(t[i], vo[1], '*')
#    plt.plot(t[i], vo[2], '*')
#
#plt.plot(t, dq[:, 6:9])



#x = np.array(x)
#F_ext = np.array(F_ext)


#plt.plot(t, q[:, 6])

#plt.figure()
#plt.subplot(321)
#plt.plot(t, r2d(q))
#plt.title('q')
#
#plt.subplot(322)
#plt.plot(t, dq)
#plt.title('dq')
#
#plt.subplot(323)
#plt.plot(t, x)
#plt.title('x')
#
#plt.subplot(324)
#plt.plot(t, F_ext)
#plt.title('F_ext')
#
#
#tau = np.array(u)
#plt.subplot(325)
#plt.plot(t, u)
#plt.title('tau')

#
###########################################################
###########################################################
################ RMS plot Internal Force ##################
#
#import numpy as np
#import matplotlib.pyplot as plt
#
#
#n_groups = 6
#
#rms_u = (20, 35, 30, 35, 27, 25)
##std_men = (2, 3, 4, 1, 2)
#
#rms_F = (25, 32, 34, 20, 25, 28)
##std_women = (3, 5, 2, 3, 3)
#
#fig, ax = plt.subplots()
#
#index = np.arange(n_groups)
#bar_width = 0.35
#
#opacity = 0.4
#error_config = {'ecolor': '0.3'}
#
#rects1 = plt.bar(index, rms_u, bar_width,
#                 alpha=opacity,
#                 color='b',
##                 yerr=std_men,
#                 error_kw=error_config,
#                 label='?? Minimized u')
#
#rects2 = plt.bar(index + bar_width, rms_F, bar_width,
#                 alpha=opacity,
#                 color='r',
##                 yerr=std_women,
#                 error_kw=error_config,
#                 label='?? Minimized F')
#
##plt.xlabel('Group')
#plt.ylabel('$\mathrm{RMS}\, (N\, \mathrm{or}\, N.m)$')
#plt.title('Generalized Internal Force by Grasping Points and Direction')
#plt.xticks(index + bar_width / 2, ('A', 'B', 'C', 'D', 'E', 'F'))
#plt.legend()
#
#plt.tight_layout()
#plt.show()



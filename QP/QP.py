#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 16:58:21 2017

@author: user
"""
import numpy as np
from numpy.linalg import inv, pinv
from cvxopt import solvers, matrix
import quadprog

    
def QP(q, dq, X_o, dX_o, X_des, Xdot_des, Xddot_des):
    q_a = q[:3]
    q_b = q[3:6]
    dq_a = dq[:3]
    dq_b = dq[3:6]

    JJ_a = CalcJ(model_a, q_a, tip_a)
    JJ_b = CalcJ(model_b, q_b, tip_b)
    J_a = np.hstack((JJ_a, np.zeros((3, 3))))
    J_b = np.hstack((np.zeros((3, 3)), JJ_b))
    
    save.J_b = J_b
    
    J = np.vstack((J_a, J_b))
    
    dJ_adq_a = CalcdJdq(model_a, q_a, dq_a, tip_a)
    dJ_bdq_b = CalcdJdq(model_b, q_b, dq_b, tip_b)
    
    x_a = CalcBody2Base(model_a, q_a, tip_a)
    x_b = CalcBody2Base(model_b, q_b, tip_b)
    
    save.x_a = x_a
    save.x_b = x_b
    
#    x_o = X_o[:3]
    r_o_a = X_o - x_a
    r_o_b = X_o - x_b
        
    G_oa = np.array([[1, 0, r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
    G_ob = np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])
        
    #Calculating P based on qr decomposition
    J_ga = J_a - np.dot(G_oa, np.dot(inv(G_ob), J_b))
    J_gb = J_b - np.dot(G_ob, np.dot(inv(G_oa), J_a))
    
    J_g = np.vstack((1/2*J_ga, 1/2*J_gb))
    
    J_g = J_ga
    k_a_only = 0
    
        
    # Plan Joint-space trajectories
    q_des, qdot_des, qddot_des, xddot_des_after, dG_oadz_o, dG_obdz_o = \
    Task2Joint(J_a, J_b, G_oa, G_ob, J_ga, dJ_adq_a, dJ_bdq_b, r_o_a, r_o_b, \
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
#    Mqh = np.dot(M, qddot_des) + h 
#    Mqh += np.dot(J_b.T, np.dot(inv(G_ob.T), Mzh))
    
    dJ_adq_a_des = dJ_adq_a
    dJ_bdq_b_des = dJ_bdq_b
    dG_oadz_o_des = dG_oadz_o
    dG_obdz_o_des = dG_obdz_o
    
#    kkp = 200
#    kkd = kkp/5
#    qddot_des_bar = qddot_des + kkp*(q_des - q[:6]) + kkd*(qdot_des - dq[:6])

    qddot_des_bar = qddot_des
    
    M_bar = M + np.dot(J_b.T, np.dot(inv(G_ob.T), \
                np.dot(M_o, np.dot(inv(G_ob), J_b))))
#              + k_a_only*1/2*np.dot(J_a.T, np.dot(inv(G_oa.T), \
#                np.dot(M_o, np.dot(inv(G_oa), J_a))))
    
    C_barq_des = np.dot(J_b.T, np.dot(inv(G_ob.T), np.dot(M_o, np.dot(inv(G_ob), \
                 dJ_bdq_b_des - dG_obdz_o_des))))
#                 k_a_only*1/2*np.dot(J_a.T, np.dot(inv(G_oa.T), np.dot(M_o, np.dot(inv(G_oa), \
#                 dJ_adq_a_des - dG_oadz_o_des))))

#    print J_a.T.shape, inv(G_oa.T).shape, h_o.shape, C_barq_des.shape
    
    h_bar = h.flatten() + np.dot(J_b.T, np.dot(inv(G_ob.T), h_o)).flatten()
#                        + k_a_only*1/2*np.dot(J_a.T, np.dot(inv(G_oa.T), h_o)).flatten()
    
    Mqh_bar_des = np.dot(M_bar, qddot_des_bar) + h_bar + C_barq_des
#    print np.dot(M_bar, qddot_des)
#    print h_bar
    
    
    S = np.eye(6)
    
    P_qr_g, Q_g, R_g, k_g, n_g, Sc_g, Su_g = qrparams(J_g)
    P_qr, Q, R, k, n, Sc, Su = qrparams(J)
    
    save.QR = [Q_g, R_g]
    
    
        
 
    ### cost function 
    W_t = np.eye(n)*1e4
    b_t_tpose = np.zeros((1, n))
    
    
    W_lambda = np.eye(k)
    
    if True:    
        W_lambda_a = np.diag([1, 1e1, 1e3])
    #    print Rotation(X_o[2])
        W_lambda_a = np.dot(Rotation(X_o[2]), np.dot(W_lambda_a, Rotation(X_o[2]).T))
        W_lambda_b = W_lambda_a
        W_lambda_b = np.dot(Rotation(X_o[2] + np.pi), np.dot(W_lambda_b, \
                            Rotation(X_o[2] + np.pi).T))    
        W_lambda = np.vstack((np.hstack((W_lambda_a, np.zeros((3, 3)))), \
                              np.hstack((np.zeros((3, 3)), W_lambda_b))))

    
#    print np.allclose(W_lambda.T, W_lambda)

    
    b_lambda = np.zeros((k, 1))
#    W_c[4, 4] = 1e-2
    W_c, b_c_tpose = ComputeW_candb_c(W_lambda, b_lambda, Q, R, k, n)
    
#    print W_c    
    
    ### constraints on torque A \tau <= a
    
    A_t, a_t = np.array([]), np.array([])    
    
#    lim1 = 2.
#    lim2 = 1.5
#    n_row = 2
#    A_t = np.zeros((n_row, n))
#    A_t[0, 0] = 1.
#    A_t[1, 1] = -1.
#    a_t = np.array([lim1, lim2], dtype=float).reshape(n_row, 1)
    
    ### constraints on \lambda_a: A_l \lambda <= a_l
    A_l, a_l = np.array([]), np.array([]) 
    
#    lim1 = -10.
#    n_row = 1
#    A_l = np.zeros((n_row, k))
#    A_l[0, 0] = -np.cos(X_o[2])
#    A_l[0, 1] = np.sin(X_o[2])
#    a_l = np.array([lim1], dtype=float).reshape(n_row, 1)
    

#    n_row = 8
#    C = LinearizeFCone(.6, n_row)
##    print C.shape
#    T  = np.array([0, 0, 1, 1, 0, 0, 0, 1, 0]).reshape(3, 3)
#    A_l = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2])))
#    a_l =  A_l[:, 2].reshape(n_row+1, 1)*mo*9.81
#    A_l = np.hstack((A_l[:, :2], np.zeros((n_row+1, 4))))
    
    
#    n_row = 8
#    C = LinearizeFCone(.6, n_row)
##    print C.shape
#    T  = np.array([0, 0, 1, 1, 0, 0, 0, 1, 0]).reshape(3, 3)
#    A_p = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2])))
#    A_pp = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2] + np.pi)))
#    a_l =  A_p[:, 2].reshape(n_row+1, 1)*mo*9.81
#    A_l = np.zeros((n_row+1, 6))
#    A_l[:, :2] = A_p[:, :2]
#    A_l[:, 3:5] = A_pp[:, :2]
    
#    Friction Cones:
    n_row = 4
    mu = .6
    C = LinearizeFCone(mu*.95, n_row)
    T  = np.array([0, 0, 1, 1, 0, 0, 0, 1, 0]).reshape(3, 3)
    
    A_a = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2])))
    A_b = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2] + np.pi)))
    
    a_l_a =  A_a[:, 2].reshape(n_row+1, 1)*mo*9.81
    a_l_b =  A_b[:, 2].reshape(n_row+1, 1)*mo*9.81
    
    A_l_a = np.zeros((n_row+1, 6))
    A_l_a[:, :2] = A_a[:, :2]
    
    A_l_b = np.zeros((n_row+1, 6))
    A_l_b[:, 3:5] = A_b[:, :2]  
    
#    Box Constraints on momentums:
    lim = 12.
    n_box = 4
    A_l_box = np.zeros((n_box, 6))
    A_l_box[0, 2] = 1. 
    A_l_box[1, 2] = -1.
    A_l_box[2, 2+3] = 1.
    A_l_box[3, 2+3] = -1.
    a_l_box = np.array([lim, lim, lim, lim], dtype=float).reshape(n_box, 1)
    
    
#    Stack all inequality constraints:
    a_l = np.vstack((a_l_a, a_l_b, a_l_box))
    A_l = np.vstack((A_l_a, A_l_b, A_l_box))
    


        
#    ### constraints on \lambda_b: A_lb \lambda_b <= a_lb
##    A_lb, a_lb = np.array([]), np.array([]) 
#    lim1 = 0.
#    lim2 = 20.
#    A_lb = np.zeros((2, k))
#    A_lb[0, 0] = -1.
#    A_lb[1, 1] = -1.
#    a_lb = np.array([lim1, lim2], dtype=float).reshape(2, 1)
    
    
    
    ### Preparation for the QP:
    if A_l.any():
#        print A_l.shape, R.shape, Sc.shape, Q.shape, S.shape
        A_l_hat = - np.dot(A_l, np.dot(inv(R), np.dot(Sc, \
                                         np.dot(Q.T, S.T))))
        a_l_hat = a_l - np.dot(A_l, np.dot(inv(R), \
                   np.dot(Sc, np.dot(Q.T, np.dot(M, qddot_des) + h)))).reshape(a_l.shape)
        
#    if A_lb.any():
#        A_lb_hat = np.dot(A_lb, np.dot(inv(G_ob.T),\
#                   np.dot(G_oa.T, np.dot(inv(R), \
#                   np.dot(Sc, np.dot(Q.T, S.T))))))
#        aux1 = np.dot(G_oa.T, np.dot(inv(R), np.dot(Sc, np.dot(Q.T, Mqh_bar_des))))
#        aux2 = np.dot(M_o, np.dot(inv(G_ob), np.dot(J_b, qddot_des_bar))) + h_o.flatten()
#        aux3 = np.dot(M_o, np.dot(inv(G_ob), dJ_bdq_b - dG_obdz_o))
#        a_lb_hat = a_lb + np.dot(A_lb, \
#                   np.dot(inv(G_ob.T), aux1 + aux2 + aux3)).reshape(a_lb.shape)
        
    
    W_hat = W_t + np.dot(S, np.dot(W_c, S.T))
    b_tpose_hat = b_t_tpose - np.dot((np.dot(M, qddot_des) + h).T, \
                                     np.dot(W_c, S.T)) - np.dot(b_c_tpose, S.T)
    
    if not A_t.any() and not A_l.any():
        G = None; hc = None
    elif A_t.any():
        GG = A_t; hhc = a_t
        if A_l.any():
            GG = np.vstack((GG, A_l_hat))
            hhc = np.vstack((hhc, a_l_hat))
#        if A_lb.any():
#            GG = np.vstack((GG, A_lb_hat))
#            hhc = np.vstack((hhc, a_lb_hat))
        G = matrix(GG)
        hc = matrix(hhc)
    elif A_l.any():
        GG = A_l_hat; hhc = a_l_hat
#        if A_lb.any():
#            GG = np.vstack((GG, A_lb_hat))
#            hhc = np.vstack((hhc, a_lb_hat))
        G = matrix(GG)
        hc = matrix(hhc)
#    else:
#        GG = A_lb_hat; hhc = a_lb_hat
#        G = matrix(GG)
#        hc = matrix(hhc)
        
    AA = np.dot(P_qr_g, S.T)
    if flag_omf:
        J_ob_approx = Estimate_J_ob(x_a, x_b)
        P1_bar = np.dot(inv(J_ob_approx.T), np.dot(M_o_approx, inv(J_ob_approx)))
        aux10 = M + np.dot(J_b.T, np.dot(P1_bar, J_b))
        
        
        P_qr_g_pre = np.dot(Su_g, QR_list[-1][0].T)
        tau_pre = np.concatenate((u_a[-1], u_b[-1]))
        J_ob_approx_pre = Estimate_J_ob(x_a_list[-1], x_b_list[-1])
        P1_bar_pre = np.dot(inv(J_ob_approx_pre.T), \
                            np.dot(M_o_approx, inv(J_ob_approx_pre)))
        aux10_pre = M_list[-1] + np.dot(J_b_list[-1].T, np.dot(P1_bar_pre, J_b_list[-1]))
        
        H = np.dot(P_qr_g_pre, np.dot(S.T, tau_pre) - \
                   np.dot(aux10_pre, np.array(ddq[-1][:6])) - h_list[-1])
        
        bb = np.array([np.dot(P_qr_g, np.dot(aux10, qddot_des_bar) + h)\
                       + H]).reshape(n_g - k_g, 1)
    else:
        bb = np.dot(P_qr_g, Mqh_bar_des).reshape(n_g - k_g, 1)
    
#    print P_qr_g

#    print AA.shape, bb.shape, GG.shape, hhc.shape

    if flag_quadprog:
        meq = AA.shape[0]
        C = - np.array(np.vstack((AA, GG)).T, dtype=np.double)
        b = - np.array(np.vstack((bb, hhc)).flatten(), dtype=np.double)
        G = W_hat
        a = - b_tpose_hat.flatten()
        
#        tic = etime.time()
        res, f, xu, iters, lagr, iact = quadprog.solve_qp(G, a, C, b, meq=meq)
#        print etime.time() - tic
#        print iters

        
    elif flag_cvxopt:
    
        P = matrix(W_hat)
        q = matrix(b_tpose_hat.T)    
        A = matrix(AA)
        b = matrix(bb)
        
#        tic = etime.time()
        solvers.options['show_progress'] = False
        sol=solvers.qp(P, q, A=A, b=b, G=G, h=hc)
        res = np.array(sol['x']).flatten()
#        print etime.time() - tic
    
#    if sol['status'] is not 'optimal' or ((np.dot(G, res) - hc) > 0).any(): 
#        raise ValueError('no optimal solution!')

    Lambda_a = np.dot(inv(R_g), np.dot(Sc_g, \
                      np.dot(Q_g.T, Mqh_bar_des - np.dot(S.T, res)))) 
    save.lambda_a_p = Lambda_a
    
    return np.dot(S.T, res), q_des, qdot_des

def qrparams(J):
    k, n = J.shape
    
#    print mrank(J)
    
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
#    Su = np.hstack((np.zeros((n, n)), np.eye(n)))
        
    P_qr, Q, R = CalcPqr(J, Su)
    
    n_m_k, n = P_qr.shape
    k = n - n_m_k
    return P_qr, Q, R, k, n, Sc, Su

def ComputeW_candb_c(W_lambda, b_lambda, Q, R, k, n):
    aux311 = np.dot(inv(R.T), np.dot(W_lambda, inv(R)))
    aux31 = np.hstack((aux311, np.zeros((k, n - k))))
    aux32 = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
    aux3 = np.vstack((aux31, aux32))
    W_c = np.dot(Q, np.dot(aux3, Q.T))
    b_c_tpose = np.zeros((1, n))
    aux1 = np.vstack((inv(R.T), np.zeros((n - k, k))))
    b_c_tpose = np.dot(Q, np.dot(aux1, b_lambda)).T
#    W_c = np.eye(n)*0
    return W_c, b_c_tpose


def aj(mu, N, j): return np.dot(FuncS(mu, N, j)[1], FuncS(mu, N, j+1)[2]) - \
np.dot(FuncS(mu, N, j)[2], FuncS(mu, N, j+1)[1]) 

def bj(mu, N, j): return np.dot(FuncS(mu, N, j)[2], FuncS(mu, N, j+1)[0]) - \
np.dot(FuncS(mu, N, j+1)[2], FuncS(mu, N, j)[0]) 

def cj(mu, N, j): return np.dot(FuncS(mu, N, j)[0], FuncS(mu, N, j+1)[1]) - \
np.dot(FuncS(mu, N, j+1)[0], FuncS(mu, N, j)[1]) 

def FuncS(mu, N, j): return [mu*np.cos(2*np.pi/N*j), mu*np.sin(2*np.pi/N*j), 1]

def LinearizeFCone(mu, N):
    C = np.zeros((N+1, 3))
    for i in range(N):
        C[i, :] = [aj(mu, N, i), bj(mu, N, i), cj(mu, N, i)]
    C[N, 2] = 1
    return C
        
    
    
    
        
    
    
    
    
    
    
    
    
    
    
    
    
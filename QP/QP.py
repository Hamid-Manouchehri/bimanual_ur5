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

    dJ_adq_a_des = dJ_adq_a
    dJ_bdq_b_des = dJ_bdq_b
    dG_oadz_o_des = dG_oadz_o
    dG_obdz_o_des = dG_obdz_o

    qddot_des_bar = qddot_des

    M_bar = M + np.dot(J_b.T, np.dot(inv(G_ob.T), \
                np.dot(M_o, np.dot(inv(G_ob), J_b))))
    C_barq_des = np.dot(J_b.T, np.dot(inv(G_ob.T), np.dot(M_o, np.dot(inv(G_ob), \
                 dJ_bdq_b_des - dG_obdz_o_des))))
    h_bar = h.flatten() + np.dot(J_b.T, np.dot(inv(G_ob.T), h_o)).flatten()

    Mqh_bar_des = np.dot(M_bar, qddot_des_bar) + h_bar + C_barq_des


    S = np.eye(6)

    P_qr_g, Q_g, R_g, k_g, n_g, Sc_g, Su_g = qrparams(J_g)
    P_qr, Q, R, k, n, Sc, Su = qrparams(J)
    # print(P_qr_g.shape)

    save.QR = [Q_g, R_g]

    ### cost function
    W_t = np.eye(n)*1e4
    b_t_tpose = np.zeros((1, n))


    W_lambda = np.eye(k)

    W_lambda_a = np.diag([1, 1e1, 1e3])
    W_lambda_a = np.dot(Rotation(X_o[2]), np.dot(W_lambda_a, Rotation(X_o[2]).T))
    W_lambda_b = W_lambda_a
    W_lambda_b = np.dot(Rotation(X_o[2] + np.pi), np.dot(W_lambda_b, \
                        Rotation(X_o[2] + np.pi).T))
    W_lambda = np.vstack((np.hstack((W_lambda_a, np.zeros((3, 3)))), \
                          np.hstack((np.zeros((3, 3)), W_lambda_b))))  # (6*6)

    # print(np.round(W_lambda, 3))
    b_lambda = np.zeros((k, 1))  # (6*1)
    W_c, b_c_tpose = ComputeW_candb_c(W_lambda, b_lambda, Q, R, k, n)  # equ(13): Righetti

    ### constraints on torque A \tau <= a

    A_t, a_t = np.array([]), np.array([])

    ### constraints on \lambda_a: A_l \lambda <= a_l
    A_l, a_l = np.array([]), np.array([])

#    Friction Cones:
    n_row = 4
    mu = .6
    C = LinearizeFCone(mu*.95, n_row) # (5*3)
    # print(C, '\n')
    T  = np.array([0, 0, 1, 1, 0, 0, 0, 1, 0]).reshape(3, 3)
    # print(C.dot(inv(T)))

    A_a = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2])))  # (5*3)
    A_b = -np.dot(C, np.dot(inv(T) , Rotation(X_o[2] + np.pi)))  # (5*3)
    # print(np.round(A_b, 3))  # third column constant: [mu, mu, -mu, -mu, 0]

    a_l_a =  A_a[:, 2].reshape(n_row+1, 1)*mo*9.81  # (5*1)
    a_l_b =  A_b[:, 2].reshape(n_row+1, 1)*mo*9.81  # (5*1)
    # print(np.round(a_l_i, 3))  # constant: [11.18, 11.18, -11.18, -11.18, 0]

    A_l_a = np.zeros((n_row+1, 6))
    A_l_a[:, :2] = A_a[:, :2]  # (5*6)
    # print(np.round(A_l_a, 3))

    A_l_b = np.zeros((n_row+1, 6))
    A_l_b[:, 3:5] = A_b[:, :2]  # (5*6)
    # print(np.round(A_l_b, 3))

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
    a_l = np.vstack((a_l_a, a_l_b))#, a_l_box))  # (14*1)
    A_l = np.vstack((A_l_a, A_l_b))#, A_l_box))  # (14*6)

    ### Preparation for the QP:
    if A_l.any():
        # print('in A_l')
        A_l_hat = - np.dot(A_l, np.dot(inv(R), np.dot(Sc, np.dot(Q.T, S.T)))) # left side equ(40): Righetti, (10*6)
        a_l_hat = a_l - np.dot(A_l, np.dot(inv(R), np.dot(Sc, np.dot(Q.T, np.dot(M, qddot_des) + h)))).reshape(a_l.shape)  # right side equ(40): Righetti, (10*1)

    # print('R:', R.shape)  # (6*6)
    # print('Sc:', Sc.shape)  # (6*6)
    # print('Q:', Q.shape)  # (6*6)
    # print('S:', S.shape)  # (6*6)

    # print('a_l:', a_l.shape)  # (10*1)
    # print('A_l:', A_l.shape)  # (10*6)
    # print('M:', M.shape)  # (6*6)


    W_hat = W_t + np.dot(S, np.dot(W_c, S.T))  # first term of equ(37) Righetti, (6*6)
    b_tpose_hat = b_t_tpose - np.dot((np.dot(M, qddot_des) + h).T, \
                                     np.dot(W_c, S.T)) - np.dot(b_c_tpose, S.T)  # second term of equ(37): Righetti, (1*6)

    # print('W_hat:', W_hat.shape)  # (6*6)
    # print('W_t:', W_t.shape)  # (6*6)
    # print('W_c:', M.shape)  # (6*6)
    # print('b_t_tpose:', b_t_tpose.shape)  # (1*6)
    # print('b_c_tpose:', b_c_tpose.shape)  # (1*6)


    if not A_t.any() and not A_l.any():
        G = None; hc = None

    elif A_t.any():
        # print('in A_t')
        GG = A_t; hhc = a_t
        if A_l.any():
            GG = np.vstack((GG, A_l_hat))
            hhc = np.vstack((hhc, a_l_hat))
        G = matrix(GG)
        hc = matrix(hhc)


    elif A_l.any(): ############################################################
        # print('in A_l')
        GG = A_l_hat  # (10*6)
        hhc = a_l_hat  # (10*1)


    AA = np.dot(P_qr_g, S.T)  # left side of equ(38): Righetti, (3*6)
    bb = np.dot(P_qr_g, Mqh_bar_des).reshape(n_g - k_g, 1)  # right side of equ(38): Righetti, (3*1)

    # print('P_qr_g:', P_qr_g.shape)  # (3*6)

    if flag_quadprog:
        meq = AA.shape[0]  # 3
        C = - np.array(np.vstack((AA, GG)).T, dtype=np.double)  # (6*13)
        b = - np.array(np.vstack((bb, hhc)).flatten(), dtype=np.double)  # (13*1)
        G = W_hat  # (6*6)
        a = - b_tpose_hat.flatten()  # (6*1)

        res, f, xu, iters, lagr, iact = quadprog.solve_qp(G, a, C, b, meq=meq)
        # print(res)  # (6*1)


    Lambda_a = np.dot(inv(R_g), np.dot(Sc_g, \
                      np.dot(Q_g.T, Mqh_bar_des - np.dot(S.T, res))))
    save.lambda_a_p = Lambda_a

    return np.dot(S.T, res), q_des, qdot_des



def qrparams(J):
    k, n = J.shape
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

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

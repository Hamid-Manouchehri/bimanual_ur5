#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 12:20:18 2017

@author: user
"""

######################################################
############### Controllers ##########################



def InverseDynamics(q, dq, X_o, dX_o, X_des, Xdot_des, Xddot_des, \
                    W = None, tau_0 = None, kp = None, kd = None):

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

#    print J_ob

    #Calculating P based on qr decomposition
    J_rel = J_a - np.dot(J_oa, np.dot(inv(J_ob), J_b))


    # Plan Joint-space trajectories
    q_des, qdot_des, qddot_des, xddot_des_after, dGoadz_o, dJ_obdz_o = \
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
#    print qr_Q
#    P = np.eye(6) - np.dot(pinv(J_rel), J_rel)


#    aux_here = inv(np.dot(J_rel, np.dot(inv(M_bar), J_rel.T)))
#    P = np.eye(6) - np.dot(J_rel.T, np.dot(aux_here, np.dot(J_rel, inv(M_bar))))
#
#    P = np.dot(inv(M_bar), P)


    if W is None: W = np.eye(S.shape[0])
#    W[0, 1] = 2
#    W[3, 1] = 20
    if tau_0 is None: tau_0 = np.zeros(S.shape[0])
    if kp is None: kp = 0
    if kd is None: kd = 0


    if min_lambda:
        W, tau_0 = IntForceParam_mine(J_a, J_b, J_oa, J_ob, S, Mqh, X_o[2])
        # print tau_0

    invw = inv(W)


    if False:
        aux1 = np.dot(invw, np.dot(S, P.T))
        aux21 = np.dot(P, np.dot(S.T, invw))
        aux22 = np.dot(S, P.T)
        aux2 = np.dot(aux21, aux22)
        winv = np.dot(aux1, pinv(aux2))
    else:
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
        aux = pinv(np.dot(P, np.dot(S.T, w_m_s)))
        winv = np.dot(w_m_s, aux)

    aux3 = np.dot(np.eye(S.shape[0]) - np.dot(winv, np.dot(P, S.T)), invw)

    invdyn = np.dot(winv, np.dot(P, Mqh_bar_des)).flatten() + np.dot(aux3, tau_0)

#    invdyn = np.dot(pinv(P), np.dot(P, Mqh))


#    invdyn = np.dot(winv, np.dot(P, Mqh)).flatten()
#    invdyn = np.dot(pinv(np.dot(Su, np.dot(qr_Q.T, S.T))), \
#                    np.dot(Su, np.dot(qr_Q.T, Mqh)))

#    Calc interaction force
    Lambda_a = np.dot(inv(qr_R), np.dot(Sc, \
                      np.dot(qr_Q.T, Mqh_bar_des - np.dot(S.T, invdyn))))
    save.lambda_a_p = Lambda_a

#    print mrank(J_rel)

#    J_prime = J_rel[:, [3,5,4]]
#    Lambda_a = np.dot(inv(J_prime.T), Mqh_bar_des[[3,5,4]] - np.dot(S.T, invdyn)[[3,5,4]])
#    save.lambda_a_p = Lambda_a

#    Lambda_a_2 = - np.dot(inv(qr_R), np.dot(Sc, np.dot(\
#                          np.eye(6) - np.dot(qr_Q.T, np.dot(winv, P)), np.dot(S.T, invdyn))))
#
#    print Lambda_a - Lambda_a_2
#    print 'lambda from qr:', Lambda_a


    return np.dot(S.T, invdyn) + 0*Mqh_bar_des + \
    kp * (q_des - q) + kd * (qdot_des - dq), \
    q_des, qdot_des


###################################################################
###################################################################
###################################################################

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
    dJ_oadz_o_des = dJ_oadz_o
    dJ_obdz_o_des = dJ_obdz_o
#    dth_o_des = q_des[3:6].sum()
#    dJ_obdz_o_des = np.cross([0, 0, dth_o_des], np.cross(r_o_b, [0, 0, dth_o_des]))
#    print 'Xddot_des_after!!!!!!!!'
    return q_des, qdot_des, qddot_des, Xddot_des_after, dJ_oadz_o_des, dJ_obdz_o_des


def IntForceParam(J_a, J_b, J_oa, J_ob, S, Mqh, W_lam_i = None):
    J = np.vstack((J_a, J_b))
    k, n = np.shape(J)
    J_o = np.vstack((J_oa, J_ob))
    Q, RR = qr(J.T)
    R = RR[:k, :k]
    if W_lam_i is None: W_lam_i = np.eye(R.shape[0])
#    aux1 = np.eye(k) - np.dot(J_o, pinv(J_o))
    N = np.eye(k) - np.dot(pinv(J_o.T), J_o.T)
#    W_lam = np.dot(aux1, np.dot(W_lam_i, aux2))
    W_lam = W_lam_i
    W_lam[1, 1] = 5
    aux311 = np.dot(inv(R.T), np.dot(W_lam, inv(R)))
    aux31 = np.hstack((aux311, np.zeros((k, n - k))))
    aux32 = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
    aux3 = np.vstack((aux31, aux32))
    Wc = np.dot(Q, np.dot(aux3, Q.T))
#    Wc = np.dot(N.T, np.dot(Wc, N))
    W = np.dot(S, np.dot(Wc, S.T))
    tau_0 = np.dot(S, np.dot(Wc, Mqh))
    return W, tau_0

def IntForceParam_mine(J_a, J_b, J_oa, J_ob, S, Mqh, theta, W_lam_i = None):
    J = np.vstack((J_a, J_b))
    k, n = np.shape(J)
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))

#    print J_oa
    Q, RR = qr(J.T)
    R = RR[:k, :k]
    if W_lam_i is None: W_lam_i = np.eye(R.shape[0])
#    W_lam_i[0, 0] = 100

#    J_o = np.vstack((J_oa, J_ob))
#    winv = 1/2*np.vstack((inv(J_oa.T), inv(J_ob.T)))
#    JJ = np.dot(winv, J_o.T)
#    N = np.eye(JJ.shape[0]) - JJ
##    N = np.eye(k) - np.dot(pinv(J_o.T), J_o.T)
#    W_lam_i = np.dot(N.T, np.dot(W_lam_i, N))

    if min_tangent:
        WW = np.diag([1., 10, 2])
        Ra = Rotation(theta).T
        Rb = Rotation(theta).T

        auxa = np.dot(Ra.T, np.dot(WW, Ra))
        auxb = np.dot(Rb.T, np.dot(WW, Rb))

#        print auxa

        W_lam_i[:3, :3] = auxa
        W_lam_i[3:, 3:] = auxb

    W_lam = W_lam_i
#    W_lam[1, 1] = 5
#    aux311 = np.dot(inv(R.T), np.dot(W_lam, inv(R)))
#    aux31 = np.hstack((aux311, np.zeros((k, n - k))))
#    aux32 = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
#    aux3 = np.vstack((aux31, aux32))
    Wc = np.dot(Q, np.dot(Sc.T, np.dot(inv(R.T), np.dot(W_lam, np.dot(inv(R), \
                                                        np.dot(Sc, Q.T))))))
#    Wc = np.dot(N.T, np.dot(Wc, N))
    W = np.dot(S, np.dot(Wc, S.T))
    tau_0 = np.dot(S, np.dot(Wc.T, Mqh))
    return W, tau_0


############### System Dynamic Functions #############
######################################################

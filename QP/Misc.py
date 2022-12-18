#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 16:52:52 2017

@author: user
"""

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

def Rotation(t):
    return np.array([np.cos(t), -np.sin(t), 0, np.sin(t), np.cos(t), 0, \
                     0, 0, 1]).reshape(3, 3)


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

def MoI(m, d, w): return 1/12.*m*(d**2 + w**2)

def ChooseRef(time):
    if time <= t_end/3:
        out = [x_des_t1, xdot_des_t1, xddot_des_t1, time]
    elif time <= 2*t_end/3:
        out = [x_des_t2, xdot_des_t2, xddot_des_t2, time - t_end/3]
    else:
        out = [x_des_t3, xdot_des_t3, xddot_des_t3 , time - t_end/3*2]
    return out

class GetLambda(object):
    lambda_a = 0
    lambda_b = 0
    qdd = 0
    q_des, qdot_des, qddot_des = 0, 0, 0
    lambda_a_p = 0

def Estimate_J_ob(x_a, x_b):
    x_o = (x_a + x_b)/2
    r_o_b = x_o - x_b
    return np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])

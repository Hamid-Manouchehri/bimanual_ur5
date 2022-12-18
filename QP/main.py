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
import quadprog

import sys
sys.path.append('/home/mshahbazi/projects/rbdl/build/python/')
import rbdl

from Utils import Anim_bimanual
from scipy.linalg import sqrtm
from numpy.linalg import matrix_rank as mrank
import time as etime

#### This is just to avoid warnings:
from Dyn_Matices import CalcM, Calch, CalcJ, CalcdJdq, CalcBody2Base
from FDyn import ForwardDynamics, dyn
from Misc import traj_plan, QRDecompose, CalcPqr, Rotation, Traj_Estimate,\
ChooseRef, MoI, GetLambda, Estimate_J_ob
from QP import QP
from IDyn import Task2Joint, IntForceParam_mine, InverseDynamics
####

execfile('Dyn_Matices.py')
execfile('FDyn.py')
execfile('Misc.py')
execfile('QP.py')
execfile('IDyn.py')

save = GetLambda()

flag_quadprog = True
flag_cvxopt = False
flag_omf = True #Object-Model-Free


######################################################
###################### Constants and Desired #########


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

min_tangent = False
min_lambda = False


#Object params
g0 = 9.81*0
M_o = np.eye(3)*mo
M_o[2, 2] = io_zz
h_o = np.array([[0], [mo*g0], [0]])

M_o_approx = np.eye(3)
M_o_approx[2, 2] = .1


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
#time_end = 2.

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

x_a_0 = CalcBody2Base(model_a, q[-1][:3], tip_a)
x_b_0 = CalcBody2Base(model_b, q[-1][3:6], tip_b)

r_o_a = q[-1][6:] - x_a_0
r_o_b = q[-1][6:] - x_b_0

J_oa_0 = np.array([[1, 0, r_o_a[1]], [0, 1, - r_o_a[0]], [0, 0, 1]])
J_ob_0 = np.array([[1, 0, r_o_b[1]], [0, 1, - r_o_b[0]], [0, 0, 1]])

J_rel_0 = J_a_0 - np.dot(J_oa_0, np.dot(inv(J_ob_0), J_b_0))

k, n = J_rel_0.shape

Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

P_qr, Q_0, R_0 = CalcPqr(J_rel_0, Su)

M_list = [M_0]
h_list = [h_0]
J_b_list = [J_b_0]
QR_list = [[Q_0, R_0]]
x_a_list = [x_a_0]
x_b_list = [x_b_0]



t = [time]

tau_a = np.array([0, 0, 0])
tau_b = np.array([0, 0, 0])

u_a = [tau_a]
u_b = [tau_b]

F_a = [np.zeros(3)]
F_b = [np.zeros(3)]

F_motion = [np.zeros(6)]
F_squeeze = [np.zeros(6)]

F_a_p = [np.zeros(3)]

x_des = [q[-1][6:9]]
dx_des = [np.zeros(3)]
ddx_des = [np.zeros(3)]


dt = 0.005


############### Constants and Desired ################
######################################################
######################################################
######################################################
############### Running Simulation ###################
while time < 1.5:

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
#    tic = etime.time()
    u, qctrl_des_prev, dqctrl_des_prev = \
    QP(q_ctrl, dq_ctrl, x_o, dx_o, \
                    x_des_now, xdot_des_now, xddot_des_now)
#    print etime.time() - tic

    u = np.array(u, dtype=float)
    tau_a, tau_b = u[:3], u[3:]

    args = (tau_a, tau_b)
    res = odeint(dyn, np.concatenate((q[-1], dq[-1])),\
    [time, time + dt], args = args)




    for i in [0, 1, 2, 3, 4, 5, 8]:
        res[-1, i] = np.mod(res[-1, i], np.pi*2)
        if res[-1, i] > np.pi: res[-1, i] = res[-1, i] - np.pi*2


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
    x_a_list.append(save.x_a)
    x_b_list.append(save.x_b)

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
F_a_p = np.array(F_a_p, dtype=float)
F_motion = np.array(F_motion, dtype=float)
F_squeeze = np.array(F_squeeze, dtype=float)
x_des = np.array(x_des)
dx_des = np.array(dx_des)
ddx_des = np.array(ddx_des)

F_a_rotated = np.array([np.dot(Rotation(q[i, 8]), F_a[i, :]) \
                        for i in range(F_a.shape[0])])
F_a_normal, F_a_tangent = F_a_rotated[:, 0], F_a_rotated[:, 1]
#F_a_tangent = np.array([np.dot(Rotation(q[i, 8]), F_a[i, :])[1] for i in range(F_a.shape[0])])

F_b_normal = np.array([np.dot(Rotation(q[i, 8] + np.pi), F_b[i, :])[0] for i in range(F_a.shape[0])])
F_b_tangent = np.array([np.dot(Rotation(q[i, 8] + np.pi), F_b[i, :])[1] for i in range(F_a.shape[0])])


#u = np.hstack((u_a, u_b))
#F = np.hstack((F_a, F_b))
#cost_tau = np.array([np.dot(u[i, :], u[i, :]) for i in range(len(u))])
#cost_f = np.array([np.dot(F[i, :], F[i, :]) for i in range(len(F))])
#plt.plot(t, cost_tau, t, cost_f, '--')

cost_f_a_x = np.array([np.dot(F_a[i, 0], F_a[i, 0]) for i in range(len(F_a))])
cost_f_a_y = np.array([np.dot(F_a[i, 1], F_a[i, 1]) for i in range(len(F_a))])
cost_f_a_th = np.array([np.dot(F_a[i,2], F_a[i, 2]) for i in range(len(F_a))])

#plt.plot(t, cost_f_a_x, t, cost_f_a_y, '--', t, cost_f_a_th, '-.')

cost_f_a_n = np.array([np.dot(F_a_normal[i], F_a_normal[i]) for i in range(len(F_a))])
cost_f_a_t = np.array([np.dot(F_a_tangent[i], F_a_tangent[i]) for i in range(len(F_a))])

#plt.plot(t, cost_f_a_n, t, cost_f_a_t, '--', t, cost_f_a_th, '-.')













#print CalcBody2Base(model_a, q[0, :3], tip_a)

#robot_anim = Anim_bimanual([model_a, model_b], q, t, [tip_a, tip_b], l0)


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

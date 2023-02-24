#!/usr/bin/env python3
"""
Created on Sat Aug 27 11:20:33 2022

@author: Hamid Manouchehri

- For specifying which arm is right(r) and which one is left(l), imagine
  yourself on top of the table and your front is toward positive x-axis.

"""
from __future__ import division
from numpy.linalg import inv, pinv, cond
from scipy.linalg import qr, sqrtm
from math import sin, cos, sqrt, atan2, asin, acos
from cvxopt import matrix
import quadprog
import rbdl
import os
import csv
import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from squaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from sys import path
path.insert(1, '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_description/config/')  # TODO: Insert dir of config folder of the package.
import config
os.chdir('/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_gazebo/scripts/')  # TODO: Insert dir of config folder of the package.
import rbdl_methods  # TODO: change the file name if necessary.
####

dt = 0.
time_gaz_pre = 0.
time_gaz = 0.
t_end = 15  # TODO
g0 = 9.81
writeHeaderOnceFlag = True  # Do not touch
onceExecModelFreeFlag = True  # Do not touch

finiteTimeSimFlag = True  # TODO: True: simulate to 't_end', Flase: Infinite time
minTangentFlag = True  # TODO
minConstraintForceFlag = False  # TODO: True: minimizing constraint forces, False: solve normal, equ (15)
quadProgFlag = False  # TODO: True: solve quadratic programming, False: solve normal

linkName_r = 'wrist_3_link_r'
linkName_l = 'wrist_3_link_l'

wrist_3_length = 0.0823  # based on 'ur5.urdf.xacro'
obj_length = .2174  # based on 'ur5.urdf.xacro'
objCOMinWrist3_r = obj_length / 2 + wrist_3_length
objCOMinWrist3_l = obj_length + wrist_3_length
poseOfObjCOMInWrist3Frame_r = np.array([0., objCOMinWrist3_r, 0.])  # (x, y, z)
poseOfObjCOMInWrist3Frame_l = np.array([0., objCOMinWrist3_l, 0.])
poseOfTipOfWrist3InWrist3Frame_r = np.array([0., wrist_3_length, 0.])  # (x, y, z)
poseOfTipOfWrist3InWrist3Frame_l = poseOfTipOfWrist3InWrist3Frame_r
wrist_3_mass = .1879  # based on 'ur5.urdf.xacro'
mg_wrist_3 = wrist_3_mass * g0
l_contact_to_com_obj_wrist3 = .0565  # based on 'ur5.urdf.xacro'

workspaceDoF = 6
singleArmDoF = 6
dualArmDoF = 2*singleArmDoF
qDotCurrent_pre_r = np.zeros(6)
qDotCurrent_pre_l = np.zeros(6)

qctrl_des_prev = np.zeros(dualArmDoF)  # initial angular position of joints; joints in home configuration have zero angles.
dqctrl_des_prev = np.zeros(dualArmDoF)  # initial angular velocity of joints; we usually start from rest
rightContactFTSensoryData = np.array([0.]*workspaceDoF)
leftWristFTSensoryData = np.array([0.]*workspaceDoF)

## 6d trajectory variables:
index = 0
traj_time = []
traj_linearPose, traj_linearVel, traj_linearAccel = [], [], []
traj_angularPoseQuat, traj_angularVel, traj_angularAccel = [], [], []

## PID gains:
k_p_pose = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]) * 10

k_o = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]]) * 5

kp_a_lin = 70  # for steady (home) configuration
kd_a_lin = kp_a_lin / 5

kp_a_ang = 35
kd_a_ang = kp_a_ang / 5


## dynamic specifications of object (in world frame):
obj_mass = .5  # TODO: according to 'ur5.urdf.xacro'
mg_obj = obj_mass * g0
M_o = np.eye(workspaceDoF)

## choose principle axis as coordinate system (fixed frame):
inertiaTensor = np.array([[.0021, 0, 0],
                          [0, .00021, 0],
                          [0, 0, .0021]])  # TODO: according to 'ur5.urdf.xacro'
M_o[:3, :3] = M_o[:3, :3]*obj_mass
M_o[3:, 3:] = inertiaTensor

## definition of vector of generalized Centripetal, Coriolis, and Gravity forces:
h_o = np.array([0., 0., mg_obj, 0., 0., 0.])

## define dependent directories and including files:
pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']
CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
CSVFileName = pathToCSVFile + CSVFileName_plot_data

pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']
upperBodyModelFileName_r = config.bimanual_ur5_dic['urdfModelName_r']
upperBodyModelFileName_l = config.bimanual_ur5_dic['urdfModelName_l']
loaded_model_r = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName_r)
loaded_model_l = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName_l)

pathToTrajData = config.bimanual_ur5_dic['trajDataFileDirectory']
trajDataFileName_bimanual = config.bimanual_ur5_dic['trajDataFileName_bimanual']
trajDataFile = pathToTrajData + trajDataFileName_bimanual

## create instances of publishers:
pub_shoulder_pan_r = rospy.Publisher('/shoulder_pan_controller_r/command',
                   Float64, queue_size=10)
pub_shoulder_lift_r = rospy.Publisher('/shoulder_lift_controller_r/command',
                    Float64, queue_size=10)
pub_elbow_r = rospy.Publisher('/elbow_controller_r/command',
            Float64, queue_size=10)
pub_wrist_1_r = rospy.Publisher('/wrist_1_controller_r/command',
              Float64, queue_size=10)
pub_wrist_2_r = rospy.Publisher('/wrist_2_controller_r/command',
              Float64, queue_size=10)
pub_wrist_3_r = rospy.Publisher('/wrist_3_controller_r/command',
              Float64, queue_size=10)

pub_shoulder_pan_l = rospy.Publisher('/shoulder_pan_controller_l/command',
                   Float64, queue_size=10)
pub_shoulder_lift_l = rospy.Publisher('/shoulder_lift_controller_l/command',
                    Float64, queue_size=10)
pub_elbow_l = rospy.Publisher('/elbow_controller_l/command',
            Float64, queue_size=10)
pub_wrist_1_l = rospy.Publisher('/wrist_1_controller_l/command',
              Float64, queue_size=10)
pub_wrist_2_l = rospy.Publisher('/wrist_2_controller_l/command',
              Float64, queue_size=10)
pub_wrist_3_l = rospy.Publisher('/wrist_3_controller_l/command',
              Float64, queue_size=10)


rospy.init_node('main_bimanual_node')


def TrajEstimate(qdd):
    """ Implement Euler Integration to resolve qDot and q """
    global dqctrl_des_prev, qctrl_des_prev

    Dt = dt
    qdot_des_now = dqctrl_des_prev + Dt*qdd  # numerical derivation
    dqctrl_des_prev = qdot_des_now

    q_des_now = qctrl_des_prev + Dt*qdot_des_now
    qctrl_des_prev = q_des_now

    q_des_now_filtered = []

    for i in q_des_now:
        i = np.mod(i, np.pi*2)
        if i > np.pi:
            i = i - np.pi*2
        q_des_now_filtered.append(i)

    return np.array(q_des_now_filtered), qdot_des_now


def PubTorqueToGazebo(torqueVec):
    """
    Publish torque commands to Gazebo.
    """
    pub_shoulder_pan_r.publish(torqueVec[0])
    pub_shoulder_lift_r.publish(torqueVec[1])
    pub_elbow_r.publish(torqueVec[2])
    pub_wrist_1_r.publish(torqueVec[3])
    pub_wrist_2_r.publish(torqueVec[4])
    pub_wrist_3_r.publish(torqueVec[5])

    pub_shoulder_pan_l.publish(torqueVec[6])
    pub_shoulder_lift_l.publish(torqueVec[7])
    pub_elbow_l.publish(torqueVec[8])
    pub_wrist_1_l.publish(torqueVec[9])
    pub_wrist_2_l.publish(torqueVec[10])
    pub_wrist_3_l.publish(torqueVec[11])


def CalcOrientationErrorInQuaternion(currentOrientationInQuat, desOrientationInQuat):
    """Calculate position error in Quaternion based on the paper."""

    wCurrent = currentOrientationInQuat[0]
    vCurrent = currentOrientationInQuat[1:]

    wDes = desOrientationInQuat[0]
    vDes = desOrientationInQuat[1:]

    e_o = np.dot(wCurrent, vDes) - np.dot(wDes, vCurrent) - np.cross(vDes, vCurrent)  # (1*3)

    return e_o  # (1*3)


def WriteToCSV(data, yAxisLabel=None,legendList=None, t=None):
    """
    Write data to the CSV file to have a live plot by reading the file ...
    simulataneously.
    Pass 'data' as a list of  data and 'legendList' as a list of string type,
    legend for each value of the plot.
    Note 1: 'legendList' and 't' are arbitrary arguments.
    Note 2: Call this method once in your program.
    """
    global yLabel, plotLegend, writeHeaderOnceFlag

    plotLegend = legendList
    yLabel = yAxisLabel

    if t is None:
        ## to set the time if it is necessary.
        t = time_gaz

    with open(CSVFileName, 'a', newline='') as file:
        writer = csv.writer(file)

        if writeHeaderOnceFlag is True and (legendList is not None or yAxisLabel is not None):
            ## Add header to the CSV file.
            writer.writerow(np.hstack(['time', yAxisLabel, legendList]))
            writeHeaderOnceFlag = False

        writer.writerow(np.hstack([t, data]))  # the first element is time var.


def CalcGeneralizedVel_ref(currentPose, desPose, desVel, e_o):

    P_dot_des = desVel[:3]
    omega_des = desVel[3:]

    P_dot_ref = P_dot_des + k_p_pose.dot(desPose - currentPose)  # (1*3)
    omega_ref = omega_des + k_o.dot(e_o)  # (1*3)

    X_dot_r = np.concatenate((P_dot_ref, omega_ref))

    return X_dot_r


def IntForceParam_mine(J_r, J_l, G_o_r, G_o_l, S, Mqh, orientationOfObjInQuat):
    """Minimizing constaint force, lambda_i:"""

    J_total = np.vstack((J_r, J_l))  # (12*12)
    k, n = np.shape(J_total)  # 12, 12
    S_c = np.hstack((np.eye(k), np.zeros((k, n - k))))

    Q_qr, R_qr = qr(J_total.T)  # (12*12), (12*12)
    W_lam = np.eye(R_qr.shape[0])  # I(12*12)

    if minTangentFlag is True:  # ??????????????????????????????

        WW_a = np.diag([1., 1., 10.])
        WW_b = np.diag([1., 1., 10.])

        Ra = QuatToRotMat(orientationOfObjInQuat).T
        Rb = QuatToRotMat(orientationOfObjInQuat).T

        auxa = Ra.T.dot(WW_a.dot(Ra))
        auxb = Rb.T.dot(WW_b.dot(Rb))

        W_lam[3:6, 3:6] = auxa
        W_lam[9:, 9:] = auxb

    ## below equ(15), (12*12):
    W_c = Q_qr.dot(S_c.T.dot(inv(R_qr.T).dot(W_lam.dot(inv(R_qr).dot(S_c.dot(Q_qr.T))))))

    ## equ(15):
    W = S.dot(W_c.dot(S.T))  # (12*12)
    tau_0 = S.dot(W_c.T.dot(Mqh))  # (1*12)

    return W, tau_0


def QRparam(J):
    k, n = J.shape

    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

    P_qr, Q, R = CalcPqr(J, Su)

    n_m_k, n = P_qr.shape
    k = n - n_m_k
    return P_qr, Q, R, k, n, Sc, Su


def ComputeW_c_and_b_c(W_lambda, b_lambda, Q, R, k, n):
    aux311 = inv(R.T).dot(W_lambda.dot(inv(R)))
    aux31 = np.hstack((aux311, np.zeros((k, n - k))))
    aux32 = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))
    aux3 = np.vstack((aux31, aux32))
    W_c = Q.dot(aux3.dot(Q.T))  # equ(13, 1): Righetti
    b_c_tpose = np.zeros((1, n))
    aux1 = np.vstack((inv(R.T), np.zeros((n - k, k))))
    b_c_tpose = Q.dot(aux1.dot(b_lambda)).T  # equ(13, 2): Righetti

    return W_c, b_c_tpose


def aj(mu, N, j):
    return np.dot(FuncS(mu, N, j)[1], FuncS(mu, N, j+1)[2]) - \
           np.dot(FuncS(mu, N, j)[2], FuncS(mu, N, j+1)[1])


def bj(mu, N, j):
    return np.dot(FuncS(mu, N, j)[2], FuncS(mu, N, j+1)[0]) - \
           np.dot(FuncS(mu, N, j+1)[2], FuncS(mu, N, j)[0])


def cj(mu, N, j):
    return np.dot(FuncS(mu, N, j)[0], FuncS(mu, N, j+1)[1]) - \
           np.dot(FuncS(mu, N, j+1)[0], FuncS(mu, N, j)[1])


def FuncS(mu, N, j):
    return [mu*np.cos(2*np.pi/N*j), mu*np.sin(2*np.pi/N*j), 1]


def LinearizeFCone(mu, N):  # mu = .57, N = 4
    C = np.zeros((N+1, 3))  # (5*3)
    for i in range(N):  # 0, 1, 2, 3
        C[i, :] = [aj(mu, N, i), bj(mu, N, i), cj(mu, N, i)]
    C[N, 2] = 1

    return C


def QRDecompose(J):

    JT = J.T
    m, n = JT.shape

    if m == 0 or n == 0:
        raise TypeError(
            'Try to calculate QR decomposition, while there is no contact!')

    qr_Q, qr_R = qr(JT)
    qr_R = qr_R[:n, :]

    return qr_Q, qr_R


def CalcPqr(J, S_u):
    qr_Q, qr_R = QRDecompose(J)

    return np.dot(S_u, qr_Q.T), qr_Q, qr_R


def Estimate_G_ob(x_a, x_b):

    x_o = (x_a + x_b)/2
    x_o[1] = -.191
    # print(x_o, '\n')
    r_o_b = x_o - x_b
    G_ob = CalcG_oi(r_o_b)

    return G_ob


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



def InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent, qDes, qDotDes, qDDotDes,
                   J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l, Gdotdz_o_l,
                   J_g, z_o, rotationMatTip_r, rotationMatTip_l):

    global contactFTinWorld

    qCurrent_r = qCurrent[:singleArmDoF]
    qCurrent_l = qCurrent[singleArmDoF:]

    qDotCurrent_r = qDotCurrent[:singleArmDoF]
    qDotCurrent_l = qDotCurrent[singleArmDoF:]

    poseOfObjCOM, rotationMatOfObj = \
            rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r, qCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)

    M_r = rbdl_methods.CalcM(loaded_model_r, qCurrent_r)
    M_l = rbdl_methods.CalcM(loaded_model_l, qCurrent_l)

    M = np.zeros((dualArmDoF, dualArmDoF))
    M[:singleArmDoF, :singleArmDoF] = M_r
    M[singleArmDoF:, singleArmDoF:] = M_l

    h_r = rbdl_methods.CalcH(loaded_model_r, qCurrent_r, qDotCurrent_r)
    h_l = rbdl_methods.CalcH(loaded_model_l, qCurrent_l, qDotCurrent_l)
    h = np.zeros(dualArmDoF)
    h = np.concatenate((h_r, h_l))

    Mqh_des = M.dot(qDDotDes) + h

    M_hat = M + J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(J_l))))
    C_hat_q_des = J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))))
    h_hat = h + J_l.T.dot(inv(G_o_l.T).dot(h_o))
    Mqh_hat_des = M_hat.dot(qDDotDes) + C_hat_q_des + h_hat  # (12*1)

    k, n = J_g.shape  # (6, 12)
    S_u = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))  # below equ(11)
    S_c = np.hstack((np.eye(k), np.zeros((k, n - k))))
    P, qr_Q, qr_R = CalcPqr(J_g, S_u)

    S = np.eye(dualArmDoF)  # (12*12)

    if minConstraintForceFlag is True:
        W, tau_0 = IntForceParam_mine(J_r, J_l, G_o_r, G_o_l, S, Mqh_des, z_o[3:])

        ## W-weighted generalized inverse, below equ(10):
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)  # W^(-1/2)
        aux1 = pinv(P.dot(S.T.dot(w_m_s)))  # P*S.T*W^(-1/2), (12*6)
        winv = w_m_s.dot(aux1)

        aux2 = (np.eye(S.shape[0]) - winv.dot(P.dot(S.T))).dot(inv(W))  # null-space term of equ(10)

        ## equ(10): inverse dynamics
        tau = winv.dot(P.dot(Mqh_hat_des)) + aux2.dot(tau_0)

        # lambda_r = inv(qr_R).dot(S_c.dot(qr_Q.T.dot(Mqh_hat_des-S.T.dot(tau))))
        # print('analytical contact_r:', np.round(lambda_r, 3), '\n')

    elif quadProgFlag is True:

        P_qr_g, Q_g, R_g, k_g, n_g, Sc_g, Su_g = qrparams(J_g)
        jacTotal = np.vstack((J_r, J_l))  # (12*12)
        P_qr, Q, R, k, n, Sc, Su = qrparams(jacTotal)


        ## cost function (control parameters):
        W_tau = np.eye(n)  # quadratic cost on torques
        b_tau_tpose = np.zeros((1, n))  # linear cost on torques

        W_lambda = np.eye(12)  # quadratic cost on contact constraints
        b_lambda = np.zeros((k, 1))  # linear cost on contact constraints.

        ## constraints on torque commands: [T_shoulder, T_upper, T_forearm, T_w1, T_w2, T_w3]
        # tau_upper_r = np.array([10000., -36., 1000., 10000., 10000., 10000.])  # TODO
        # tau_downer_r = np.array([-10000., -42., -13., -10000., -10000., -10000.])  # TODO

        tau_upper_r = np.array([10000., -35., 1000., 10000., 10000., 10000.])  # TODO
        tau_downer_r = np.array([-10000., -40., -1000., -10000., -10000., -10000.])  # TODO

        tau_upper_l = np.array([10000., 1000., 10000., 10000., 10000., 10000.])  # TODO
        tau_downer_l = np.array([-10000., -1000., -10000., -10000., -10000., -10000.])  # TODO

        a_tau_inequal = np.vstack((tau_upper_r, tau_upper_l, -tau_downer_r, -tau_downer_l)).flatten()  # (24,)
        A_tau_inequal = np.vstack((np.eye(12), -np.eye(12)))  # (24*12)


        ## constraints on contact force-torque:
        n_row = 4  # linearized friction cone sides
        mu_s = .4
        C = LinearizeFCone(mu_s, n_row) # (5*3)
        T  = np.array([[1, 0, 0],
                       [0, 0, -1],
                       [0, 1, 0]])  # pi/2 around x-axis of world frame.

        A_a = -np.dot(C, np.dot(T, rotationMatTip_r))  # (5*3)
        A_b = -np.dot(C, np.dot(T, rotationMatTip_l))  # (5*3)

        a_l_a =  A_a[:, 2]*mg_obj  # (5*1)
        a_l_b =  A_b[:, 2]*mg_obj  # (5*1)

        A_l_a = np.zeros((n_row+1, 12))
        A_l_a[:, :2] = A_a[:, :2]  # (5*6)

        A_l_b = np.zeros((n_row+1, 12))
        A_l_b[:, 6:8] = A_b[:, :2]  # (5*6)

        b_lambda_inequal = np.vstack((a_l_a, a_l_b)).flatten()  # (10*1)
        B_lambda_inequal = np.vstack((A_l_a, A_l_b))  # (10*12)

        ## Preparation for the QP:
        A_l_hat = - B_lambda_inequal.dot(inv(R).dot(Sc.dot(Q.T.dot(S.T)))) # left side equ(40): Righetti, (10*12)
        a_l_hat = b_lambda_inequal - B_lambda_inequal.dot(inv(R).dot(Sc.dot(Q.T.dot(Mqh_des))))  # right side equ(40): Righetti, (10*10)

        W_c, b_c_tpose = ComputeW_candb_c(W_lambda, b_lambda, Q, R, k, n)  # equ(13): Righetti

        W_hat = W_tau + S.dot(W_c.dot(S.T))  # first term of equ(37) Righetti, (6*6)
        b_tpose_hat = b_tau_tpose - Mqh_des.T.dot(W_c.dot(S.T)) - b_c_tpose.dot(S.T)  # second term of equ(37): Righetti, (1*6)

        PST = P_qr_g.dot(S.T)  # left side of equ(38): Righetti, (6*12)
        PMqh_hat_des = P_qr_g.dot(Mqh_hat_des).flatten()  # right side of equ(38): Righetti, (6,)

        ## inequality constrainsts:
        C = - np.vstack([PST, A_tau_inequal, A_l_hat]).T
        b = - np.hstack([PMqh_hat_des, a_tau_inequal, a_l_hat]).flatten()
        # C = - np.vstack([PST]).T
        # b = - np.hstack([PMqh_hat_des]).flatten()

        ## cost function:
        G = W_hat  # (12*12)
        a = - b_tpose_hat.flatten()

        meq = PST.shape[0]
        tau, f, xu, iters, lagr, iact = quadprog.solve_qp(G, a, C, b, meq=meq)  # do not remove 'meq = meq'


    else: # solve normal:

        W = np.eye(S.shape[0])  # (12*12)
        tau_0 = np.zeros(S.shape[0])  # (1*12)

        ## W-weighted generalized inverse, below equ(10):
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)  # W^(-1/2)
        aux1 = pinv(P.dot(S.T.dot(w_m_s)))  # P*S.T*W^(-1/2), (12*6)
        winv = w_m_s.dot(aux1)

        aux2 = (np.eye(S.shape[0]) - winv.dot(P.dot(S.T))).dot(inv(W))  # null-space term of equ(10)

        ## equ(10): inverse dynamics
        tau = winv.dot(P.dot(Mqh_hat_des)) + aux2.dot(tau_0)

        # WriteToCSV(np.concatenate([contactFTinWorld[:3], lambda_r[:3]]), 'force_contact (N)', ['fx_sensor', 'fy_sensor', 'fz_sensor', 'fx_analytic', 'fy_analytic', 'fz_analytic'])

        # lambda_r = inv(qr_R).dot(S_c.dot(qr_Q.T.dot(Mqh_hat_des-S.T.dot(tau))))
        # WriteToCSV(lambda_r, 'lambda_r', ['F_x', 'F_y', 'F_z', 'T_roll', 'T_pitch', 'T_yaw'])
        # error = contactFTinWorld - lambda_r
        # WriteToCSV(error[:3], 'force_error (N)', ['e_x', 'e_y', 'e_z'])
        # WriteToCSV(error[3:], 'torque_error (Nm)', ['e_roll', 'e_pitch', 'e_yaw'])

    # WriteToCSV(tau[:6], 'right_torque (Nm)', ['T_shoulder', 'T_upper_arm', 'T_forearm', 'T_wrist_1', 'T_wrist_2', 'T_wrist_3'])
    # WriteToCSV(tau[6:], 'left_torque (Nm)', ['T_shoulder', 'T_upper_arm', 'T_forearm', 'T_wrist_1', 'T_wrist_2', 'T_wrist_3'])

    return tau


def qrparams(J):
    k, n = J.shape

    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

    P_qr, Q, R = CalcPqr(J, Su)

    n_m_k, n = P_qr.shape
    k = n - n_m_k
    return P_qr, Q, R, k, n, Sc, Su


def QuatToRotMat(unitQuatVec):
    """section 6.4 of 'attitude' paper:"""
    q0 = unitQuatVec[0]
    q1 = unitQuatVec[1]
    q2 = unitQuatVec[2]
    q3 = unitQuatVec[3]

    RotMat = \
        np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*q1*q2 + 2*q0*q3, 2*q1*q3 - 2*q0*q2],
                  [2*q1*q2 - 2*q0*q3, q0**2 - q1**2 + q2**2 - q3**2, 2*q2*q3 + 2*q0*q1],
                  [2*q1*q3 + 2*q0*q2, 2*q2*q3 - 2*q0*q1, q0**2 - q1**2 - q2**2 + q3**2]])

    return RotMat


def RotMatToQuaternion(R):
    """section 6.5 of 'attitude' paper:
    Note: rotation matrix in the paper is defined in the way that maps world
    into body-fixed frame, so there needs some modification...
    """

    # R = R.T  # Note ^
    r11, r12, r13 = R[0, :]
    r21, r22, r23 = R[1, :]
    r31, r32, r33 = R[2, :]

    if r22 > -r33 and r11 > -r22 and r11 > -r33:
        unitQuat = 1/2*np.array([sqrt(1 + r11 + r22 + r33),
                                (r23 - r32)/sqrt(1 + r11 + r22 + r33),
                                (r13 - r31)/sqrt(1 + r11 + r22 + r33),
                                (r12 - r21)/sqrt(1 + r11 + r22 + r33)])

    elif r22 < -r33 and r11 > r22 and r11 > r33:
        unitQuat = 1/2*np.array([(r23 - r32)/sqrt(1 + r11 - r22 - r33),
                                 sqrt(1 + r11 - r22 - r33),
                                 (r12 + r21)/sqrt(1 + r11 - r22 - r33),
                                 (r13 + r31)/sqrt(1 + r11 - r22 - r33)])

    elif r22 > r33 and r11 < r22 and r11 < -r33:
        unitQuat = 1/2*np.array([(r31 - r13)/sqrt(1 - r11 + r22 - r33),
                                 (r12 + r21)/sqrt(1 - r11 + r22 - r33),
                                 sqrt(1 - r11 + r22 - r33),
                                 (r23 + r32)/sqrt(1 - r11 + r22 - r33)])

    elif r22 < r33 and r11 < -r22 and r11 < r33:
        unitQuat = 1/2*np.array([(r12 - r21)/sqrt(1 - r11 - r22 + r33),
                                 (r13 + r31)/sqrt(1 - r11 - r22 + r33),
                                 (r23 + r32)/sqrt(1 - r11 - r22 + r33),
                                 sqrt(1 - r11 - r22 + r33)])

    else:
        print('there is something wrong with "RotMatToQuaternion" !')
        unitQuat = None

    return unitQuat  # (w, x, y, z)


def Calc_dGdz(v_o, r_o_i):

    currentOmega_o = v_o[3:]

    omegaCrossRoiCrossOmega = \
                    np.cross(currentOmega_o, np.cross(r_o_i, currentOmega_o))

    block_WRW = SkewSymMat(np.array(omegaCrossRoiCrossOmega))  # (3*3)

    block_63 = np.vstack((block_WRW, np.zeros((3, 3))))  # (6*3)

    blockZ_63 = np.vstack((np.zeros((3, 3)), np.zeros((3, 3))))  # (6*3)

    Gdot_oi = np.hstack((blockZ_63, block_63))  # (6*6)

    Gdot_oiV_o = Gdot_oi.dot(v_o)  # GDot_oa*qDot_o: (1*6)

    return Gdot_oiV_o  # (1*6)


def TaskToJoint(poseDesTraj, velDesTraj, accelDesTraj, r_o_r, r_o_l,
                dJdq_r, dJdq_l, G_o_r, G_o_l, J_g, J_r, J_l,
                currentGeneralizedPoseOfObjQuat, currentGeneralizedVelOfObj):

    currentPoseOfObj = currentGeneralizedPoseOfObjQuat[:3]
    currentOrientationOfObjInQuat = currentGeneralizedPoseOfObjQuat[3:]

    Gdotdz_o_r = Calc_dGdz(currentGeneralizedVelOfObj, r_o_r)  # (1*6)
    Gdotdz_o_l = Calc_dGdz(currentGeneralizedVelOfObj, r_o_l)  # (1*6)

    dJ_A_dq = dJdq_r - Gdotdz_o_r - \
            G_o_r.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))  # an element of equ(21): (1*6)
    dJ_B_dq = inv(G_o_l).dot(dJdq_l - Gdotdz_o_l)  # an element of equ(21): (1*6)

    translationalError = poseDesTraj[:3] - currentPoseOfObj  # (1*3)
    # WriteToCSV(translationalError, 'pose Error', ['e_x', 'e_y', 'e_z'])

    e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, poseDesTraj[3:])  # equ(4), orientation planning, (1*3)
    # WriteToCSV(e_o, 'orientation Error', ['e_r', 'e_p', 'e_y'])
    poseError = np.concatenate((translationalError, e_o))
    # WriteToCSV(poseError, 'Generalized Pose Error', ['e_x', 'e_y', 'e_z', 'e_roll', 'e_pitch', 'e_yaw'])

    xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning, (1*6)
    velError = xDot_ref - currentGeneralizedVelOfObj

    accelDesTraj_lin = accelDesTraj[:3] + kd_a_lin * velError[:3] + kp_a_lin * poseError[:3]  # a, below equ(21), (1*3)
    accelDesTraj_ang = accelDesTraj[3:] + kd_a_ang * velError[3:] + kp_a_ang * poseError[3:]  # a, below equ(21), (1*3)
    accelDesTraj = np.concatenate((accelDesTraj_lin, accelDesTraj_ang))
    # accelDesTraj = np.concatenate((accelDesTraj_lin, np.zeros(3)))

    J_B = inv(G_o_l).dot(J_l)  # (6*12)
    J_A = np.vstack((J_g, J_B))  # (12*12)
    dJ_A_dq = np.concatenate((dJ_A_dq, dJ_B_dq))  # (1*12)
    xDDotDes = np.concatenate((np.zeros(6), accelDesTraj))  # (1*12)
    qDDotDes = pinv(J_A).dot(xDDotDes - dJ_A_dq)  # equ(21): inverse kinematics, (1*12)

    qDes, qDotDes = TrajEstimate(qDDotDes)

    return qDes, qDotDes, qDDotDes, Gdotdz_o_l


def SkewSymMat(inputVector):
    """Compute the corresponding skew symetric matrix of 'inputVector' vector."""
    inputVector = np.array([inputVector])
    zeroMat = np.zeros((3, 3))
    zeroMat[0, 1] = -inputVector[0, 2]
    zeroMat[0, 2] = inputVector[0, 1]
    zeroMat[1, 2] = -inputVector[0, 0]
    zeroMat = zeroMat + (-zeroMat.T)

    return zeroMat  # (3*3)


def CalcG_oi(r_o_i):

    P_oi_cross = SkewSymMat(r_o_i)  # (3*3)
    G_oi_topRows = np.hstack((np.eye(3), P_oi_cross))
    G_oi_bottomRows = np.hstack((np.zeros((3, 3)), np.eye(3)))
    G_oi = np.vstack((G_oi_topRows, G_oi_bottomRows))  # (6*6)

    return G_oi


def SkewSymMat(inputVector):
    """Compute skew symetric matrix of 'inputList' vector."""
    inputVector = np.array([inputVector])
    zeroMat = np.zeros((3, 3))
    zeroMat[0, 1] = -inputVector[0, 2]
    zeroMat[0, 2] = inputVector[0, 1]
    zeroMat[1, 2] = -inputVector[0, 0]
    zeroMat = zeroMat + (-zeroMat.T)

    return zeroMat  # (3*3)


def MapFTContactToWorld(loaded_model, qCurrent, linkName, contactFTsensorData):
    """Project measured ft forces in the contact (right end-effector)."""

    global contactFTinWorld

    poseOfFTsensorInWrist3 = np.array([0., wrist_3_length, 0.])

    poseOfFT, rotationMatOfFT = \
                rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model, qCurrent,
                                                        linkName,
                                                        poseOfFTsensorInWrist3)

    A11 = rotationMatOfFT  # (3*3)
    A12 = np.zeros((3, 3))
    skewSymOfP_s_t = SkewSymMat(poseOfFT)  # (3*3)
    A21 = skewSymOfP_s_t.dot(rotationMatOfFT)  # (3*3)
    A22 = rotationMatOfFT

    firstRow = np.hstack((A11, A12))  # (3*6)
    secondRow = np.hstack((A21, A22))  # (3*6)
    T_s_t = np.vstack((firstRow, secondRow))  # (6*6)

    contactFTinWorld = T_s_t.dot(contactFTsensorData)  # (1*6)

    if linkName is linkName_r:
        # WriteToCSV(contactFTinWorld[:3], 'right_force_world (N)', ['F_x', 'F_y', 'F_z'])
        # print('FT in Wrist_r:', np.round(wristFTsensorData_r, 3))
        # print('FT in Contact_r:', np.round(contactFTsensorData, 3))
        # print('FT in World_r:', np.round(contactFTinWorld, 3), '\n')
        pass

    elif linkName is linkName_l:
        # WriteToCSV(contactFTinWorld[:3], 'left_force_world (N)', ['F_x', 'F_y', 'F_z'])
        # print('FT in Wrist_l:', np.round(wristFTsensorData_l, 3))
        # print('FT in Contact_l:', np.round(contactFTsensorData, 3))
        # print('FT in World_l:', np.round(contactFTinWorld, 3), '\n')
        pass


def acos_(value):
    """ to avoid value error 'math domain error' """
    if value > 1:
        return acos(1)

    elif value < -1:
        return acos(-1)

    else:
        return acos(value)


def FTwristSensorCallback_l(ft_data_l):
    """Sensor mounted in 'wrist_3_joint_l'. """
    global leftWristFTSensoryData, wristFTsensorData_l

    wrist_force = ft_data_l.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data_l.wrench.torque  # [T_x, T_y, T_z] local frame

    wristFTsensorData_l = \
                    np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                              wrist_torque.x, wrist_torque.y, wrist_torque.z])

    sensorAngleTo_x = acos_(wrist_force.x / (mg_wrist_3 + mg_obj/2))
    sensorAngleTo_y = acos_(wrist_force.y / (mg_wrist_3 + mg_obj/2))
    sensorAngleTo_z = acos_(wrist_force.z / (mg_wrist_3 + mg_obj/2))

    ## remove effect of 'wrist_3_link_l' mass for force:
    Fobj_x = wrist_force.x - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of 'wrist_3_link_l' torque:
    Tobj_x = wrist_torque.x - wrist_3_length*mg_obj*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z - wrist_3_length*mg_obj*cos(sensorAngleTo_x)

    leftWristFTSensoryData = np.array([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z])
    WriteToCSV(leftWristFTSensoryData[:3], 'left_force_sensor (N)', ['F_x', 'F_y', 'F_z'])


def FTwristSensorCallback_r(ft_data_r):
    """
    Sensor mounted in 'wrist_3_joint_r'.
    Measure generalized constraint forces, in wrist-sensor frame.
    """
    global rightContactFTSensoryData, wristFTsensorData_r

    wrist_force = ft_data_r.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data_r.wrench.torque  # [T_x, T_y, T_z] local frame

    wristFTsensorData_r = \
                    np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                              wrist_torque.x, wrist_torque.y, wrist_torque.z])

    sensorAngleTo_x = acos_(wrist_force.x / (mg_wrist_3 + mg_obj/2))
    sensorAngleTo_y = acos_(wrist_force.y / (mg_wrist_3 + mg_obj/2))
    sensorAngleTo_z = acos_(wrist_force.z / (mg_wrist_3 + mg_obj/2))

    ## remove effect of 'wrist_3_link_r' mass for force:
    Fobj_x = wrist_force.x# - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y# - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z# - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of 'wrist_3_link_r' torque:
    Tobj_x = wrist_torque.x# - wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_z) - l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z# + wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_x) + l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_x)

    rightContactFTSensoryData = np.array([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z])
    # WriteToCSV(rightContactFTSensoryData[:3], 'right_force_sensor (N)', ['F_x', 'F_y', 'F_z'])


def PositinoControl(q, qDot, qDDot):
    """ Control position conntrol for testing coordinations"""
    k_p = 700
    k_v = 40

    ## joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    qDes_r = np.array([0, 0, 0, 0, 0, 0])  # TODO
    qDes_l = np.array([0, 0, 0, 0, 0, 0])  # TODO
    qDes = np.concatenate((qDes_r, qDes_l))
    qDotDes = np.zeros(dualArmDoF)
    qDDotDes = np.zeros(dualArmDoF)

    q_r = q[:6]
    q_l = q[6:]

    qDot_r = qDot[:6]
    qDot_l = qDot[6:]

    M_r = rbdl_methods.CalcM(loaded_model_r, q_r)
    M_l = rbdl_methods.CalcM(loaded_model_l, q_l)

    M = np.zeros((dualArmDoF, dualArmDoF))
    M[:singleArmDoF, :singleArmDoF] = M_r
    M[singleArmDoF:, singleArmDoF:] = M_l

    h_r = rbdl_methods.CalcH(loaded_model_r, q_r, qDot_r)
    h_l = rbdl_methods.CalcH(loaded_model_l, q_l, qDot_l)
    h = np.zeros(dualArmDoF)
    h = np.concatenate((h_r, h_l))

    pose_error = qDes - q
    vel_error = qDotDes - qDot

    f_prime = qDDotDes + k_v * vel_error + k_p * pose_error

    tau = M.dot(f_prime) + h

    return tau


def BimanualAlgorithm(qCurrent, qDotCurrent, qDDotCurrent,
                      poseDesTraj, velDesTraj, accelDesTraj):

    qCurrent_r = qCurrent[:singleArmDoF]
    qCurrent_l = qCurrent[singleArmDoF:]

    qDotCurrent_r = qDotCurrent[:singleArmDoF]
    qDotCurrent_l = qDotCurrent[singleArmDoF:]

    qDDotCurrent_r = qDDotCurrent[:singleArmDoF]
    qDDotCurrent_l = qDDotCurrent[singleArmDoF:]

    Jac_r = rbdl_methods.Jacobian(loaded_model_r, qCurrent_r, linkName_r,
                                  poseOfTipOfWrist3InWrist3Frame_r)  # (6*6)
    Jac_l = rbdl_methods.Jacobian(loaded_model_l, qCurrent_l, linkName_l,
                                  poseOfTipOfWrist3InWrist3Frame_l)  # (6*6)

    J_r = np.hstack((Jac_r, np.zeros((singleArmDoF,singleArmDoF))))  # (6*12)
    J_l = np.hstack((np.zeros((singleArmDoF,singleArmDoF)), Jac_l))  # (6*12)

    dJdq_r = rbdl_methods.CalcdJdq(loaded_model_r,
                                   qCurrent_r, qDotCurrent_r, qDDotCurrent_r,
                                   linkName_r, poseOfTipOfWrist3InWrist3Frame_r)  # (1*6)
    dJdq_l = rbdl_methods.CalcdJdq(loaded_model_l,
                                   qCurrent_l, qDotCurrent_l, qDDotCurrent_l,
                                   linkName_l, poseOfTipOfWrist3InWrist3Frame_l)  # (1*6)

    ## forward kinematics:
    poseOfTip_r, rotationMatTip_r = \
    rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r, qCurrent_r,
                                            linkName_r,
                                            poseOfTipOfWrist3InWrist3Frame_r)
    poseOfTip_l, rotationMatTip_l = \
    rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_l, qCurrent_l,
                                            linkName_l,
                                            poseOfTipOfWrist3InWrist3Frame_l)
    poseOfObjCOM, rotationMatOfObj = \
            rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r, qCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)

    currentOrientationOfObjInQuat = RotMatToQuaternion(rotationMatOfObj)
    currentGeneralizedPoseOfObjQuat = \
                np.concatenate((poseOfObjCOM, currentOrientationOfObjInQuat))

    # WriteToCSV(np.concatenate([poseDesTraj[:3], poseOfObjCOM]), 'tranlational_trajectory (m)', ['x_des', 'y_des', 'z_des', 'x_actual', 'y_actual', 'z_actual'])
    # WriteToCSV(np.concatenate([poseDesTraj[3:], currentOrientationOfObjInQuat]), 'orientational_trajectory (rad)', ['w_des', 'q_x_des', 'q_y_des', 'q_z_des', 'w_actual', 'q_x_actual', 'q_y_actual', 'q_z_actual'])

    r_o_r = poseOfObjCOM - poseOfTip_r  # (1*3)
    r_o_l = poseOfObjCOM - poseOfTip_l  # (1*3)

    G_o_r = CalcG_oi(r_o_r)  # (6*6)
    G_o_l = CalcG_oi(r_o_l)  # (6*6)

    J_g = J_r - G_o_r.dot(inv(G_o_l).dot(J_l))  # (6*12): equ(7)

    currentGeneralizedVelOfObj = \
            rbdl_methods.CalcGeneralizedVelOfObject(loaded_model_r,
                                                    qCurrent_r,
                                                    qDotCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)

    jointPoseDes, jointVelDes, jointAccelDes, Gdotdz_o_l = \
            TaskToJoint(poseDesTraj, velDesTraj, accelDesTraj, r_o_r, r_o_l,
                        dJdq_r, dJdq_l, G_o_r, G_o_l, J_g, J_r, J_l,
                        currentGeneralizedPoseOfObjQuat,
                        currentGeneralizedVelOfObj)

    jointTau = InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent,
                              jointPoseDes, jointVelDes, jointAccelDes,
                              J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l,
                              Gdotdz_o_l, J_g, currentGeneralizedPoseOfObjQuat,
                              rotationMatTip_r, rotationMatTip_l)

    return jointTau


def IterateThroughTraj6dData(gaz_time):
    global index

    poseTrajectoryDes = np.concatenate((traj_linearPose[index], traj_angularPoseQuat[index]))
    velTrajectoryDes = np.concatenate((traj_linearVel[index], traj_angularVel[index]))
    accelTrajectoryDes = np.concatenate((traj_linearAccel[index], traj_angularAccel[index]))

    index += 10  # 'dt' in 'traj6d' is 10 times lower than 'dt' of Gazebo

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes


def JointStatesCallback(data):
    """
    Subscribe to robot's joints' position and velocity and publish torques.
    """
    global qDotCurrent_pre_r, qDotCurrent_pre_l, dt, time_gaz, time_gaz_pre, \
           rightContactFTSensoryData, leftWristFTSensoryData

    ## Terminate the node after 't_end' seconds of simulation:
    if time_gaz >= t_end and finiteTimeSimFlag:
        print("\n\nshut down 'main_bimanual_node'.\n\n")
        rospy.signal_shutdown('node terminated')

    q = data.position  # class tuple, in radians
    qDot = data.velocity  # in rad/s

    q = np.array(q, dtype=float)
    qDot = np.array(qDot, dtype=float)

    qCurrent_r = np.zeros(singleArmDoF)
    qDotCurrent_r = np.zeros(singleArmDoF)
    qDDotCurrent_r = np.zeros(singleArmDoF)

    qCurrent_l = np.zeros(singleArmDoF)
    qDotCurrent_l = np.zeros(singleArmDoF)
    qDDotCurrent_l = np.zeros(singleArmDoF)

    qCurrent_r[0] = q[5]  # shoulder_pan_joint_r
    qCurrent_r[1] = q[3]  # shoulder_lift_joint_r
    qCurrent_r[2] = q[1]  # elbow_joint_r
    qCurrent_r[3] = q[7]  # wrist_1_joint_r
    qCurrent_r[4] = q[9]  # wrist_2_joint_r
    qCurrent_r[5] = q[11]  # wrist_3_joint_r

    qCurrent_l[0] = q[4]  # shoulder_pan_joint_l
    qCurrent_l[1] = q[2]  # shoulder_lift_joint_l
    qCurrent_l[2] = q[0]  # elbow_joint_l
    qCurrent_l[3] = q[6]  # wrist_1_joint_l
    qCurrent_l[4] = q[8]  # wrist_2_joint_l
    qCurrent_l[5] = q[10]  # wrist_3_joint_l

    qCurrent = np.concatenate((qCurrent_r, qCurrent_l))

    qDotCurrent_r[0] = qDot[5]
    qDotCurrent_r[1] = qDot[3]
    qDotCurrent_r[2] = qDot[1]
    qDotCurrent_r[3] = qDot[7]
    qDotCurrent_r[4] = qDot[9]
    qDotCurrent_r[5] = qDot[11]

    qDotCurrent_l[0] = qDot[4]
    qDotCurrent_l[1] = qDot[2]
    qDotCurrent_l[2] = qDot[0]
    qDotCurrent_l[3] = qDot[6]
    qDotCurrent_l[4] = qDot[8]
    qDotCurrent_l[5] = qDot[10]

    qDotCurrent = np.concatenate((qDotCurrent_r, qDotCurrent_l))

    time_gaz = rospy.get_time()  # get gazebo simulation time (Sim Time)
    dt = time_gaz - time_gaz_pre
    time_gaz_pre = time_gaz

    qDDotCurrent = np.concatenate((qDDotCurrent_r, qDDotCurrent_l))

    # MapFTContactToWorld(loaded_model_r, qCurrent_r, linkName_r, rightContactFTSensoryData)  # uncomment '/ft_sensor_topic_wrist3_r' subscriber
    # MapFTContactToWorld(loaded_model_l, qCurrent_l, linkName_l, leftWristFTSensoryData)  # uncomment '/ft_sensor_topic_wrist3_l' subscriber

    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                                              IterateThroughTraj6dData(time_gaz)

    desJointsTau = BimanualAlgorithm(qCurrent, qDotCurrent, qDDotCurrent,
                      poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes)

    # desJointsTau = PositinoControl(qCurrent, qDotCurrent, qDDotCurrent)

    PubTorqueToGazebo(desJointsTau)



def ReadTrajData():

    global traj_time, traj_linearPose, traj_linearVel, traj_linearAccel, \
    traj_angularPoseQuat, traj_angularVel, traj_angularAccel

    with open(trajDataFile, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(' ')
            l = [x for x in line if x!= '']

            traj_time.append(l[0])

            traj_linearPose.append(l[1:4])  # linear position (x, y, z)
            traj_linearVel.append(l[4:7])  # linear velocity
            traj_linearAccel.append(l[7:10])  # linear acceleration

            traj_angularPoseQuat.append(l[10:14])  # angular position (quaternion (w, q1, q2, q3))
            traj_angularVel.append(l[14:17])  # angular velocity (omega)
            traj_angularAccel.append(l[17:])  # angular acceleration (alpha)

    ## cast string data to float:
    traj_time = np.array(traj_time, dtype=float)
    traj_linearPose = np.array(traj_linearPose, dtype=float)
    traj_linearVel = np.array(traj_linearVel, dtype=float)
    traj_linearAccel = np.array(traj_linearAccel, dtype=float)
    traj_angularPoseQuat = np.array(traj_angularPoseQuat, dtype=float)
    traj_angularVel = np.array(traj_angularVel, dtype=float)
    traj_angularAccel = np.array(traj_angularAccel, dtype=float)


def RemoveCSVFile(path, fileName):
    """Remove the csv file to avoid appending data to the preivous data."""
    if os.path.isfile(path + fileName) is True:
        os.remove(path + fileName)


if __name__ == '__main__':

    RemoveCSVFile(pathToCSVFile, CSVFileName_plot_data)

    try:
        ReadTrajData()  # read entire 'trajDataFile' file and store it in global variables.
        rospy.Subscriber("/joint_states", JointState, JointStatesCallback)
        # rospy.Subscriber("/ft_sensor_topic_wrist3_r", WrenchStamped, FTwristSensorCallback_r)
        # rospy.Subscriber("/ft_sensor_topic_wrist3_l", WrenchStamped, FTwristSensorCallback_l)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

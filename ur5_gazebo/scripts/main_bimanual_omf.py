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
import common_methods
# import QP_opt
####

dt = 0.
time_gaz_pre = 0.
time_gaz = 0.
t_end = 4
g0 = 9.81
writeHeaderOnceFlag = True  # Do not touch
onceExecModelFreeFlag = True  # Do not touch
finiteTimeSimFlag = True  # TODO: True: simulate to 't_end', Flase: Infinite time
minConstraintForceFlag = False  # TODO
minTangentFlag = False  # TODO

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
doubleArmDoF = 2*singleArmDoF
qDotCurrent_pre_r = np.zeros(6)
qDotCurrent_pre_l = np.zeros(6)

qctrl_des_prev = np.zeros(doubleArmDoF)  # initial angular position of joints; joints in home configuration have zero angles.
dqctrl_des_prev = np.zeros(doubleArmDoF)  # initial angular velocity of joints; we usually start from rest
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
                     [0, 0, 1]]) * 30

k_o = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]]) * 5

kp_a_lin = 60  # for steady (home) configuration
# kp_a_lin = 200  # while have motion
kd_a_lin = kp_a_lin / 5

kp_a_ang = 10
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

# M_o_approx = np.eye(6)
M_o_approx = np.eye(6)*obj_mass
inertiaTensor_approx = np.array([[.0021, 0, 0],
                                 [0, .00021, 0],
                                 [0, 0, .0021]])
# M_o_approx[3:, 3:] = inertiaTensor_approx
M_o_approx[3:, 3:] = inertiaTensor

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


rospy.init_node('main_bimanual_omf_node')


qCurrent_r_0 = np.zeros(6)
qCurrent_l_0 = np.zeros(6)

qDotCurrent_r_0 = np.zeros(6)
qDotCurrent_l_0 = np.zeros(6)

qDDotCurrent_r_0 = np.zeros(6)
qDDotCurrent_l_0 = np.zeros(6)

qDDotCurrent_0 = np.concatenate((qDDotCurrent_r_0, qDDotCurrent_l_0))

tau_r = np.zeros(6)
tau_l = np.zeros(6)

M_0 = np.zeros((12, 12))
M_0[:6, :6] = rbdl_methods.CalcM(loaded_model_r, qCurrent_r_0)
M_0[6:, 6:] = rbdl_methods.CalcM(loaded_model_l, qCurrent_l_0)

h_0 = np.zeros((12))
h_0[:6] = rbdl_methods.CalcH(loaded_model_r, qCurrent_r_0, qDotCurrent_r_0)
h_0[6:] = rbdl_methods.CalcH(loaded_model_l, qCurrent_l_0, qDotCurrent_l_0)

Jac_r_0 = rbdl_methods.Jacobian(loaded_model_r, qCurrent_r_0, linkName_r,
                                poseOfTipOfWrist3InWrist3Frame_r)  # (6*6)
Jac_l_0 = rbdl_methods.Jacobian(loaded_model_l, qCurrent_l_0, linkName_l,
                                poseOfTipOfWrist3InWrist3Frame_l)  # (6*6)

J_r_0 = np.hstack((Jac_r_0, np.zeros((singleArmDoF,singleArmDoF))))  # (6*12)
J_l_0 = np.hstack((np.zeros((singleArmDoF,singleArmDoF)), Jac_l_0))  # (6*12)

poseOfTip_r, rotationMatTip_r = \
rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r, qCurrent_r_0,
                                        linkName_r,
                                        poseOfTipOfWrist3InWrist3Frame_r)
poseOfTip_l, rotationMatTip_l = \
rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_l, qCurrent_l_0,
                                        linkName_l,
                                        poseOfTipOfWrist3InWrist3Frame_l)

poseOfObjCOM, rotationMatOfObj = \
        rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r, qCurrent_r_0,
                                                linkName_r,
                                                poseOfObjCOMInWrist3Frame_r)

r_o_r_0 = poseOfObjCOM - poseOfTip_r  # (1*3)
r_o_l_0 = poseOfObjCOM - poseOfTip_l  # (1*3)

G_o_r_0 = common_methods.CalcG_oi(r_o_r_0)  # (6*6)
G_o_l_0 = common_methods.CalcG_oi(r_o_r_0)  # (6*6)

J_g_0 = J_r_0 - G_o_r_0.dot(inv(G_o_l_0).dot(J_l_0))  # (6*12): equ(7)

k, n = J_g_0.shape

S_c = np.hstack((np.eye(k), np.zeros((k, n - k))))
S_u = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

P_qr, Q_0, R_0 = common_methods.CalcPqr(J_g_0, S_u)

S = np.eye(M_0.shape[0])
W = np.eye(S.shape[0])
invw = inv(W)

if False:
    aux1 = invw.dot(S.dot(P_qr.T))
    aux21 = P_qr.dot(S.T.dot(invw))
    aux22 = S.dot(P_qr.T)
    aux2 = aux21.dot(aux22)
    winv_0 = aux1.dot(pinv(aux2))

else:
    w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
    aux = pinv(np.dot(P_qr, np.dot(S.T, w_m_s)))
    winv_0 = np.dot(w_m_s, aux)

tau_pre = np.zeros(12)
J_l_pre = J_l_0
M_pre = M_0
Q_pre = Q_0
winv_pre = winv_0
qdd_pre = qDDotCurrent_0
h_pre = h_0
poseOfTip_r_pre = poseOfTip_r
poseOfTip_l_pre = poseOfTip_l


def TrajEstimate(qdd):
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

    with open(CSVFileName, 'a', newline='') as \
            file:
        writer = csv.writer(file)

        if writeHeaderOnceFlag is True and (legendList is not None or yAxisLabel is not None):
            ## Add header to the CSV file.
            writer.writerow(np.hstack(['time', yAxisLabel, legendList]))
            writeHeaderOnceFlag = False

        writer.writerow(np.hstack([t, data]))  # the first element is time var.


def IntForceParam_mine(J_r, J_l, G_o_r, G_o_l, S, Mqh, orientationOfObjInQuat):
    """Minimizing constaint force."""
    J_total = np.vstack((J_r, J_l))  # (12*12)
    k, n = np.shape(J_total)  # 12, 12
    S_c = np.hstack((np.eye(k), np.zeros((k, n - k))))

    Q_qr, R_qr = qr(J_total.T)  # (12*12), (12*12)

    if minTangentFlag is True:  # ??????????????????????????????
        WW = np.diag([1., 1, 2, 1., 1., 10.])
        Ra = Rotation(orientationOfObjInQuat).T
        Rb = Rotation(orientationOfObjInQuat).T

        auxa = Ra.T.dot(WW.dot(Ra))
        auxb = Rb.T.dot(WW.dot(Rb))

        W_lam[:3, :3] = auxa
        W_lam[3:, 3:] = auxb

    else:
        W_lam = np.eye(R_qr.shape[0])  # I(12*12)

    ## below equ(15), (12*12):
    W_c = Q_qr.dot(S_c.T.dot(inv(R_qr.T).dot(W_lam.dot(inv(R_qr).dot(S_c.dot(Q_qr.T))))))

    ## equ(15):
    W = S.dot(W_c.dot(S.T))  # (12*12)
    tau_0 = S.dot(W_c.T.dot(Mqh))  # (1*12)

    return W, tau_0


def InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent, qDes, qDotDes, qDDotDes,
                   J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l, Gdotdz_o_l,
                   J_g, z_o, poseOfTip_r, poseOfTip_l):

    global tau_pre, Q_pre, M_pre, J_l_pre, winv_pre, qdd_pre, h_pre, poseOfTip_r_pre, poseOfTip_l_pre

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

    M = np.zeros((doubleArmDoF, doubleArmDoF))
    M[:singleArmDoF, :singleArmDoF] = M_r
    M[singleArmDoF:, singleArmDoF:] = M_l

    h_r = rbdl_methods.CalcH(loaded_model_r, qCurrent_r, qDotCurrent_r)
    h_l = rbdl_methods.CalcH(loaded_model_l, qCurrent_l, qDotCurrent_l)
    h = np.zeros(doubleArmDoF)
    h = np.concatenate((h_r, h_l))

    Mqh = M.dot(qDDotDes) + h

    M_hat = M + J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(J_l))))
    C_hat_q_des = J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))))
    h_hat = h + J_l.T.dot(inv(G_o_l.T).dot(h_o))
    Mqh_hat_des = M_hat.dot(qDDotDes) + C_hat_q_des + h_hat

    k, n = J_g.shape  # (6, 12)
    S_u = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))  # below equ(11)
    S_c = np.hstack((np.eye(k), np.zeros((k, n - k))))
    P, qr_Q, qr_R = common_methods.CalcPqr(J_g, S_u)

    S = np.eye(doubleArmDoF)  # (12*12)
    W = np.eye(S.shape[0])  # (12*12)
    tau_0 = np.zeros(S.shape[0])  # (12*1)

    if False:
        invw = inv(W)
        aux1 = invw.dot(S.dot(P.T))
        aux21 = P.dot(S.T.dot(invw))
        aux22 = S.dot(P.T)
        aux2 = aux21.dot(aux22)
        winv = aux1.dot(pinv(aux2))
    else:
        w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
        aux = pinv(np.dot(P, np.dot(S.T, w_m_s)))
        winv = np.dot(w_m_s, aux)

    P_pre = S_u.dot(Q_pre.T)
    # tau_pre = np.concatenate((ua_pre, ub_pre))
    G_ob_approx = common_methods.Estimate_G_ob(poseOfTip_r_pre, poseOfTip_l_pre)  # ????????????
    P1_bar = inv(G_ob_approx.T).dot(M_o_approx.dot(inv(G_ob_approx)))
    aux10 = M + J_l.T.dot(P1_bar.dot(J_l))
    aux10_pre = M_pre + J_l_pre.T.dot(P1_bar.dot(J_l_pre))

    H = tau_pre - winv_pre.dot(P_pre.dot(aux10_pre.dot(qdd_pre) + h_pre))
    tau = winv.dot(P.dot(aux10.dot(qDDotDes) + h)) + H

    tau_pre = tau
    Q_pre = qr_Q
    M_pre = M
    J_l_pre = J_l
    winv_pre = winv
    qdd_pre = qDDotCurrent
    h_pre = h
    poseOfTip_r_pre = poseOfTip_r
    poseOfTip_l_pre = poseOfTip_l

    return S.T.dot(tau) # + 0 * (q_des - q) + 0 * (qdot_des - dq)


def TaskToJoint(qCurrent, qDotCurrent, qDDotCurrent, poseDesTraj, velDesTraj,
                accelDesTraj, r_o_r, r_o_l, dJdq_r, dJdq_l, G_o_r, G_o_l, J_g,
                J_r, J_l, currentGeneralizedPoseOfObjQuat,
                currentGeneralizedVelOfObj):

    qCurrent_r = qCurrent[:singleArmDoF]
    qCurrent_l = qCurrent[singleArmDoF:]

    qDotCurrent_r = qDotCurrent[:singleArmDoF]
    qDotCurrent_l = qDotCurrent[singleArmDoF:]

    currentPoseOfObj = currentGeneralizedPoseOfObjQuat[:3]
    currentOrientationOfObjInQuat = currentGeneralizedPoseOfObjQuat[3:]

    Gdotdz_o_r = common_methods.Calc_dGdz(currentGeneralizedVelOfObj, r_o_r)  # (1*6)
    Gdotdz_o_l = common_methods.Calc_dGdz(currentGeneralizedVelOfObj, r_o_l)  # (1*6)

    dJ_A_dq = dJdq_r - Gdotdz_o_r - \
            G_o_r.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))  # an element of equ(21): (1*6)
    dJ_B_dq = inv(G_o_l).dot(dJdq_l - Gdotdz_o_l)  # an element of equ(21): (1*6)

    translationalError = poseDesTraj[:3] - currentPoseOfObj  # (1*3)
    # WriteToCSV(translationalError, 'pose Error', ['e_x', 'e_y', 'e_z'])

    e_o = common_methods.CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, poseDesTraj[3:])  # equ(4), orientation planning, (1*3)
    # WriteToCSV(e_o, 'orientation Error', ['e_r', 'e_p', 'e_y'])
    poseError = np.concatenate((translationalError, e_o))
    # WriteToCSV(poseError, 'Generalized Pose Error', ['e_x', 'e_y', 'e_z', 'e_roll', 'e_pitch', 'e_yaw'])

    xDot_ref = common_methods.CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o, k_p_pose, k_o)  # equ(1), orientation planning, (1*6)
    velError = xDot_ref - currentGeneralizedVelOfObj

    accelDesTraj_lin = accelDesTraj[:3] + kd_a_lin * velError[:3] + kp_a_lin * poseError[:3]  # a, below equ(21), (1*3)
    accelDesTraj_ang = accelDesTraj[3:] + kd_a_ang * velError[3:] + kp_a_ang * poseError[3:]  # a, below equ(21), (1*3)
    accelDesTraj = np.concatenate((accelDesTraj_lin, accelDesTraj_ang))

    J_B = inv(G_o_l).dot(J_l)  # (6*12)
    J_A = np.vstack((J_g, J_B))  # (12*12)
    dJ_A_dq = np.concatenate((dJ_A_dq, dJ_B_dq))  # (1*12)
    xDDotDes = np.concatenate((np.zeros(6), accelDesTraj))  # (1*12)
    qDDotDes = pinv(J_A).dot(xDDotDes - dJ_A_dq)  # equ(21), (1*12)
    # print(np.round(qDDotDes, 3))

    qDes, qDotDes = TrajEstimate(qDDotDes)

    return qDes, qDotDes, qDDotDes, Gdotdz_o_l


def FTwristSensorCallback_l(ft_data_l):
    """Sensor mounted in 'wrist_3_joint_l'. """
    global leftWristFTSensoryData, wristFTsensorData_l

    wrist_force = ft_data_l.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data_l.wrench.torque  # [T_x, T_y, T_z] local frame

    wristFTsensorData_l = \
                    np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                              wrist_torque.x, wrist_torque.y, wrist_torque.z])

    sensorAngleTo_x = common_methods.acos_(wrist_force.x / (mg_wrist_3 + mg_obj))
    sensorAngleTo_y = common_methods.acos_(wrist_force.y / (mg_wrist_3 + mg_obj))
    sensorAngleTo_z = common_methods.acos_(wrist_force.z / (mg_wrist_3 + mg_obj))

    ## remove effect of 'wrist_3_link_l' mass for force:
    Fobj_x = wrist_force.x - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of 'wrist_3_link_l' torque:
    Tobj_x = wrist_torque.x - wrist_3_length*mg_obj*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z - wrist_3_length*mg_obj*cos(sensorAngleTo_x)

    leftWristFTSensoryData = np.array([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z])


def FTwristSensorCallback_r(ft_data_r):
    """Sensor mounted in 'wrist_3_joint_r'. """
    global rightContactFTSensoryData, wristFTsensorData_r

    wrist_force = ft_data_r.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data_r.wrench.torque  # [T_x, T_y, T_z] local frame

    wristFTsensorData_r = \
                    np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                              wrist_torque.x, wrist_torque.y, wrist_torque.z])

    sensorAngleTo_x = common_methods.acos_(wrist_force.x / (mg_wrist_3 + mg_obj))
    sensorAngleTo_y = common_methods.acos_(wrist_force.y / (mg_wrist_3 + mg_obj))
    sensorAngleTo_z = common_methods.acos_(wrist_force.z / (mg_wrist_3 + mg_obj))

    ## remove effect of 'wrist_3_link_r' mass for force:
    Fobj_x = wrist_force.x - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of 'wrist_3_link_r' torque:
    Tobj_x = wrist_torque.x - wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_z) - l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z + wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_x) + l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_x)

    rightContactFTSensoryData = np.array([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z])


def BimanualAlgorithm_OMF(qCurrent, qDotCurrent, qDDotCurrent,
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

    currentOrientationOfObjInQuat = common_methods.RotMatToQuaternion(rotationMatOfObj)
    currentGeneralizedPoseOfObjQuat = \
                np.concatenate((poseOfObjCOM, currentOrientationOfObjInQuat))

    r_o_r = poseOfObjCOM - poseOfTip_r  # (1*3)
    r_o_l = poseOfObjCOM - poseOfTip_l  # (1*3)

    G_o_r = common_methods.CalcG_oi(r_o_r)  # (6*6)
    G_o_l = common_methods.CalcG_oi(r_o_l)  # (6*6)

    J_g = J_r - G_o_r.dot(inv(G_o_l).dot(J_l))  # (6*12): equ(7)

    currentGeneralizedVelOfObj = \
            rbdl_methods.CalcGeneralizedVelOfObject(loaded_model_r,
                                                    qCurrent_r,
                                                    qDotCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)

    jointPoseDes, jointVelDes, jointAccelDes, Gdotdz_o_l = \
            TaskToJoint(qCurrent, qDotCurrent, qDDotCurrent,
                        poseDesTraj, velDesTraj, accelDesTraj, r_o_r, r_o_l,
                        dJdq_r, dJdq_l, G_o_r, G_o_l, J_g, J_r, J_l,
                        currentGeneralizedPoseOfObjQuat,
                        currentGeneralizedVelOfObj)

    jointTau = InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent,
                              jointPoseDes, jointVelDes, jointAccelDes,
                              J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l,
                              Gdotdz_o_l, J_g, currentGeneralizedPoseOfObjQuat,
                              poseOfTip_r, poseOfTip_l)

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

    # common_methods.MapFTContactToWorld(loaded_model_r, qCurrent_r, linkName_r, rightContactFTSensoryData)  # uncomment '/ft_sensor_topic_wrist3_r' subscriber
    # common_methods.MapFTContactToWorld(loaded_model_l, qCurrent_l, linkName_l, leftWristFTSensoryData)  # uncomment '/ft_sensor_topic_wrist3_l' subscriber

    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                                              IterateThroughTraj6dData(time_gaz)

    ## Object model free algorithm:
    desJointsTau = BimanualAlgorithm_OMF(qCurrent, qDotCurrent, qDDotCurrent,
                      poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes)

    # desJointsTau = common_methods.PositinoControl(qCurrent, qDotCurrent, qDDotCurrent)

    common_methods.PubTorqueToGazebo(desJointsTau)


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


if __name__ == '__main__':

    common_methods.RemoveCSVFile(pathToCSVFile, CSVFileName_plot_data)

    try:
        ReadTrajData()  # read entire 'trajDataFile' file and store it in global variables.
        rospy.Subscriber("/joint_states", JointState, JointStatesCallback)
        # rospy.Subscriber("/ft_sensor_topic_wrist3_r", WrenchStamped, FTwristSensorCallback_r)
        # rospy.Subscriber("/ft_sensor_topic_wrist3_l", WrenchStamped, FTwristSensorCallback_l)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

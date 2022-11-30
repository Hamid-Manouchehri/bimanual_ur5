#!/usr/bin/env python3
"""
Created on Sat Aug 27 11:20:33 2022

@author: Hamid Manouchehri
"""
from __future__ import division
from numpy.linalg import inv, pinv
from scipy.linalg import qr, sqrtm
from math import sin, cos, sqrt, atan2, asin
import rbdl
import os
import csv
import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from squaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from sys import path
path.insert(1, '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_description/config/')  # TODO: Insert dir of config folder of the package.
import config
os.chdir('/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_gazebo/scripts/')  # TODO: Insert dir of config folder of the package.
import rbdl_methods  # TODO: change the file name if necessary.
####

time_ = 0
dt = 0.
time_gaz_pre = 0.
time_gaz = 0.
t_end = 4.5
g0 = 9.81
writeHeaderOnceFlag = True
finiteTimeSimFlag = True  # TODO: True: simulate to 't_end', Flase: Infinite time
""" For specifying which arm is right(r) and which one is left(l), imagine
yourself on top of the table and your front is toward positive x-axis.
"""
linkName_r = 'wrist_3_link_r'
linkName_l = 'wrist_3_link_l'

wrist_3_length = 0.0823  # based on 'ur5.urdf.xacro'
obj_length = .2174  # based on 'ur5.urdf.xacro'
objCOMinWrist3_r = obj_length / 4 + wrist_3_length
objCOMinWrist3_l = 3*obj_length / 4 + wrist_3_length
poseOfObjCOMInWrist3Frame_r = np.array([0., objCOMinWrist3_r, 0.])  # (x, y, z)
poseOfObjCOMInWrist3Frame_l = np.array([0., objCOMinWrist3_l, 0.])
poseOfTipOfWrist3InWrist3Frame_r = np.array([0., wrist_3_length, 0.])  # (x, y, z)
poseOfTipOfWrist3InWrist3Frame_l = poseOfTipOfWrist3InWrist3Frame_r

workspaceDoF = 6
singleArmDoF = 6
doubleArmDoF = 2*singleArmDoF
qDotCurrent_pre_r = np.zeros(6)
qDotCurrent_pre_l = np.zeros(6)

qctrl_des_prev = np.zeros(doubleArmDoF)  # initial angular position of joints; joints in home configuration have zero angles.
dqctrl_des_prev = np.zeros(doubleArmDoF)  # initial angular velocity of joints; we usually start from rest

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
                [0, 0, 1]]) * 100

kp_a_lin = 80
kd_a_lin = kp_a_lin / 15

kp_a_ang = 120
kd_a_ang = kp_a_ang / 10

kp = np.array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1]]) * 80

kd = np.array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1]]) * 10


## right arm:
initialPoseOfObjInWorld_x_r = 0.3922
initialPoseOfObjInWorld_y_r = -.191
initialPoseOfObjInWorld_z_r = .609

## Attention that the sequence of orientation is xyz Euler angles:
initialOrientationOfObj_roll_r = 0.
initialOrientationOfObj_pitch_r = 0.
initialOrientationOfObj_yaw_r = 0.

## left arm:
initialPoseOfObjInWorld_x_l = 0.3922
initialPoseOfObjInWorld_y_l = .191
initialPoseOfObjInWorld_z_l = .609

initialOrientationOfObj_roll_l = 0.
initialOrientationOfObj_pitch_l = 0.
initialOrientationOfObj_yaw_l = np.pi


## dynamic specifications of object:
obj_mass = .5  # TODO: according to 'ur5.urdf.xacro'
io_zz = 1  # TODO: according to 'ur5.urdf.xacro'
M_o = np.eye(singleArmDoF)*obj_mass
# M_o[5, 5] = io_zz  # ???????????????????????????
h_o = np.array([0, obj_mass*g0, 0, 0, 0, 0]) # ??????????????????????????????????


## define dependent directories and including files:
CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']
CSVFileName = CSVFileName

upperBodyModelFileName_r = config.bimanual_ur5_dic['urdfModelName_r']
upperBodyModelFileName_l = config.bimanual_ur5_dic['urdfModelName_l']
pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']
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
    Publish torques to Gazebo (manipulate the object in a linear trajectory)
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


def CalcOrientationErrorInQuaternion(orientationInQuatCurrent, orientationInQuatDes):
    """Calculate position error in Quaternion based on the paper."""

    wCurrent = orientationInQuatCurrent[0]
    vCurrent = orientationInQuatCurrent[1:]

    wDes = orientationInQuatDes[0]
    vDes = orientationInQuatDes[1:]

    e_o = np.dot(wCurrent, vDes) - np.dot(wDes, vCurrent) - np.cross(vDes, vCurrent)  # (3*1)

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

    with open(CSVFileName, 'a', newline='') as \
            file:
        writer = csv.writer(file)

        if writeHeaderOnceFlag is True and (legendList is not None or yAxisLabel is not None):
            ## Add header to the CSV file.
            writer.writerow(np.hstack(['time', yAxisLabel, legendList]))
            writeHeaderOnceFlag = False

        writer.writerow(np.hstack([t, data]))  # the first element is time var.


def CalcGeneralizedVel_ref(currentPose, desPose, desVel, e_o):
    """Calculate position error in Quaternion based on the paper."""

    P_dot_des = desVel[:3]
    omega_des = desVel[3:]

    P_dot_ref = P_dot_des + k_p_pose.dot(desPose - currentPose)
    omega_ref = omega_des + k_o.dot(e_o)

    X_dot_r = np.concatenate((P_dot_ref, omega_ref))

    return X_dot_r


def IntForceParam_mine(J_r, J_l, G_o_r, G_o_l, S, Mqh, theta, W_lam_i=None):
    """Minimizing constaint force."""
    J = np.vstack((J_r, J_l))  # (6*6)
    k, n = np.shape(J)
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))

    Q, RR = qr(J.T)  # (6*6), (6*6)
    R = RR[:k, :k]

    if W_lam_i is None:
        W_lam_i = np.eye(R.shape[0])  # I(6*6)

    if min_tangent:  # QUES_2: why minimization is on rotation (Ri)?
        WW = np.diag([1., 10, 2])
        Ra = Rotation(theta).T
        Rb = Rotation(theta).T

        auxa = np.dot(Ra.T, np.dot(WW, Ra))
        auxb = np.dot(Rb.T, np.dot(WW, Rb))

        W_lam_i[:3, :3] = auxa
        W_lam_i[3:, 3:] = auxb

    W_lam = W_lam_i  # (6*6)

    # below equ(15)(6*6):
    Wc = np.dot(Q, np.dot(Sc.T, np.dot(inv(R.T),
                                       np.dot(W_lam,
                                       np.dot(inv(R), np.dot(Sc, Q.T))))))

    # equ(15):
    W = np.dot(S, np.dot(Wc, S.T))  # (6*6)
    tau_0 = np.dot(S, np.dot(Wc.T, Mqh))  # (6*6)

    return W, tau_0


def QRDecompose(J):

    JT = J.T
    m, n = JT.shape

    if m == 0 or n == 0:
        raise TypeError(
            'Try to calculate QR decomposition, while there is no contact!')

    qr_Q, qr_R = qr(JT)
    qr_R = qr_R[:n, :]

    return qr_Q, qr_R


def CalcPqr(J, Su):
    qr_Q, qr_R = QRDecompose(J)

    return np.dot(Su, qr_Q.T), qr_Q, qr_R


def InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent, qDes, qDotDes, qDDotDes,
                   J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l, Gdotdz_o_l,
                   J_g, X_o):

    qCurrent_r = qCurrent[:singleArmDoF]
    qCurrent_l = qCurrent[singleArmDoF:]

    qDotCurrent_r = qDotCurrent[:singleArmDoF]
    qDotCurrent_l = qDotCurrent[singleArmDoF:]

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

    M_bar = M + J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(J_l))))
    C_barq_des = J_l.T.dot(inv(G_o_l.T).dot(M_o.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))))
    h_bar = h + J_l.T.dot(inv(G_o_l.T).dot(h_o))
    Mqh_bar_des = M_bar.dot(qDDotDes) + C_barq_des + h_bar

    k, n = J_g.shape
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))  # below equ(11)
    P, qr_Q, qr_R = CalcPqr(J_g, Su)

    S = np.eye(doubleArmDoF)  # (1*12)

    # W, tau_0 = IntForceParam_mine(J_r, J_l, G_o_r, G_o_l, S, Mqh, X_o[3:])
    W = np.eye(S.shape[0])  # (12*12)
    tau_0 = np.zeros(S.shape[0])  # (1*12)

    ## below equ(10):
    w_m_s = np.linalg.matrix_power(sqrtm(W), -1)  # W^(-1/2)
    aux1 = pinv(P.dot(S.T.dot(w_m_s)))  # P*S.T*W^(-1/2)
    winv = w_m_s.dot(aux1)

    aux2 = (np.eye(S.shape[0]) - winv.dot(P.dot(S.T))).dot(inv(W))  # null-space term of equ(10)

    # equ(10):
    invdyn = winv.dot(P.dot(Mqh_bar_des)) + aux2.dot(tau_0)
    # invdyn = Mqh_bar_des

    return invdyn


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
        q_0 = 1/2*np.array([sqrt(1 + r11 + r22 + r33),
                            (r23 - r32)/sqrt(1 + r11 + r22 + r33),
                            (r13 - r31)/sqrt(1 + r11 + r22 + r33),
                            (r12 - r21)/sqrt(1 + r11 + r22 + r33)])

        unitQuat = q_0

    elif r22 < -r33 and r11 > r22 and r11 > r33:
        q_1 = 1/2*np.array([(r23 - r32)/sqrt(1 + r11 - r22 - r33),
                             sqrt(1 + r11 - r22 - r33),
                             (r12 + r21)/sqrt(1 + r11 - r22 - r33),
                             (r13 + r31)/sqrt(1 + r11 - r22 - r33)])

        unitQuat = q_1

    elif r22 > r33 and r11 < r22 and r11 < -r33:
        q_2 = 1/2*np.array([(r31 - r13)/sqrt(1 - r11 + r22 - r33),
                            (r12 + r21)/sqrt(1 - r11 + r22 - r33),
                            sqrt(1 - r11 + r22 - r33),
                            (r23 + r32)/sqrt(1 - r11 + r22 - r33)])

        unitQuat = q_2

    elif r22 < r33 and r11 < -r22 and r11 < r33:
        q_3 = 1/2*np.array([(r12 - r21)/sqrt(1 - r11 - r22 + r33),
                            (r13 + r31)/sqrt(1 - r11 - r22 + r33),
                            (r23 + r32)/sqrt(1 - r11 - r22 + r33),
                            sqrt(1 - r11 - r22 + r33)])

        unitQuat = q_3

    else:
        print('there is something wrong with "RotMatToQuaternion" !')
        unitQuat = None

    return unitQuat


def Calc_dGdz(dX_o, r_o_i):

    currentOmega_o = dX_o[3:]

    omegaCrossRoiCrossOmega = \
                    np.cross(currentOmega_o, np.cross(r_o_i, currentOmega_o))

    block_WRW = SkewSymMat(np.array(omegaCrossRoiCrossOmega))  # (3*3)

    block_63 = np.vstack((block_WRW, np.zeros((3, 3))))  # (6*3)

    blockZ_63 = np.vstack((np.zeros((3, 3)), np.zeros((3, 3))))  # (6*3)

    Gdot_oi = np.hstack((blockZ_63, block_63))  # (6*6)

    Gdot_oiV_o = Gdot_oi.dot(dX_o)  # GDot_oa*qDot_o: (1*6)

    return Gdot_oiV_o  # (1*6)


def TaskToJoint(qCurrent, qDotCurrent, qDDotCurrent, poseDesTraj, velDesTraj,
                accelDesTraj, r_o_r, r_o_l, dJdq_r, dJdq_l, G_o_r, G_o_l, J_g,
                J_r, J_l, currentGeneralizedPoseOfObj):

    qCurrent_r = qCurrent[:singleArmDoF]
    qCurrent_l = qCurrent[singleArmDoF:]

    qDotCurrent_r = qDotCurrent[:singleArmDoF]
    qDotCurrent_l = qDotCurrent[singleArmDoF:]

    currentPoseOfObj = currentGeneralizedPoseOfObj[:3]
    currentOrientationOfObjInQuat = currentGeneralizedPoseOfObj[3:]

    currentGeneralizedVelOfObj = \
            rbdl_methods.CalcGeneralizedVelOfObject(loaded_model_r,
                                                    qCurrent_r,
                                                    qDotCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)

    Gdotdz_o_r = Calc_dGdz(currentGeneralizedVelOfObj, r_o_r)  # (1*6)
    Gdotdz_o_l = Calc_dGdz(currentGeneralizedVelOfObj, r_o_l)  # (1*6)

    dJ_A_dq = dJdq_r - Gdotdz_o_r - \
            G_o_r.dot(inv(G_o_l).dot(dJdq_l - Gdotdz_o_l))  # an element of equ(21): (1*6)
    dJ_B_dq = inv(G_o_l).dot(dJdq_l - Gdotdz_o_l)  # an element of equ(21): (1*6)

    translationalError = poseDesTraj[:3] - currentPoseOfObj  # (1*3)
    e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, poseDesTraj[3:])  # equ(4), orientation planning, (1*3)
    poseError = np.concatenate((translationalError, e_o))

    xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning, (1*6)
    velError = xDot_ref - currentGeneralizedVelOfObj

    accelDesTraj_lin = accelDesTraj[:3] + kd_a_lin * velError[:3] + kp_a_lin * poseError[:3]  # a, below equ(21), (1*3)
    accelDesTraj_ang = accelDesTraj[3:] + kd_a_ang * velError[3:] + kp_a_ang * poseError[3:]  # a, below equ(21), (1*3)
    accelDesTraj = np.concatenate((accelDesTraj_lin, accelDesTraj_ang))

    J_B = inv(G_o_l).dot(J_l)
    J_A = np.vstack((J_g, J_B))
    dJ_A_dq = np.concatenate((dJ_A_dq, dJ_B_dq))
    xDDotDes = np.concatenate((np.zeros(6), accelDesTraj))
    qDDotDes = pinv(J_A).dot(xDDotDes - dJ_A_dq)  # equ(21), (1*12)

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


def BimanualAlgorithm(qCurrent, qDotCurrent, qDDotCurrent, poseDesTraj,
                      velDesTraj, accelDesTraj):

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

    currentOrientationOfObjInQuat = RotMatToQuaternion(rotationMatOfObj)
    currentGeneralizedPoseOfObj = np.concatenate((poseOfObjCOM, currentOrientationOfObjInQuat))

    r_o_r = poseOfObjCOM - poseOfTip_r  # (1*3)
    r_o_l = poseOfObjCOM - poseOfTip_l  # (1*3)

    G_o_r = CalcG_oi(r_o_r)  # (6*6)
    G_o_l = CalcG_oi(r_o_l)  # (6*6)

    J_g = J_r - G_o_r.dot(inv(G_o_l).dot(J_l))  # J_g (6*12): equ(7)

    jointPoseDes, jointVelDes, jointAccelDes, Gdotdz_o_l = \
            TaskToJoint(qCurrent, qDotCurrent, qDDotCurrent,
                        poseDesTraj, velDesTraj, accelDesTraj, r_o_r, r_o_l,
                        dJdq_r, dJdq_l, G_o_r, G_o_l, J_g, J_r, J_l,
                        currentGeneralizedPoseOfObj)


    jointTau = InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent,
                              jointPoseDes, jointVelDes, jointAccelDes,
                              J_r, J_l, G_o_r, G_o_l, dJdq_r, dJdq_l,
                              Gdotdz_o_l, J_g, currentGeneralizedPoseOfObj)

    return jointTau


def TrajectoryGeneration(t):
    """Circular trajectory of the defined 'radius':"""

    radius = .2

    ## desired trajectory (position):
    xDes = initialPoseOfObjInWorld_x_r
    yDes = radius*np.sin(t) + initialPoseOfObjInWorld_y_r + objCOMinWrist3_r
    zDes = radius*np.cos(t) + initialPoseOfObjInWorld_z_r
    phiDes = initialOrientationOfObj_roll_r
    thetaDes = initialOrientationOfObj_pitch_r
    psyDes = initialOrientationOfObj_yaw_r
    poseTrajectoryDes = np.array([xDes, yDes, zDes, phiDes, thetaDes, psyDes])

    ## desired trajectory (velocity):
    xDotDes = 0.
    yDotDes = radius*np.cos(t)
    zDotDes = -radius*np.sin(t)
    phiDotDes = 0.
    thetaDotDes = 0.
    psyDotDes = 0.
    velTrajectoryDes = np.array([xDotDes, yDotDes, zDotDes,
                                 phiDotDes, thetaDotDes, psyDotDes])

    ## desired trajectory (acceleration):
    xDDotDes = 0.
    yDDotDes = -radius*np.sin(t)
    zDDotDes = -radius*np.cos(t)
    phiDDotDes = 0.
    thetaDDotDes = 0.
    psyDDotDes = 0.
    accelTrajectoryDes = np.array([xDDotDes, yDDotDes, zDDotDes,
                                   phiDDotDes, thetaDDotDes, psyDDotDes])

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes


def IterateThroughTraj6dData(gaz_time):
    global index

    # index = ClosestValue(traj_time[i : step_size], gaz_time)

    poseTrajectoryDes = np.concatenate((traj_linearPose[index], traj_angularPoseQuat[index]))
    velTrajectoryDes = np.concatenate((traj_linearVel[index], traj_angularVel[index]))
    accelTrajectoryDes = np.concatenate((traj_linearAccel[index], traj_angularAccel[index]))

    index += 10
    # index += 1

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes


def JointStatesCallback(data):
    """
    Subscribe to robot's joints' position and velocity and publish torques.
    """
    global time_, qDotCurrent_pre_r, qDotCurrent_pre_l, dt, time_gaz, \
           time_gaz_pre

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
    dt = time_gaz - time_gaz_pre + .001
    time_gaz_pre = time_gaz

    qDDotCurrent_r = (qDotCurrent_r - qDotCurrent_pre_r) / dt
    qDotCurrent_pre_r = qDotCurrent_r

    qDDotCurrent_l = (qDotCurrent_l - qDotCurrent_pre_l) / dt
    qDotCurrent_pre_l = qDotCurrent_l

    qDDotCurrent = np.concatenate((qDDotCurrent_r, qDDotCurrent_l))

    # poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
    #                                             TrajectoryGeneration(time_gaz)
    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                                              IterateThroughTraj6dData(time_gaz)

    desJointsTau = BimanualAlgorithm(qCurrent, qDotCurrent, qDDotCurrent,
                      poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes)

    PubTorqueToGazebo(desJointsTau)

    # time_ += dt


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
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

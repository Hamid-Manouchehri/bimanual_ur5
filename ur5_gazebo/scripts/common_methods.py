#!/usr/bin/env python3

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

linkName_r = 'wrist_3_link_r'
linkName_l = 'wrist_3_link_l'

g0 = 9.81
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

## define dependent directories and including files:
pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']
upperBodyModelFileName_r = config.bimanual_ur5_dic['urdfModelName_r']
upperBodyModelFileName_l = config.bimanual_ur5_dic['urdfModelName_l']
loaded_model_r = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName_r)
loaded_model_l = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName_l)


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


################################################################################
################################################################################
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

################################################################################
################################################################################
def CalcOrientationErrorInQuaternion(currentOrientationInQuat, desOrientationInQuat):
    """Calculate position error in Quaternion based on the paper."""

    wCurrent = currentOrientationInQuat[0]
    vCurrent = currentOrientationInQuat[1:]

    wDes = desOrientationInQuat[0]
    vDes = desOrientationInQuat[1:]

    e_o = np.dot(wCurrent, vDes) - np.dot(wDes, vCurrent) - np.cross(vDes, vCurrent)  # (1*3)

    return e_o  # (1*3)

################################################################################
################################################################################
def CalcGeneralizedVel_ref(currentPose, desPose, desVel, e_o, k_p_pose, k_o):

    P_dot_des = desVel[:3]
    omega_des = desVel[3:]

    P_dot_ref = P_dot_des + k_p_pose.dot(desPose - currentPose)  # (1*3)
    omega_ref = omega_des + k_o.dot(e_o)  # (1*3)

    X_dot_r = np.concatenate((P_dot_ref, omega_ref))

    return X_dot_r

################################################################################
################################################################################
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

################################################################################
################################################################################
def QRparam(J):
    k, n = J.shape
    Sc = np.hstack((np.eye(k), np.zeros((k, n - k))))
    Su = np.hstack((np.zeros((n - k, k)), np.eye(n - k)))

    P_qr, Q, R = CalcPqr(J, Su)

    n_m_k, n = P_qr.shape
    k = n - n_m_k
    return P_qr, Q, R, k, n, Sc, Su

################################################################################
################################################################################
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

################################################################################
################################################################################
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

################################################################################
################################################################################
def SkewSymMat(inputVector):
    """Compute the corresponding skew symetric matrix of 'inputVector' vector."""
    inputVector = np.array([inputVector])
    zeroMat = np.zeros((3, 3))
    zeroMat[0, 1] = -inputVector[0, 2]
    zeroMat[0, 2] = inputVector[0, 1]
    zeroMat[1, 2] = -inputVector[0, 0]
    zeroMat = zeroMat + (-zeroMat.T)

    return zeroMat  # (3*3)

################################################################################
################################################################################
def CalcG_oi(r_o_i):

    P_oi_cross = SkewSymMat(r_o_i)  # (3*3)
    G_oi_topRows = np.hstack((np.eye(3), P_oi_cross))
    G_oi_bottomRows = np.hstack((np.zeros((3, 3)), np.eye(3)))
    G_oi = np.vstack((G_oi_topRows, G_oi_bottomRows))  # (6*6)

    return G_oi

################################################################################
################################################################################
def Estimate_G_ob(x_a, x_b):

    x_o = (x_a + x_b)/2
    r_o_b = x_o - x_b
    G_ob = CalcG_oi(r_o_b)

    return G_ob

################################################################################
################################################################################
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

    return unitQuat

################################################################################
################################################################################
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

################################################################################
################################################################################
def MapFTContactToWorld(loaded_model, qCurrent, linkName, contactFTsensorData):
    """Project measured ft forces in the contact (right end-effector)."""

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
        # print('FT in Wrist_r:', np.round(wristFTsensorData_r, 3))
        # print('FT in Contact_r:', np.round(contactFTsensorData, 3))
        print('FT in World_r:', np.round(contactFTinWorld, 3), '\n')
        pass

    elif linkName is linkName_l:
        # print('FT in Wrist_l:', np.round(wristFTsensorData_l, 3))
        # print('FT in Contact_l:', np.round(contactFTsensorData, 3))
        # print('FT in World_l:', np.round(contactFTinWorld, 3), '\n')
        pass

################################################################################
################################################################################
def acos_(value):
    """ to avoid value error 'math domain error' """
    if value > 1:
        return acos(1)

    elif value < -1:
        return acos(-1)

    else:
        return acos(value)

################################################################################
################################################################################
def PositinoControl(q, qDot, qDDot):
    """ Control position conntrol for testing coordinations"""
    k_p = 700
    k_v = 40

    ## joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    qDes_r = np.array([0, 0, 0, 0, 0, 0])  # TODO
    qDes_l = np.array([0, 0, 0, 0, 0, 0])  # TODO
    qDes = np.concatenate((qDes_r, qDes_l))
    qDotDes = np.zeros(doubleArmDoF)
    qDDotDes = np.zeros(doubleArmDoF)

    q_r = q[:6]
    q_l = q[6:]

    qDot_r = qDot[:6]
    qDot_l = qDot[6:]

    M_r = rbdl_methods.CalcM(loaded_model_r, q_r)
    M_l = rbdl_methods.CalcM(loaded_model_l, q_l)

    M = np.zeros((doubleArmDoF, doubleArmDoF))
    M[:singleArmDoF, :singleArmDoF] = M_r
    M[singleArmDoF:, singleArmDoF:] = M_l

    h_r = rbdl_methods.CalcH(loaded_model_r, q_r, qDot_r)
    h_l = rbdl_methods.CalcH(loaded_model_l, q_l, qDot_l)
    h = np.zeros(doubleArmDoF)
    h = np.concatenate((h_r, h_l))

    pose_error = qDes - q
    vel_error = qDotDes - qDot

    f_prime = qDDotDes + k_v * vel_error + k_p * pose_error

    tau = M.dot(f_prime) + h

    return tau

################################################################################
################################################################################
def RemoveCSVFile(path, fileName):
    """Remove the csv file to avoid appending data to the preivous data."""
    if os.path.isfile(path + fileName) is True:
        os.remove(path + fileName)

################################################################################
################################################################################

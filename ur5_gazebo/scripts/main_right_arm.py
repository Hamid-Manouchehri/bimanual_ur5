#!/usr/bin/env python3
"""
Created on Sat Aug 27 11:20:33 2022

@author: Hamid Manouchehri
"""
from __future__ import division

from numpy.linalg import inv, pinv, cond
from scipy.linalg import qr, sqrtm
from math import sin, cos, sqrt, acos
import rbdl
import os
import csv
import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import pyquaternion as qt
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
t_end = 4
g0 = 9.81
writeHeaderOnceFlag = True
finiteTimeSimFlag = False  # TODO: True: simulate to 't_end', Flase: Infinite time
wrist_3_length = 0.0823  # based on 'ur5.urdf.xacro'
obj_length = .2174  # based on 'ur5.urdf.xacro'
obj_mass = 1.0  # based on 'ur5.urdf.xacro'
mg_obj = obj_mass * g0
wrist_3_mass = .1879  # based on 'ur5.urdf.xacro'
mg_wrist_3 = wrist_3_mass * g0
l_obj_frame_to_wrist3_com = .035 / 2  # based on 'ur5.urdf.xacro'
l_frame_to_com_wrist3 = wrist_3_length - l_obj_frame_to_wrist3_com  # based on 'ur5.urdf.xacro'
l_frame_to_com_obj = .1087  # based on 'ur5.urdf.xacro'

linkName_r = 'wrist_3_link_r'

wrist_3_length = 0.0823  # based on 'ur5.urdf.xacro'
obj_length = .2174  # based on 'ur5.urdf.xacro'
objCOMinWrist3_r = obj_length / 2 + wrist_3_length  # TODO: uncomment as object is added.
# objCOMinWrist3_r = 0  # TODO: uncomment as object is removed.
poseOfObjCOMInWrist3Frame_r = np.array([0., objCOMinWrist3_r, 0.])  # (x, y, z)
l_contact_to_com_obj_wrist3 = .0565  # based on 'ur5.urdf.xacro'
wrist_3_mass = .1879  # based on 'ur5.urdf.xacro'
mg_wrist_3 = wrist_3_mass * g0

workspaceDoF = 6
singleArmDoF = 6
qDotCurrent_pre_r = np.zeros(singleArmDoF)
qctrl_des_prev_r = np.zeros(singleArmDoF)  # initial angular position of joints; joints in home configuration have zero angles.
dqctrl_des_prev_r = np.zeros(singleArmDoF)  # initial angular velocity of joints; we usually start from rest
rightWristFTSensoryData = np.array([0.]*workspaceDoF)

index = 0
traj_time = []
traj_linearPose, traj_linearVel, traj_linearAccel = [], [], []
traj_angularPoseQuat, traj_angularVel, traj_angularAccel = [], [], []

k_p_pose = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]) * 50

k_o = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]]) * 20

kp_a_lin = 300
kd_a_lin = kp_a_lin / 5

kp_a_ang = 50
kd_a_ang = kp_a_ang / 5


initialPoseOfObjInWorld_x_r = 0.3922
# initialPoseOfObjInWorld_y_r = -.191   # TODO: uncomment as object is removed
initialPoseOfObjInWorld_y_r = -.191 + objCOMinWrist3_r  # = .2415; TODO: uncomment as object is added
initialPoseOfObjInWorld_z_r = .609

initialOrientationOfObj_roll_r = 0.
initialOrientationOfObj_pitch_r = 0.
initialOrientationOfObj_yaw_r = 0.


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
h_o = np.array([0., 0., -mg_obj, 0., 0., 0.])


pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']
CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
CSVFileName = pathToCSVFile + CSVFileName_plot_data

pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']
upperBodyModelFileName = config.bimanual_ur5_dic['urdfModelName_r']
loaded_model_r = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName)

pathToTrajData = config.bimanual_ur5_dic['trajDataFileDirectory']
trajDataFileName_r = config.bimanual_ur5_dic['trajDataFileName_r']
trajDataFile = pathToTrajData + trajDataFileName_r


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


rospy.init_node('right_ur5_node')


def TrajEstimate(qdd):
    global dqctrl_des_prev_r, qctrl_des_prev_r

    Dt = dt
    qdot_des_now_r = dqctrl_des_prev_r + Dt*qdd  # numerical derivation
    dqctrl_des_prev_r = qdot_des_now_r

    q_des_now_r = qctrl_des_prev_r + Dt*qdot_des_now_r
    qctrl_des_prev_r = q_des_now_r

    q_des_now_filtered = []

    for i in q_des_now_r:
        i = np.mod(i, np.pi*2)
        if i > np.pi:
            i = i - np.pi*2
        q_des_now_filtered.append(i)

    return np.array(q_des_now_filtered), qdot_des_now_r


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

        writer.writerow(np.hstack([t, data]))  # the first el ment is time var.


def EulerToUnitQuaternion_zyx(eulerAngles):
    """calculate equ(297) of "attitude" paper:"""
    phi = eulerAngles[0]
    theta = eulerAngles[1]
    psy = eulerAngles[2]

    w = np.cos(phi/2)*np.cos(theta/2)*np.cos(psy/2) + np.sin(phi/2)*np.sin(theta/2)*np.sin(psy/2)
    v1 = -np.cos(phi/2)*np.sin(theta/2)*np.sin(psy/2) + np.cos(theta/2)*np.cos(psy/2)*np.sin(phi/2)
    v2 = np.cos(phi/2)*np.cos(psy/2)*np.sin(theta/2) + np.sin(phi/2)*np.cos(theta/2)*np.sin(psy/2)
    v3 = np.cos(phi/2)*np.cos(theta/2)*np.sin(psy/2) - np.sin(phi/2)*np.cos(psy/2)*np.sin(theta/2)

    return np.array([w, v1, v2, v3])


def CalcGeneralizedVel_ref(currentPose, desPose, desVel, e_o):
    """Calculate position error in Quaternion based on the paper."""

    P_dot_des = desVel[:3]
    omega_des = desVel[3:]

    P_dot_ref = P_dot_des + k_p_pose.dot(desPose - currentPose)
    omega_ref = omega_des + k_o.dot(e_o)

    # X_dot_r = np.concatenate((P_dot_ref, omega_ref))
    # return X_dot_r

    return P_dot_ref, omega_ref


def PositinoControl(q_r, qDot_r, qDDot_r):

    k_p = 500
    k_v = 30

    ## joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    qDes_r = np.array([0., 0., 0., 0., 0., 0.])  # TODO
    qDotDes_r = np.zeros(singleArmDoF)
    qDDotDes_r = np.zeros(singleArmDoF)

    M_r = rbdl_methods.CalcM(loaded_model_r, q_r)
    h_r = rbdl_methods.CalcH(loaded_model_r, q_r, qDot_r)

    pose_error = qDes_r - q_r
    vel_error = qDotDes_r - qDot_r

    f_prime = qDDotDes_r + k_v * vel_error + k_p * pose_error

    tau = M_r.dot(f_prime) + h_r

    return tau


def InverseDynamic(qCurrent_r, qDotCurrent_r, qDDotCurrent_r, qDes, qDotDes, qDDotDes):

    M = rbdl_methods.CalcM(loaded_model_r, qCurrent_r)
    h = rbdl_methods.CalcH(loaded_model_r, qCurrent_r, qDotCurrent_r)

    ## Inverse Dynamics controller:
    desiredTorque = M.dot(qDDotDes) + h

    return desiredTorque


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
                                 (r31 - r13)/sqrt(1 + r11 + r22 + r33),
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


def Task2Joint(qCurrent_r, qDotCurrent_r, qDDotCurrent_r, poseDesTraj, velDesTraj, accelDesTraj):

    currentPoseOfObj, RotMat = \
            rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_r,
                                                    qCurrent_r,
                                                    linkName_r,
                                                    poseOfObjCOMInWrist3Frame_r)  # pose of 'wrist_3_link' frame in world/inertia frame
    currentOrientationOfObjInQuat = RotMatToQuaternion(RotMat)  # [w, q0, q1, q2], equ(145), attitude

    # desOrientationOfObjInQuat = EulerToUnitQuaternion_zyx(poseDesTraj[3:])  # equ(297), attitude, for circular trajectory
    # e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, desOrientationOfObjInQuat)  # for circular trajectory

    e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, poseDesTraj[3:])  # equ(4), orientation planning, for traj6d trajectory
    # WriteToCSV(e_o, 'pose Error_angular', ['e_r', 'e_p', 'e_y'])

    # xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning
    Pdot_ref, omega_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning

    currentVelOfObj = \
          rbdl_methods.CalcGeneralizedVelOfObject(loaded_model_r,
                                                  qCurrent_r,
                                                  qDotCurrent_r,
                                                  linkName_r,
                                                  poseOfObjCOMInWrist3Frame_r)

    translationalError = poseDesTraj[:3] - currentPoseOfObj
    # WriteToCSV(translationalError, 'pose Error_linear', ['e_x', 'e_y', 'e_z'])

    accelDesTraj_lin = accelDesTraj[:3] + kd_a_lin * (Pdot_ref - currentVelOfObj[:3]) + kp_a_lin * translationalError
    accelDesTraj_ang = accelDesTraj[3:] + kd_a_ang * (omega_ref - currentVelOfObj[3:]) + kp_a_ang * e_o
    accelDesTraj = np.concatenate((accelDesTraj_lin, accelDesTraj_ang))

    dJdq = rbdl_methods.CalcdJdq(loaded_model_r,
                                 qCurrent_r, qDotCurrent_r, qDDotCurrent_r,
                                 linkName_r, poseOfObjCOMInWrist3Frame_r)

    jac = rbdl_methods.Jacobian(loaded_model_r, qCurrent_r, linkName_r,
                                poseOfObjCOMInWrist3Frame_r)

    qDDotDes = pinv(jac).dot(accelDesTraj - dJdq)  # equ(21)

    qDes, qDotDes = TrajEstimate(qDDotDes)

    return qDes, qDotDes, qDDotDes


def SkewSymMat(inputVector):
    """Compute the corresponding skew symetric matrix of 'inputVector' vector."""
    inputVector = np.array([inputVector])
    zeroMat = np.zeros((3, 3))
    zeroMat[0, 1] = -inputVector[0, 2]
    zeroMat[0, 2] = inputVector[0, 1]
    zeroMat[1, 2] = -inputVector[0, 0]
    zeroMat = zeroMat + (-zeroMat.T)

    return zeroMat  # (3*3)


def IterateThroughTraj6dData(gaz_time):
    global index

    poseTrajectoryDes = np.concatenate((traj_linearPose[index], traj_angularPoseQuat[index]))
    velTrajectoryDes = np.concatenate((traj_linearVel[index], traj_angularVel[index]))
    accelTrajectoryDes = np.concatenate((traj_linearAccel[index], traj_angularAccel[index]))

    index += 10  # 'dt' in 'traj6d' is 10 times lower than 'dt' of Gazebo
    # index += 1

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes


def TrajectoryGeneration(r, t):
    """Circular trajectory of the defined 'radius':"""

    ## desired trajectory (position):
    xDes = initialPoseOfObjInWorld_x_r
    yDes = initialPoseOfObjInWorld_y_r + r*np.cos(t)
    zDes = initialPoseOfObjInWorld_z_r + r*np.sin(t)
    phiDes = initialOrientationOfObj_roll_r
    thetaDes = initialOrientationOfObj_pitch_r
    psyDes = initialOrientationOfObj_yaw_r
    poseTrajectoryDes = np.array([xDes, yDes, zDes, phiDes, thetaDes, psyDes])

    ## desired trajectory (velocity):
    xDotDes = 0.
    yDotDes = -r*np.sin(t)
    zDotDes = r*np.cos(t)
    phiDotDes = 0.
    thetaDotDes = 0.
    psyDotDes = 0.
    velTrajectoryDes = np.array([xDotDes, yDotDes, zDotDes,
                                 phiDotDes, thetaDotDes, psyDotDes])

    ## desired trajectory (acceleration):
    xDDotDes = 0.
    yDDotDes = -r*np.cos(t)
    zDDotDes = -r*np.sin(t)
    phiDDotDes = 0.
    thetaDDotDes = 0.
    psyDDotDes = 0.
    accelTrajectoryDes = np.array([xDDotDes, yDDotDes, zDDotDes,
                                   phiDDotDes, thetaDDotDes, psyDDotDes])

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes

def acos_(value):
    """ to avoid value error 'math domain error' """
    if value > 1:
        return acos(1)

    elif value < -1:
        return acos(-1)

    else:
        return acos(value)


def FTwristSensorCallback_r(ft_data_r):
    """Sensor mounted in 'wrist_3_joint_r'. """
    global rightContactFTSensoryData, sensorData_r, omega_x, omega_z

    wrist_force = ft_data_r.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data_r.wrench.torque  # [T_x, T_y, T_z] local frame

    sensorData_r = np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                             wrist_torque.x, wrist_torque.y, wrist_torque.z])

    sensorAngleTo_x = acos_(wrist_force.x / (mg_wrist_3 + mg_obj))
    sensorAngleTo_y = acos_(wrist_force.y / (mg_wrist_3 + mg_obj))
    sensorAngleTo_z = acos_(wrist_force.z / (mg_wrist_3 + mg_obj))

    ## remove effect of 'wrist_3_link_r' mass for force:
    Fobj_x = wrist_force.x - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of 'wrist_3_link_r' torque:
    Tobj_x = wrist_torque.x - wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_z) - l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z + wrist_3_length*(mg_wrist_3 + mg_obj)*cos(sensorAngleTo_x) + l_contact_to_com_obj_wrist3*mg_wrist_3*cos(sensorAngleTo_x)

    rightContactFTSensoryData = np.array([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z])


def MapFTContactToWorld(loaded_model, qCurrent, linkName, FTsensorWristData):
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

    FTinWorld = T_s_t.dot(FTsensorWristData)  # (1*6)

    print('FT in Contact:', np.round(FTsensorWristData, 3))
    print('FT in World:', np.round(FTinWorld, 3))
    print('FT in Wrist:', np.round(sensorData_r, 3), '\n')



def JointStatesCallback(data):
    """
    Subscribe to robot's joints' position and velocity and publish torques.
    """
    global time_, qDotCurrent_pre_r, dt, time_gaz, time_gaz_pre

    ## Terminate the node after 't_end' seconds of simulation:
    if time_gaz >= t_end and finiteTimeSimFlag:
        print("\n\nshut down 'main_node'.\n\n")
        rospy.signal_shutdown('node terminated')

    q = data.position  # class tuple, in radians
    qDot = data.velocity  # in rad/s

    q = np.array(q, dtype=float)
    qDot = np.array(qDot, dtype=float)

    qCurrent_r = np.zeros(singleArmDoF)
    qDotCurrent_r = np.zeros(singleArmDoF)
    qDDotCurrent_r = np.zeros(singleArmDoF)

    qCurrent_r[0] = q[5]  # shoulder_pan_joint_r
    qCurrent_r[1] = q[3]  # shoulder_lift_joint_r
    qCurrent_r[2] = q[1]  # elbow_joint_r
    qCurrent_r[3] = q[7]  # wrist_1_joint_r
    qCurrent_r[4] = q[9]  # wrist_2_joint_r
    qCurrent_r[5] = q[11]  # wrist_3_joint_r

    qDotCurrent_r[0] = qDot[5]
    qDotCurrent_r[1] = qDot[3]
    qDotCurrent_r[2] = qDot[1]
    qDotCurrent_r[3] = qDot[7]
    qDotCurrent_r[4] = qDot[9]
    qDotCurrent_r[5] = qDot[11]

    time_gaz = rospy.get_time()  # get gazebo simulation time (Sim Time)
    dt = time_gaz - time_gaz_pre + .001
    time_gaz_pre = time_gaz

    # qDDotCurrent_r = (qDotCurrent_r - qDotCurrent_pre_r) / dt
    # qDotCurrent_pre_r = qDotCurrent_r

    MapFTContactToWorld(loaded_model_r, qCurrent_r, linkName_r, rightContactFTSensoryData)  # uncomment '/ft_sensor_topic_wrist3_r' subscriber

    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                                              IterateThroughTraj6dData(time_gaz)
    # poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
    #                                             TrajectoryGeneration(.1, time_gaz)

    ## detemine desired states of robot in joint-space: (qDDot_desired, ...)
    jointPoseDes, jointVelDes, jointAccelDes = Task2Joint(qCurrent_r,
                                                          qDotCurrent_r,
                                                          qDDotCurrent_r,
                                                          poseTrajectoryDes,
                                                          velTrajectoryDes,
                                                          accelTrajectoryDes)

    # jointTau = InverseDynamic(qCurrent_r, qDotCurrent_r, qDDotCurrent_r,
    #                           jointPoseDes, jointVelDes, jointAccelDes)

    jointTau = PositinoControl(qCurrent_r, qDotCurrent_r, qDDotCurrent_r)

    PubTorqueToGazebo(jointTau)


def FTwristSensorCallback_r(ft_data):
    """Sensor mounted in 'wrist_3_joint_r'. """
    global rightWristFTSensoryData

    wrist_force = ft_data.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data.wrench.torque  # [T_x, T_y, T_z] local frame

    rightWristFTSensoryData = \
                    np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                              wrist_torque.x, wrist_torque.y, wrist_torque.z],
                              dtype=float)

    sensorAngleTo_x = acos(wrist_force.x / (mg_wrist_3 + mg_obj))
    sensorAngleTo_y = acos(wrist_force.y / (mg_wrist_3 + mg_obj))
    sensorAngleTo_z = acos(wrist_force.z / (mg_wrist_3 + mg_obj))

    Fobj_x = wrist_force.x - mg_wrist_3*cos(sensorAngleTo_x)
    Fobj_y = wrist_force.y - mg_wrist_3*cos(sensorAngleTo_y)
    Fobj_z = wrist_force.z - mg_wrist_3*cos(sensorAngleTo_z)

    ## remove effect of wrist_3 torque:
    Tobj_x = wrist_torque.x - l_frame_to_com_wrist3*mg_wrist_3*cos(sensorAngleTo_z) -\
                        l_frame_to_com_wrist3*mg_obj*cos(sensorAngleTo_z) - \
                        l_obj_frame_to_wrist3_com*mg_obj*cos(sensorAngleTo_z)
    Tobj_y = wrist_torque.y
    Tobj_z = wrist_torque.z + l_frame_to_com_wrist3*mg_wrist_3*cos(sensorAngleTo_x) + \
                        l_frame_to_com_wrist3*mg_obj*cos(sensorAngleTo_x) + \
                        l_obj_frame_to_wrist3_com*mg_obj*cos(sensorAngleTo_x)

    print('wrist sensor:', np.round([Fobj_x, Fobj_y, Fobj_z, Tobj_x, Tobj_y, Tobj_z], 3))


def FTobjSensorCallback_r(ft_data):
    """Sensor mounted in 'wrist_3_to_obj_joint_r'. """

    wrist_force = ft_data.wrench.force  # [F_x, F_y, F_z] local frame
    wrist_torque = ft_data.wrench.torque  # [T_x, T_y, T_z] local frame

    objFTSensoryData = np.array([wrist_force.x, wrist_force.y, wrist_force.z,
                                 wrist_torque.x, wrist_torque.y, wrist_torque.z],
                                 dtype=float)

    print('obj joint sensor: ', np.round(objFTSensoryData, 3), '\n')


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
        rospy.Subscriber("/ft_sensor_topic_wrist3_r", WrenchStamped, FTwristSensorCallback_r)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

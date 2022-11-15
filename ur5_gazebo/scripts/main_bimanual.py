#!/usr/bin/env python3
"""
Created on Sat Aug 27 11:20:33 2022

@author: Hamid Manouchehri
"""
from __future__ import division
from numpy.linalg import inv, pinv
from scipy.linalg import qr, sqrtm
from math import sin, cos, sqrt
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
import methods  # TODO: change the file name if necessary.
####

time_ = 0
dt = 0.
time_gaz_pre = 0.
time_gaz = 0.
t_end = 5
g0 = 9.81
writeHeaderOnceFlag = True
finiteTimeSimFlag = True  # TODO: True: simulate to 't_end', Flase: Infinite time
linkName = 'wrist_3_link_l'  # TODO

workspaceDof = 6  # TODO
singleArmDof = 6  # TODO
qDotCurrent_pre = np.array([0, 0, 0, 0, 0, 0])
poseOfObj_pre = np.array([0, 0, 0, 0, 0, 0])

k_p_pose = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]) * 20

k_o = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]]) * 5

kp_a = 100
kd_a = kp_a / 10

kp = np.array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1]]) * 60

kd = np.array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 1]]) * 25


"""
(x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)):
Note: any modification to trajectory need modification of 'LABEL_1',
function of 'ChooseRef' and of course the following trajecory definitions:
Note: translational coordinations (x, y, z) are in world frame, so avoid values
which are close to 'child_neck' frame.
"""
## right arm:
poseOfObjInWorld_x_r = 0.3922
poseOfObjInWorld_y_r = -.191
poseOfObjInWorld_z_r = .609

## Attention that the sequence of orientation is xyz Euler angles:
orientationOfObj_roll_r = 0.
orientationOfObj_pitch_r = 0.
orientationOfObj_yaw_r = 0.

## left arm:
poseOfObjInWorld_x_l = 0.3922
poseOfObjInWorld_y_l = .191
poseOfObjInWorld_z_l = .609

orientationOfObj_roll_l = 0.
orientationOfObj_pitch_l = 0.
orientationOfObj_yaw_l = -np.pi


## Path attributes:
numOfTraj = 2

desiredInitialStateOfObj_traj_1 = np.array([poseOfObjInWorld_x_l, poseOfObjInWorld_y_l, poseOfObjInWorld_z_l,
                                            orientationOfObj_roll_l, orientationOfObj_pitch_l, orientationOfObj_yaw_l])

desiredFinalStateOfObj_traj_1 = np.array([poseOfObjInWorld_x_l, poseOfObjInWorld_y_l, poseOfObjInWorld_z_l + .2,
                                          orientationOfObj_roll_l, orientationOfObj_pitch_l, orientationOfObj_yaw_l])

desiredInitialStateOfObj_traj_2 = desiredFinalStateOfObj_traj_1
desiredFinalStateOfObj_traj_2 = desiredInitialStateOfObj_traj_1


initPoseVelAccelOfObj_traj_1 = [desiredInitialStateOfObj_traj_1, np.zeros(workspaceDof), np.zeros(workspaceDof)]
finalPoseVelAccelOfObj_traj_1 = [desiredFinalStateOfObj_traj_1, np.zeros(workspaceDof), np.zeros(workspaceDof)]

initPoseVelAccelOfObj_traj_2 = [desiredInitialStateOfObj_traj_2, np.zeros(workspaceDof), np.zeros(workspaceDof)]
finalPoseVelAccelOfObj_traj_2 = [desiredFinalStateOfObj_traj_2, np.zeros(workspaceDof), np.zeros(workspaceDof)]


q_obj = [desiredInitialStateOfObj_traj_1]
q = [[0.]*singleArmDof]
q = np.hstack((q, q_obj))

q_des = [q[-1][:singleArmDof]]

qctrl_des_prev = q_des[-1][:singleArmDof]
dqctrl_des_prev = np.zeros(singleArmDof)  # TODO: we usually start from rest


CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']

upperBodyModelFileName = config.bimanual_ur5_dic['urdfModelName_l']  # TODO
pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']

loaded_model = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName)

## create instances of publishers:
pub_shoulder_pan = rospy.Publisher('/shoulder_pan_controller_l/command',
                   Float64, queue_size=10)
pub_shoulder_lift = rospy.Publisher('/shoulder_lift_controller_l/command',
                    Float64, queue_size=10)
pub_elbow = rospy.Publisher('/elbow_controller_l/command',
            Float64, queue_size=10)
pub_wrist_1 = rospy.Publisher('/wrist_1_controller_l/command',
              Float64, queue_size=10)
pub_wrist_2 = rospy.Publisher('/wrist_2_controller_l/command',
              Float64, queue_size=10)
pub_wrist_3 = rospy.Publisher('/wrist_3_controller_l/command',
              Float64, queue_size=10)


rospy.init_node('main_node')


def TrajEstimate(qdd):
    Dt = dt
    qdot_des_now = dqctrl_des_prev + Dt*qdd  # numerical derivation
    q_des_now = qctrl_des_prev + Dt*qdot_des_now

    q_des_now_filtered = []

    for i in q_des_now:
        i = np.mod(i, np.pi*2)
        if i > np.pi:
            i = i - np.pi*2
        q_des_now_filtered.append(i)

    return np.array(q_des_now_filtered), qdot_des_now


def TrajPlan(t_start, t_end, z_start_o, z_end_o, traj_type='Quantic'):
    """
    Compute desired generalize trajectory, velocity and acceleration ...
    of the Object.
    """
    initGeneralizedPose, initGeneralizedVel, initGeneralizedAccel = z_start_o
    finalGeneralizedPose, finalGeneralizedVel, finalGeneralizedAccel = z_end_o

    def f(x):
        return [x**5, x**4, x**3, x**2, x, 1]

    def fd(x):
        return [5*x**4, 4*x**3, 3*x**2, 2*x, 1, 0]

    def fdd(x):
        return [20*x**3, 12*x**2, 6*x, 2, 0, 0]

    if traj_type == 'Quantic':

        A = np.array([f(t_start), fd(t_start), fdd(t_start),
                      f(t_end), fd(t_end), fdd(t_end)])

        desiredGeneralizedTraj = []
        desiredGeneralizedVel = []
        desiredGeneralizedAccel = []

        """
        Iterate to calculate all the states of the object for position,
        velocity and acceleration then append to the list:
        """
        for i in range(len(initGeneralizedPose)):

            B = np.array([[initGeneralizedPose[i], initGeneralizedVel[i],
                           initGeneralizedAccel[i], finalGeneralizedPose[i],
                           finalGeneralizedVel[i],
                           finalGeneralizedAccel[i]]]).T

            p = np.dot(np.linalg.inv(A), B)

            desiredGeneralizedTraj += list([lambda x, p=p:
                                            sum([p[0]*x**5, p[1]*x**4,
                                                 p[2]*x**3, p[3]*x**2,
                                                 p[4]*x, p[5]])])

            desiredGeneralizedVel.append(lambda x, p=p:
                                         sum([p[0]*5*x**4, p[1]*4*x**3,
                                              p[2]*3*x**2, p[3]*2*x, p[4]]))

            desiredGeneralizedAccel.append(lambda x, p=p:
                                           sum([p[0]*20*x**3, p[1]*12*x**2,
                                                p[2]*6*x, p[3]*2]))

    return [desiredGeneralizedTraj, desiredGeneralizedVel,
            desiredGeneralizedAccel]


def PubTorqueToGazebo(torqueVec):
    """
    Publish torques to Gazebo (manipulate the object in a linear trajectory)
    """
    pub_shoulder_pan.publish(torqueVec[0])
    pub_shoulder_lift.publish(torqueVec[1])
    pub_elbow.publish(torqueVec[2])
    pub_wrist_1.publish(torqueVec[3])
    pub_wrist_2.publish(torqueVec[4])
    pub_wrist_3.publish(torqueVec[5])


def CalcOrientationErrorInQuaternion(orientationInQuatCurrent, orientationInQuatDes):
    """Calculate position error in Quaternion based on the paper."""

    wCurrent = orientationInQuatCurrent[0]
    vCurrent = orientationInQuatCurrent[1:]

    wDes = orientationInQuatDes[0]
    vDes = orientationInQuatDes[1:]

    e_o = np.dot(wCurrent, vDes) - np.dot(wDes, vCurrent) - np.cross(vDes, vCurrent)  # (3*1)

    return e_o  # (1*3)


def CalcEulerGeneralizedAccel(angularPose, eulerVel, eulerAccel):
    """
    Calculate conversion of Euler acceleration (ddRoll, ddPitch, ddYaw)
    into angular acceleration (alpha_x, alpha_y, alpha_z):
    """
    phi = angularPose[3]
    theta = angularPose[4]
    psy = angularPose[5]

    dPhi = eulerVel[3]
    dTheta = eulerVel[4]
    dPsy = eulerVel[5]

    ddPhi = eulerAccel[3]
    ddTheta = eulerAccel[4]
    ddPsy = eulerAccel[5]

    ## Time derivative of 'transformMat.objEulerVel':
    alpha_x = -sin(psy)*cos(theta)*dPhi*dPsy - \
              sin(psy)*ddTheta - sin(theta)*cos(psy)*dPhi*dTheta + \
              cos(psy)*cos(theta)*ddPhi - cos(psy)*dPsy*dTheta

    alpha_y = -sin(psy)*sin(theta)*dPhi*dTheta + \
              sin(psy)*cos(theta)*ddPhi - sin(psy)*dPsy*dTheta + \
              cos(psy)*cos(theta)*dPhi*dPsy + cos(psy)*ddTheta

    alpha_z = -sin(theta)*ddPhi - cos(theta)*dPhi*dTheta + ddPsy


    desiredGeneralizedAccelOfObj = np.concatenate((eulerAccel[:3],
                                                   [alpha_x, alpha_y, alpha_z]))

    return desiredGeneralizedAccelOfObj


def CalcEulerGeneralizedVel(xObjDes, xDotObjDes):
    """
    Calculate conversion of Euler angles rate of change (dRoll, dPitch, dYaw)
    into angular velocity (omega_x, omega_y, omega_z):
    """
    phi = xObjDes[3]
    tehta = xObjDes[4]
    psy = xObjDes[5]

    eulerAngleRatesMat = np.array([[np.cos(tehta)*np.cos(psy), -np.sin(psy), 0],
                                   [np.cos(tehta)*np.sin(psy), np.cos(psy), 0],
                                   [-np.sin(tehta), 0, 1]])

    omegaVec = eulerAngleRatesMat.dot(xDotObjDes[3:])  # (1*3)

    translationalVelOfObj_r = xDotObjDes[:3]

    desiredGeneralizedVelOfObj = np.concatenate((translationalVelOfObj_r,
                                                 omegaVec))

    return desiredGeneralizedVelOfObj


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
        t = time_

    with open(pathToCSVFile + CSVFileName_plot_data, 'a', newline='') as \
            file:
        writer = csv.writer(file)

        if writeHeaderOnceFlag is True and (legendList is not None or yAxisLabel is not None):
            ## Add header to the CSV file.
            writer.writerow(np.hstack(['time', yAxisLabel, legendList]))
            writeHeaderOnceFlag = False

        writer.writerow(np.hstack([t, data]))  # the first element is time var.

def EulerToUnitQuaternion_xyz(eulerAngles):
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

    X_dot_r = np.concatenate((P_dot_ref, omega_ref))

    return X_dot_r



def InverseDynamic(qCurrent, qDotCurrent, qDDotCurrent, qDes, qDotDes, qDDotDes):

    M = methods.CalcM(loaded_model, qCurrent)
    h = methods.CalcH(loaded_model, qCurrent, qDotCurrent, qDDotCurrent)

    ## Inverse Dynamics controller:
    errorPose = qDes - qCurrent
    errorVel = qDotDes - qDotCurrent
    qDDot_des = qDDotDes + kd.dot(errorVel) + kp.dot(errorPose)
    desiredTorque = M.dot(qDDot_des) + h

    ## PD with gravity compnensation controller: modify 'methods.CalcH()'
    # errorPose = qDes - qCurrent
    # errorVel = qDotDes - qDotCurrent
    # desiredTorque = h + kp.dot(errorPose) - kd.dot(qDotCurrent)

    return desiredTorque


def RotMatToQuaternion(R):
    """section 6.5 of 'attitude' paper:
    Note: rotation matrix in the paper is defined in the way that maps world
    into body-fixed frame, so there needs some modification...
    """
    R = R.T  # Note ^
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


def Task2Joint(qCurrent, qDotCurrent, qDDotCurrent, poseDesTraj, velDesTraj, accelDesTraj):

    jac = methods.Jacobian(loaded_model, linkName, qCurrent)

    currentPoseOfObj, RotMat = methods.GeneralizedPoseOfObj(loaded_model,
                                                            linkName, qCurrent)  # pose of 'wrist_3_link' frame in world/inertia frame
    currentOrientationOfObjInQuat = RotMatToQuaternion(RotMat)  # [w, q0, q1, q2], equ(145), attitude

    desOrientationOfObjInQuat = EulerToUnitQuaternion_xyz(poseDesTraj[3:])  # equ(297), attitude, which order ?????????????????

    translationalError = poseDesTraj[:3] - currentPoseOfObj
    e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat,
                                           desOrientationOfObjInQuat)  # equ(4), orientation planning
    poseError = np.concatenate((translationalError, e_o))

    velOfObjDes = CalcEulerGeneralizedVel(poseDesTraj, velDesTraj)  # ??????????????????
    accelOfObjDes = CalcEulerGeneralizedAccel(poseDesTraj, velDesTraj,
                                              accelDesTraj)  # ??????????????????

    xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning
    # xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velOfObjDes, e_o)  # equ(1), orientation planning

    currentVelOfObj = methods.CalcGeneralizedVelOfObject(loaded_model, linkName,
                                                         qCurrent, qDotCurrent)

    velError = xDot_ref - currentVelOfObj


    # accelDesTraj = accelOfObjDes + kd_a * velError + kp_a * poseError  # or below ????????
    accelDesTraj = accelDesTraj + kd_a * velError + kp_a * poseError
    # WriteToCSV(poseError[3:], 'angular pose error', ['phi', 'theta', 'psy'])

    dJdq = methods.CalcdJdq(loaded_model, linkName, qCurrent, qDotCurrent,
                            qDDotCurrent)

    qDDotDes = pinv(jac).dot(accelDesTraj - dJdq)  # equ(21)

    qDes, qDotDes = TrajEstimate(qDDotDes)

    return qDes, qDotDes, qDDotDes


def CalcDesiredTraj(xDes, xDotDes, xDDotDes, t):
    """ trajectory generation of end-effector in task-space:"""

    xDes_now = np.array([xDes[i](t) for i in rlx]).flatten()
    xDotDes_now = np.array([xDotDes[i](t) for i in rlx]).flatten()
    xDDotDes_now = np.array([xDDotDes[i](t) for i in rlx]).flatten()

    return xDes_now, xDotDes_now, xDDotDes_now


def ChooseRef(time):
    if time <= t_end/numOfTraj:
        out = [desPose_traj_1, desVel_traj_1, desAccel_traj_1, time]

    else:
        out = [desPose_traj_2, desVel_traj_2, desAccel_traj_2, time - t_end/numOfTraj]

    return out


def TrajectoryGeneration(t):
    """Circular trajectory of the defined 'radius':"""

    radius = .2

    # ## desired trajectory (position):
    # xDes = poseOfObjInWorld_x_r
    # yDes = radius*np.sin(t) + .4
    # zDes = radius*np.cos(t) + .4
    # phiDes = 0.
    # thetaDes = 0.
    # psyDes = 0.
    # poseTrajectoryDes = np.array([xDes, yDes, zDes, phiDes, thetaDes, psyDes])
    #
    # ## desired trajectory (velocity):
    # xDotDes = 0.
    # yDotDes = radius*np.cos(t)
    # zDotDes = -radius*np.sin(t)
    # phiDotDes = 0.
    # thetaDotDes = 0.
    # psyDotDes = 0.
    # velTrajectoryDes = np.array([xDotDes, yDotDes, zDotDes,
    #                              phiDotDes, thetaDotDes, psyDotDes])
    #
    # ## desired trajectory (acceleration):
    # xDDotDes = 0.
    # yDDotDes = -radius*np.sin(t)
    # zDDotDes = -radius*np.cos(t)
    # phiDDotDes = 0.
    # thetaDDotDes = 0.
    # psyDDotDes = 0.
    # accelTrajectoryDes = np.array([xDDotDes, yDDotDes, zDDotDes,
    #                                phiDDotDes, thetaDDotDes, psyDDotDes])

    ## desired trajectory (position):
    xDes = poseOfObjInWorld_x_l
    yDes = poseOfObjInWorld_y_l
    zDes = poseOfObjInWorld_z_l
    phiDes = 0.
    thetaDes = 0.
    psyDes = t
    poseTrajectoryDes = np.array([xDes, yDes, zDes, phiDes, thetaDes, psyDes])

    ## desired trajectory (velocity):
    xDotDes = 0.
    yDotDes = 0.
    zDotDes = 0.
    phiDotDes = 0.
    thetaDotDes = 0.
    psyDotDes = 1
    velTrajectoryDes = np.array([xDotDes, yDotDes, zDotDes,
                                 phiDotDes, thetaDotDes, psyDotDes])

    ## desired trajectory (acceleration):
    xDDotDes = 0.
    yDDotDes = 0.
    zDDotDes = 0.
    phiDDotDes = 0.
    thetaDDotDes = 0.
    psyDDotDes = 0.
    accelTrajectoryDes = np.array([xDDotDes, yDDotDes, zDDotDes,
                                   phiDDotDes, thetaDDotDes, psyDDotDes])

    return poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes


def JointStatesCallback(data):
    """
    Subscribe to robot's joints' position and velocity and publish torques.
    """
    global time_, qDotCurrent_pre, poseOfObj_pre, dt, time_gaz, time_gaz_pre

    ## Terminate the node after 't_end' seconds of simulation:
    # if time_ >= t_end and finiteTimeSimFlag:
    #     print("\n\nshut down 'main_node'.\n\n")
    #     rospy.signal_shutdown('node terminated')

    q = data.position  # class tuple, in radians
    qDot = data.velocity  # in rad/s

    q = np.array(q, dtype=float)
    qDot = np.array(qDot, dtype=float)

    qCurrent_r = np.zeros(singleArmDof)
    qDotCurrent_r = np.zeros(singleArmDof)
    qDDotCurrent_r = np.zeros(singleArmDof)

    qCurrent_l = np.zeros(singleArmDof)
    qDotCurrent_l = np.zeros(singleArmDof)
    qDDotCurrent_l = np.zeros(singleArmDof)

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


    time_gaz = rospy.get_time()  # get gazebo simulation time (Sim Time)
    dt = time_gaz - time_gaz_pre + .001
    time_gaz_pre = time_gaz

    qDDotCurrent_l = (qDotCurrent_l - qDotCurrent_pre) / dt
    qDotCurrent_pre = qDotCurrent_l

    xDes_t, xDotDes_t, xDDotDes_t, timePrime = ChooseRef(time_gaz)

    ## end-effector desired states in Cartesian-space:
    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                    CalcDesiredTraj(xDes_t, xDotDes_t, xDDotDes_t, timePrime)
    # poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
    #                                             TrajectoryGeneration(time_gaz)

    ## detemine desired states of robot in joint-space: (qDDot_desired, ...)
    jointPoseDes, jointVelDes, jointAccelDes = Task2Joint(qCurrent_l,
                                                          qDotCurrent_l,
                                                          qDDotCurrent_l,
                                                          poseTrajectoryDes,
                                                          velTrajectoryDes,
                                                          accelTrajectoryDes)


    # jointPoseDes = np.array([0., 0., 0., 0., 0., 0.])
    # jointVelDes = np.array([0., 0., 0., 0., 0., 0.])
    # jointAccelDes = np.array([0., 0., 0., 0., 0., 0.])

    jointTau = InverseDynamic(qCurrent_l, qDotCurrent_l, qDDotCurrent_l,
                              jointPoseDes, jointVelDes, jointAccelDes)
    # print(np.round(jointTau, 3))
    PubTorqueToGazebo(jointTau)

    time_ += dt


def RemoveCSVFile(path, fileName):
    """Remove the csv file to avoid appending data to the preivous data."""
    if os.path.isfile(path + fileName) is True:
        os.remove(path + fileName)


if __name__ == '__main__':

    RemoveCSVFile(pathToCSVFile, CSVFileName_plot_data)

    ## Generate desired states for the whole trajectory of object: (LABEL_1)
    ## First trajectory:
    desPose_traj_1, desVel_traj_1, desAccel_traj_1 = \
        TrajPlan(0, t_end/numOfTraj, initPoseVelAccelOfObj_traj_1,
                 finalPoseVelAccelOfObj_traj_1)

    ## Second trajectory:
    desPose_traj_2, desVel_traj_2, desAccel_traj_2 = \
        TrajPlan(0, t_end/numOfTraj, initPoseVelAccelOfObj_traj_2,
                 finalPoseVelAccelOfObj_traj_2)

    rlx = range(len(desPose_traj_1))  # range(0, 6) --> 0, 1, 2, 3, 4, 5

    try:
        rospy.Subscriber("/joint_states", JointState, JointStatesCallback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

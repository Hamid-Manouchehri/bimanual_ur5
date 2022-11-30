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
t_end = 4.5
g0 = 9.81
writeHeaderOnceFlag = True
finiteTimeSimFlag = False  # TODO: True: simulate to 't_end', Flase: Infinite time

linkName_l = 'wrist_3_link_l'

objCOMinWrist3_l = 0
poseOfObjCOMInWrist3Frame_l = np.array([0., objCOMinWrist3_l, 0.])  # (x, y, z)

workspaceDof = 6
singleArmDof = 6
qDotCurrent_pre_l = np.zeros(singleArmDof)
qctrl_des_prev_l = np.zeros(singleArmDof)  # initial angular position of joints; joints in home configuration have zero angles.
dqctrl_des_prev_l = np.zeros(singleArmDof)  # initial angular velocity of joints; we usually start from rest

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

kp_a_lin = 100
kd_a_lin = kp_a_lin / 5

kp_a_ang = 100
kd_a_ang = kp_a_ang / 10


initialPoseOfObjInWorld_x_l = 0.3922
initialPoseOfObjInWorld_y_l = .191
initialPoseOfObjInWorld_z_l = .609

initialOrientationOfObj_roll_l = 0.
initialOrientationOfObj_pitch_l = 0.
initialOrientationOfObj_yaw_l = np.pi


pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']
CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
CSVFileName = pathToCSVFile + CSVFileName_plot_data

pathToArmURDFModels = config.bimanual_ur5_dic['urdfDirectory']
upperBodyModelFileName = config.bimanual_ur5_dic['urdfModelName_l']
loaded_model_l = rbdl.loadModel(pathToArmURDFModels + upperBodyModelFileName)

pathToTrajData = config.bimanual_ur5_dic['trajDataFileDirectory']
trajDataFileName_l = config.bimanual_ur5_dic['trajDataFileName_l']
trajDataFile = pathToTrajData + trajDataFileName_l


## create instances of publishers:
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

rospy.init_node('left_ur5_node')

def TrajEstimate(qdd):
    global dqctrl_des_prev_l, qctrl_des_prev_l

    Dt = dt
    qdot_des_now_l = dqctrl_des_prev_l + Dt*qdd  # numerical derivation
    dqctrl_des_prev_l = qdot_des_now_l

    q_des_now_l = qctrl_des_prev_l + Dt*qdot_des_now_l
    qctrl_des_prev_l = q_des_now_l

    q_des_now_filtered = []

    for i in q_des_now_l:
        i = np.mod(i, np.pi*2)
        if i > np.pi:
            i = i - np.pi*2
        q_des_now_filtered.append(i)

    return np.array(q_des_now_filtered), qdot_des_now_l


def PubTorqueToGazebo(torqueVec):
    """
    Publish torques to Gazebo (manipulate the object in a linear trajectory)
    """
    pub_shoulder_pan_l.publish(torqueVec[0])
    pub_shoulder_lift_l.publish(torqueVec[1])
    pub_elbow_l.publish(torqueVec[2])
    pub_wrist_1_l.publish(torqueVec[3])
    pub_wrist_2_l.publish(torqueVec[4])
    pub_wrist_3_l.publish(torqueVec[5])


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


def InverseDynamic(qCurrent_l, qDotCurrent_l, qDDotCurrent_l, qDes, qDotDes, qDDotDes):

    M = rbdl_methods.CalcM(loaded_model_l, qCurrent_l)
    h = rbdl_methods.CalcH(loaded_model_l, qCurrent_l, qDotCurrent_l)

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


def Task2Joint(qCurrent_l, qDotCurrent_l, qDDotCurrent_l, poseDesTraj, velDesTraj, accelDesTraj):

    currentPoseOfObj, RotMat = \
            rbdl_methods.CalcGeneralizedPoseOfPoint(loaded_model_l,
                                                    qCurrent_l,
                                                    linkName_l,
                                                    poseOfObjCOMInWrist3Frame_l)  # pose of 'wrist_3_link' frame in world/inertia frame
    currentOrientationOfObjInQuat = RotMatToQuaternion(RotMat)  # [w, q0, q1, q2], equ(145), attitude

    # desOrientationOfObjInQuat = EulerToUnitQuaternion_zyx(poseDesTraj[3:])  # equ(297), attitude, for circular trajectory
    # e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, desOrientationOfObjInQuat)  # for circular trajectory

    e_o = CalcOrientationErrorInQuaternion(currentOrientationOfObjInQuat, poseDesTraj[3:])  # equ(4), orientation planning, traj6d
    WriteToCSV(e_o, 'pose Error_angular', ['e_r', 'e_p', 'e_y'])

    # xDot_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning
    Pdot_ref, omega_ref = CalcGeneralizedVel_ref(currentPoseOfObj, poseDesTraj[:3], velDesTraj, e_o)  # equ(1), orientation planning

    currentVelOfObj = \
          rbdl_methods.CalcGeneralizedVelOfObject(loaded_model_l,
                                                  qCurrent_l,
                                                  qDotCurrent_l,
                                                  linkName_l,
                                                  poseOfObjCOMInWrist3Frame_l)

    translationalError = poseDesTraj[:3] - currentPoseOfObj
    # WriteToCSV(translationalError, 'pose Error_linear', ['e_x', 'e_y', 'e_z'])

    accelDesTraj_lin = accelDesTraj[:3] + kd_a_lin * (Pdot_ref - currentVelOfObj[:3]) + kp_a_lin * translationalError
    accelDesTraj_ang = accelDesTraj[3:] + kd_a_ang * (omega_ref - currentVelOfObj[3:]) + kp_a_ang * e_o
    accelDesTraj = np.concatenate((accelDesTraj_lin, accelDesTraj_ang))

    dJdq = rbdl_methods.CalcdJdq(loaded_model_l,
                                 qCurrent_l, qDotCurrent_l, qDDotCurrent_l,
                                 linkName_l, poseOfObjCOMInWrist3Frame_l)

    jac = rbdl_methods.Jacobian(loaded_model_l, qCurrent_l, linkName_l,
                                poseOfObjCOMInWrist3Frame_l)

    qDDotDes = pinv(jac).dot(accelDesTraj - dJdq)  # equ(21)

    qDes, qDotDes = TrajEstimate(qDDotDes)

    return qDes, qDotDes, qDDotDes


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
    xDes = initialPoseOfObjInWorld_x_l
    yDes = initialPoseOfObjInWorld_y_l
    zDes = initialPoseOfObjInWorld_z_l
    phiDes = initialOrientationOfObj_roll_l
    thetaDes = initialOrientationOfObj_pitch_l
    psyDes = initialOrientationOfObj_yaw_l
    poseTrajectoryDes = np.array([xDes, yDes, zDes, phiDes, thetaDes, psyDes])

    ## desired trajectory (velocity):
    xDotDes = 0.
    yDotDes = 0
    zDotDes = 0
    phiDotDes = 0.
    thetaDotDes = 0.
    psyDotDes = 0.
    velTrajectoryDes = np.array([xDotDes, yDotDes, zDotDes,
                                 phiDotDes, thetaDotDes, psyDotDes])

    ## desired trajectory (acceleration):
    xDDotDes = 0.
    yDDotDes = 0
    zDDotDes = 0
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
    global time_, qDotCurrent_pre_l, dt, time_gaz, time_gaz_pre

    ## Terminate the node after 't_end' seconds of simulation:
    if time_gaz >= t_end and finiteTimeSimFlag:
        print("\n\nshut down 'main_node'.\n\n")
        rospy.signal_shutdown('node terminated')

    q = data.position  # class tuple, in radians
    qDot = data.velocity  # in rad/s

    q = np.array(q, dtype=float)
    qDot = np.array(qDot, dtype=float)

    qCurrent_l = np.zeros(singleArmDof)
    qDotCurrent_l = np.zeros(singleArmDof)
    qDDotCurrent_l = np.zeros(singleArmDof)

    qCurrent_l[0] = q[4]  # shoulder_pan_joint_l
    qCurrent_l[1] = q[2]  # shoulder_lift_joint_l
    qCurrent_l[2] = q[0]  # elbow_joint_l
    qCurrent_l[3] = q[6]  # wrist_1_joint_l
    qCurrent_l[4] = q[8]  # wrist_2_joint_l
    qCurrent_l[5] = q[10]  # wrist_3_joint_l

    qDotCurrent_l[0] = qDot[4]
    qDotCurrent_l[1] = qDot[2]
    qDotCurrent_l[2] = qDot[0]
    qDotCurrent_l[3] = qDot[6]
    qDotCurrent_l[4] = qDot[8]
    qDotCurrent_l[5] = qDot[10]

    time_gaz = rospy.get_time()  # get gazebo simulation time (Sim Time)
    dt = time_gaz - time_gaz_pre + .001
    time_gaz_pre = time_gaz

    # qDDotCurrent_l = (qDotCurrent_l - qDotCurrent_pre_l) / dt
    # qDotCurrent_pre_l = qDotCurrent_l

    poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
                                              IterateThroughTraj6dData(time_gaz)
    # poseTrajectoryDes, velTrajectoryDes, accelTrajectoryDes = \
    #                                             TrajectoryGeneration(.2, time_gaz)

    ## detemine desired states of robot in joint-space: (qDDot_desired, ...)
    jointPoseDes, jointVelDes, jointAccelDes = Task2Joint(qCurrent_l,
                                                          qDotCurrent_l,
                                                          qDDotCurrent_l,
                                                          poseTrajectoryDes,
                                                          velTrajectoryDes,
                                                          accelTrajectoryDes)

    jointTau = InverseDynamic(qCurrent_l, qDotCurrent_l, qDDotCurrent_l,
                              jointPoseDes, jointVelDes, jointAccelDes)

    PubTorqueToGazebo(jointTau)

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

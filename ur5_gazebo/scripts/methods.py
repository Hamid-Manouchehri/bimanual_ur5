'''
Author: Hamid Manouchehri
co authors: Mohammad Shahbazi, Nooshin Koohli
Year: 2022-2023
'''
import pylab as pl
import math
import rbdl
import numpy as np
import csv
from scipy.spatial.transform import Rotation as R
from sys import path
path.insert(1, '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_description/config/')  # TODO: Insert dir of config folder of the package.
import config

plotLegend = ''
yLabel = ''
writeHeaderOnceFlag = True
workspaceDof = 6  # TODO

CSVFileName_plot_data = config.bimanual_ur5_dic['CSVFileName']
pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']


def CalcM(model, q):
    """Compute join-space inertia matrices of arms."""

    M = np.zeros((model.q_size, model.q_size))  # (6*6)
    rbdl.CompositeRigidBodyAlgorithm(model, q, M, True)

    return M


def CalcH(model, q, qdot, qddot):
    """Compute centrifugal, coriolis and gravity force terms."""
    q = np.array(q, dtype=float)
    qdot = np.array(qdot, dtype=float)
    tau = np.zeros(model.q_size)

    # qdot = np.zeros(6)  # uncomment for 'PD control with Gravity compensation'
    qddot = np.zeros(6)
    rbdl.InverseDynamics(model, q, qdot, qddot, tau)  # (1*6)

    H = tau

    return H


def GeneralizedPoseOfObj(model, linkName, q):

    # objCOMinWrist3 = 0.1823
    objCOMinWrist3 = 0.
    poseOfObjInHandFrame = np.asarray([0., objCOMinWrist3, 0.])

    ## link names: [base_link, shoulder_link, upper_arm_link, forearm_link,
    ## wrist_1_link, wrist_2_link, wrist_3_link]
    poseOfObj = rbdl.CalcBodyToBaseCoordinates(model, q,
                                               model.GetBodyId(linkName),
                                               poseOfObjInHandFrame)

    rotationMatOfBox = rbdl.CalcBodyWorldOrientation(model, q,
                                                     model.GetBodyId(linkName))

    return poseOfObj, rotationMatOfBox


def Jacobian(model, linkName, q):

    jc = np.zeros((workspaceDof, model.q_size))  # (6*6): due to whole 'model' and 'q' are imported.

    # objCOMinWrist3 = 0.1823
    objCOMinWrist3 = 0.
    poseOfObjInHandFrame = np.asarray([0., objCOMinWrist3, 0.])

    rbdl.CalcPointJacobian6D(model, q, model.GetBodyId(linkName),
                             poseOfObjInHandFrame, jc)  # (6*6)

    jc_r = jc[3:, :]
    jc_l = jc[:3, :]
    jc = np.vstack((jc_r, jc_l))

    return jc


def CalcGeneralizedVelOfObject(model, linkName, q, qdot):
    """
    Calculate generalized velocity of the object via the right-hand ...
    kinematics in base frame.
    """
    # objCOMinWrist3 = 0.1823
    objCOMinWrist3 = 0.
    poseOfObjInHandFrame = np.asarray([0., objCOMinWrist3, 0.])

    generalizedVelOfObj = rbdl.CalcPointVelocity6D(model, q, qdot,
                                                   model.GetBodyId(linkName),
                                                   poseOfObjInHandFrame)

    translationalVelOfObj = generalizedVelOfObj[3:6]
    angularVelOfObj = generalizedVelOfObj[:3]

    generalizedVelOfObj = np.hstack((translationalVelOfObj, angularVelOfObj))

    return generalizedVelOfObj



def CalcdJdq(model, linkName, q, qdot, qddot):
    """Compute linear acceleration of a point on body."""

    # objCOMinWrist3 = 0.1823
    objCOMinWrist3 = 0.
    poseOfObjInHandFrame = np.asarray([0., objCOMinWrist3, 0.])
    qddot = np.zeros(6)

    bodyAccel = rbdl.CalcPointAcceleration6D(model, q, qdot, qddot,
                                             model.GetBodyId(linkName),
                                             poseOfObjInHandFrame)  # (1*3)

    return bodyAccel

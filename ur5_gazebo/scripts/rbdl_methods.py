'''
Author: Hamid Manouchehri
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

workspaceDoF = 6
singleArmDoF = 6


def CalcM(model, q):
    """Compute joint-space inertia matrices of arms."""

    M = np.zeros((model.q_size, model.q_size))  # (6*6), for each arm
    rbdl.CompositeRigidBodyAlgorithm(model, q, M, True)

    return M


def CalcH(model, q, qdot):
    """Compute centrifugal, coriolis and gravity force terms."""
    q = np.array(q, dtype=float)
    qdot = np.array(qdot, dtype=float)
    qddot = np.zeros(model.dof_count)
    tau = np.zeros(model.dof_count)

    rbdl.InverseDynamics(model, q, qdot, qddot, tau)  # (1*6), for each arm
    H = tau

    return H


def CalcGeneralizedPoseOfPoint(model, q, linkName, pointLocal):
    """
    Calculate generalized position of a point in world frame:
    Note: both wrists have same position for their tips in local frames (forward kinematics).
    (poseOfTipOfWrist3InWrist3Frame)
    """
    poseOfPoint = rbdl.CalcBodyToBaseCoordinates(model, q,
                                                 model.GetBodyId(linkName),
                                                 pointLocal)  # (1*3)
    rotationMatOfBody = \
                    rbdl.CalcBodyWorldOrientation(model, q,
                                                  model.GetBodyId(linkName))

    return poseOfPoint, rotationMatOfBody


def Jacobian(model, q, linkName, pointLocal):

    jc = np.zeros((workspaceDoF, model.dof_count))  # (6*6)

    rbdl.CalcPointJacobian6D(model, q, model.GetBodyId(linkName), pointLocal, jc)  # (6*6)

    jc_angular = jc[:3, :]  # (3*6)
    jc_linear = jc[3:, :]  # (3*6)
    jc = np.vstack((jc_linear, jc_angular))  # (6*6)

    return jc


def CalcGeneralizedVelOfObject(model, q, qdot, linkName, pointLocal):
    """
    Calculate generalized velocity of the object via the right-hand ...
    kinematics in base frame (forward kinematics).
    """
    generalizedVelOfObj = rbdl.CalcPointVelocity6D(model, q, qdot,
                                                   model.GetBodyId(linkName),
                                                   pointLocal)

    velOfObj_angular = generalizedVelOfObj[:3]
    velOfObj_linear = generalizedVelOfObj[3:6]

    generalizedVelOfObj = np.concatenate((velOfObj_linear, velOfObj_angular))

    return generalizedVelOfObj


def CalcdJdq(model, q, qdot, qddot, linkName, pointLocal):
    """Compute linear acceleration of a point on body."""
    # qddot = np.zeros(model.dof_count)
    bodyAccel = rbdl.CalcPointAcceleration6D(model, q, qdot, qddot,
                                             model.GetBodyId(linkName),
                                             pointLocal)  # (1*6)

    bodyAccel_angular = bodyAccel[:3]
    bodyAccel_linear = bodyAccel[3:]
    bodyAccel = np.concatenate((bodyAccel_linear, bodyAccel_angular))

    return bodyAccel

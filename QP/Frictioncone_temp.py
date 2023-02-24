#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 15 15:31:09 2017

@author: user
"""
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')


mu = .7
t1 = np.linspace(-20., 20., 2000)
t2 = np.linspace(-20., 20., 2000)
#t2 = np.array([20]*200)

#t = np.linspace(0, 2*np.pi, 100)
#t1 = 5*np.cos(t)
#t2 = 5*np.sin(t)

#n = np.linspace(0, 10, 100)
t1, t2 = np.meshgrid(t1, t2)

n = 1./mu * np.sqrt(t1**2 + t2**2)

#
#plt.scatter(t1, t2, n)

ind1 = n<=2.65
ind2 = n>=2.6
ind = ind1 & ind2

#plt.scatter(t1[ind], t2[ind], zs=n[ind])
plt.plot(t1[ind], t2[ind], 'o')



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

def LinearizeFCone(mu, N):
    C = np.zeros((N+1, 3))
#    d = np.zeros(N+1)
    for i in range(N):
        C[i, :] = [aj(mu, N, i), bj(mu, N, i), cj(mu, N, i)]
#        d[i] = cj(mu, N, i)
    C[N, 2] = 1
    return C

N = 3
C = LinearizeFCone(mu, N)

#zs = []
for i in range(N):
#    print i
    zss = - 1/C[i, 2]*(C[i, 0]*t1 + C[i, 1]*t2)
#    zs.append(zss)
    ind1 = zss<=2.65
    ind2 = zss>=2.6
    ind = ind1 & ind2
    print i, len(ind.nonzero()[0])
#    plt.scatter(t1[ind], t2[ind], zs=zss[ind], marker='^', c = 'r')

    plt.plot(t1[ind], t2[ind], '*')
#    if i in [0, 1, 2]:
#        plt.scatter(t1, t2, zs=zss, marker='^', c = 'r')

#ind1 = zss<=2.61
#ind2 = zss>=2.6
#ind = ind1 & ind2


#tt = np.linspace(-5, 5, 100)
#nn = 1/mu*np.sqrt(20**2 + tt**2)
#
#plt.plot(tt, nn)

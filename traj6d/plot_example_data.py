#!/usr/bin/env python2

import numpy as np
import matplotlib.pyplot as plt
import pyquaternion as qt

trajDataFileName = './trajectory_data.txt'

def computeNumericalDiff(x, dt):
    m, n = x.shape
    dx = []

    for i, row in enumerate(x):
        if i == 0:
            up3 = x[i+3, :]
            up2 = x[i+2, :]
            up = x[i+1, :]
            down = x[i, :]
            dx.append((1/3*up3 - 3/2*up2 + 3*up - 11/6*down)/dt)
        elif i == m - 1:
            up = x[i, :]
            down = x[i-1, :]
            down2 = x[i-2, :]
            down3 = x[i-3, :]
            dx.append((-1/3*down3 + 3/2*down2 - 3*down + 11/6*up)/dt)
        else:
            up = (x[i, :] + x[i+1, :])/2
            down = (x[i, :] + x[i-1, :])/2
            dx.append((up - down)/dt)

    return np.array(dx)

def testNumericalDiff(q, dt):
    dq = computeNumericalDiff(q, dt)
    ddq = computeNumericalDiff(dq, dt)

    w, dw = [], []
    for i, qq in enumerate(q):
        quat = qt.Quaternion(qq)
        dquat = qt.Quaternion(dq[i, :])
        ddquat = qt.Quaternion(ddq[i, :])

        wi = 2*dquat*quat.inverse
        dwi = 2*ddquat*quat.inverse - wi*dquat*quat.inverse

        w.append(wi.vector)
        dw.append(dwi.vector)

    return np.array(w), np.array(dw)




t, p, dp, ddp, quat, w, dw = [], [], [], [], [], [], []

with open(trajDataFileName, 'r') as f:
    lines = f.readlines()
    for line in lines:
        line = line.split(' ')
        l = [x for x in line if x!= '']

        t.append(l[0])

        p.append(l[1:4])  # linear position (x, y, z)
        dp.append(l[4:7])  # linear velocity
        ddp.append(l[7:10])  # linear acceleration

        quat.append(l[10:14])  # angular position (quaternion (w, q1, q2, q3))
        w.append(l[14:17])  # angular velocity (omega)
        dw.append(l[17:])  # angular acceleration (alpha)

## cast string data to float:
t = np.array(t, dtype=float)
p = np.array(p, dtype=float)
dp = np.array(dp, dtype=float)
ddp = np.array(ddp, dtype=float)
quat = np.array(quat, dtype=float)
w = np.array(w, dtype=float)
dw = np.array(dw, dtype=float)


dt = t[1] - t[0]
w_num = w[:-2, :]*0
dw_num = dw[:-2, :]*0
w_num, dw_num = testNumericalDiff(quat[:-2, :], dt)


plt.subplot(321)
plt.plot(t[:-2], p[:-2, :])
[plt.plot(t[-2], p[-2, i], "*") for i in range(3)]
[plt.plot(t[-1], p[-1, i], "*") for i in range(3)]
plt.legend(['x', 'y', 'z'])
plt.title('linear position')

plt.subplot(323)
plt.plot(t[:-2], dp[:-2, :])
[plt.plot(t[-2], dp[-2, i], "*") for i in range(3)]
[plt.plot(t[-1], dp[-1, i], "*") for i in range(3)]
plt.legend(['v_x', 'v_y', 'v_z'])
plt.title('linear velocity')

plt.subplot(325)
plt.plot(t[:-2], ddp[:-2, :])
[plt.plot(t[-2], ddp[-2, i], "*") for i in range(3)]
[plt.plot(t[-1], ddp[-1, i], "*") for i in range(3)]
plt.legend(['a_x', 'a_y', 'a_z'])
plt.title('linear acceleration')

plt.subplot(322)
plt.plot(t[:-2], quat[:-2, :])
[plt.plot(t[-2], quat[-2, i], "*") for i in range(4)]
[plt.plot(t[-1], quat[-1, i], "*") for i in range(4)]
plt.legend(['w', 'q1', 'q2', 'q3'])
plt.title('angular position (quaternion)')

plt.subplot(324)
plt.plot(t[:-2], w_num, '--')
plt.plot(t[:-2], w[:-2, :])
[plt.plot(t[-2], w[-2, i], "*") for i in range(3)]
[plt.plot(t[-1], w[-1, i], "*") for i in range(3)]
plt.legend(['w_x', 'w_y', 'w_z'])
plt.title('angular velocity')

plt.subplot(326)
plt.plot(t[:-2], dw_num, '--')
plt.plot(t[:-2], dw[:-2, :])
[plt.plot(t[-2], dw[-2, i], "*") for i in range(3)]
[plt.plot(t[-1], dw[-1, i], "*") for i in range(3)]
plt.legend(['alpha_x', 'alpha_y', 'alpha_z'])
plt.title('angular acceleration')
plt.tight_layout()  # fix subplot title overlap.

plt.show()

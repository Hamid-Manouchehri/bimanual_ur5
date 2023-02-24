#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 22:29:50 2017

@author: user
"""

import matplotlib.pyplot as plt

plt.rcParams['ps.useafm'] = True
plt.rcParams['pdf.use14corefonts'] = True
plt.rcParams['text.usetex'] = True

font_label = 14

### Plot for object position

plt.figure(figsize=(5, 3.5))

plt.subplot(311)
plt.plot(t, q[:, 6], color='k', lw=1.5)
plt.plot(t, x_des[:, 0], '--', color='r', lw=1.5)
plt.ylabel(r"$x\, (\mathrm{m})$", fontsize=font_label)
#ax1 = plt.axes()
#ax1.yaxis.grid()
plt.grid(1)

plt.subplot(312)
plt.plot(t, q[:, 7], color='k', lw=1.5)
plt.plot(t, x_des[:, 1], '--', color='r', lw=1.5)
plt.ylabel(r"$y\, (\mathrm{m})$", fontsize=font_label)
plt.legend(['Actual', 'Desired'], loc='upper center', fontsize=12)
#ax2 = plt.axes()
#ax2.yaxis.grid()
plt.grid(1)


plt.subplot(313)
plt.plot(t, np.rad2deg(q[:, 8]), color='k', lw=1.5)
plt.plot(t, np.rad2deg(x_des[:, 2]), '--', color='r', lw=1.5)
plt.ylabel(r"$\theta\, (\mathrm{degree})$", fontsize=font_label)
plt.xlabel(r"$\mathrm{Time} \, (\mathrm{s})$", fontsize=font_label)
plt.ylim(-70, 70)
#ax3 = plt.axes()
#ax3.yaxis.grid()
plt.tight_layout()
plt.grid(1)
#plt.savefig('PositionModelBasedIROS.pdf')


### Plot for torques when minimizing torque or force

plt.figure(figsize=(5, 2.5))

plt.plot(t, u_a[:, 0], color='k', lw=1.5)
plt.plot(t, u_a[:, 1], '--',color='r', lw=1.5)
plt.plot(t, u_a[:, 2], ':',color='b', lw=1.5)
plt.ylabel(r"$\tau\, (\mathrm{Nm})$", fontsize=font_label)
plt.xlabel(r"$\mathrm{Time} \, (\mathrm{s})$", fontsize=font_label)
plt.grid(1)
leg = plt.legend([r'$\tau_1, \mathrm{shoulder}$', r'$\tau_2, \mathrm{elbow}$',
            r'$\tau_3, \mathrm{wrist}$'], loc='lower right', fontsize=12)

leg.get_frame().set_alpha(0.6)

plt.ylim([-10, 8.3])
plt.yticks([-10,  -8,  -6,  -4,  -2,   0,   2,   4,   6, 8])
#plt.savefig('CommandsMinTorque.pdf')
#plt.savefig('CommandsMinForce.pdf')




### plot for RMS of motion inducing
#rms = np.sqrt(np.mean(F_motion*F_motion, axis=0))


n_groups = 6

rms_u = (1.28302988,  0.58724289,  0.08008031,  1.28378465,  0.58567238,
        0.08008031)
#std_men = (2, 3, 4, 1, 2)

rms_F = (1.2829954 ,  0.58719128,  0.08007388,  1.28374209,  0.58562973,
        0.08007388)
#std_women = (3, 5, 2, 3, 3)

fig, ax = plt.subplots(figsize=(5, 1.9))

index = np.arange(n_groups)
bar_width = 0.35

error_config = {'ecolor': '0.3'}

rects1 = plt.bar(index, rms_u, bar_width,
                 alpha=.4,
                 color='b',
#                 yerr=std_men,
                 error_kw=error_config,
                 label='case 1')

rects2 = plt.bar(index + bar_width, rms_F, bar_width,
                 alpha=.3,
                 color='r',
#                 yerr=std_women,
                 error_kw=error_config,
                 label='case 2')

#plt.xlabel('Group')
plt.ylabel(r'$\mathrm{RMS}\, \lambda_\mathrm{m}\, (\mathrm{N}\, \
           \mathrm{or}\, \mathrm{Nm})$', fontsize=font_label)
#plt.title('Generalized Internal Force by Grasping Points and Direction')
plt.xticks(index + bar_width / 2, (r'$\mathrm{a}_x$', r'$\mathrm{a}_y$', \
                                   r'$\mathrm{a}_\theta$', r'$\mathrm{b}_x$', \
                                   r'$\mathrm{b}_y$', r'$\mathrm{b}_\theta$'), fontsize=font_label)
plt.legend(fontsize=12)

plt.tight_layout()

#plt.savefig('RMSMotion.pdf')


### plot for RMS of internal force
#rms = np.sqrt(np.mean(F_motion*F_motion, axis=0))


n_groups = 6

rms_u = (1.26599625,  3.50145689,  0.95421799,  1.26599625,  3.50145689,
        0.9577395)
#std_men = (2, 3, 4, 1, 2)

rms_F = (0.18443698,  0.15401907,  0.06277963,  0.18443698,  0.15401907,
        0.06349738)
#std_women = (3, 5, 2, 3, 3)

fig, ax = plt.subplots(figsize=(5, 1.9))

index = np.arange(n_groups)
bar_width = 0.35

#opacity = 0.4
error_config = {'ecolor': '0.3'}

rects1 = plt.bar(index, rms_u, bar_width,
                 alpha=.4,
                 color='b',
#                 yerr=std_men,
                 error_kw=error_config,
                 label='case 1')

rects2 = plt.bar(index + bar_width, rms_F, bar_width,
                 alpha=.3,
                 color='r',
#                 yerr=std_women,
                 error_kw=error_config,
                 label='case 2')

#plt.xlabel('Group')
plt.ylabel(r'$\mathrm{RMS}\, \lambda_\mathrm{s}\, (\mathrm{N}\,\
           \mathrm{or}\, \mathrm{Nm})$', fontsize=font_label)
#plt.title('Generalized Internal Force by Grasping Points and Direction')
plt.xticks(index + bar_width / 2, (r'$\mathrm{a}_x$', r'$\mathrm{a}_y$', \
                                   r'$\mathrm{a}_\theta$', r'$\mathrm{b}_x$', \
                                   r'$\mathrm{b}_y$', r'$\mathrm{b}_\theta$'), fontsize=font_label)
leg = plt.legend(fontsize=12)
leg.get_frame().set_alpha(0.6)

plt.tight_layout()
#plt.savefig('RMSSqueeze.pdf')



### Plot model free position error

plt.figure(figsize=(5, 2.5))

plt.plot(t, x_des[:, 0] - q[:, 6], color='k', lw=1.5)
plt.plot(t, x_des[:, 1] - q[:, 7], '--',color='r', lw=1.5)
plt.plot(t, x_des[:, 2] - q[:, 8], ':',color='b', lw=1.5)
plt.ylabel(r"$\mathrm{Position\, error}\, (\mathrm{m}\, \mathrm{or}\, \mathrm{rad})$", fontsize=font_label)
plt.xlabel(r"$\mathrm{Time} \, (\mathrm{s})$", fontsize=font_label)
plt.grid(1)
leg = plt.legend([r'$x$', r'$y$',
            r'$\theta$'], loc='upper right', fontsize=12)
leg.get_frame().set_alpha(0.5)

plt.tight_layout()
#plt.savefig('ModelFreePosition.pdf')



### Plot model free force error

plt.figure(figsize=(5, 2.5))

plt.plot(t, F_a[:, 0] - F_a_p[:, 0], color='k', lw=1.5)
plt.plot(t, F_a[:, 1] - F_a_p[:, 1], '--',color='r', lw=1.5)
plt.plot(t, F_a[:, 2] - F_a_p[:, 2], ':',color='b', lw=1.5)
plt.ylabel(r"$\mathrm{Force\, error}\, (\mathrm{N}\, \mathrm{or}\, \mathrm{Nm})$", fontsize=font_label)
plt.xlabel(r"$\mathrm{Time} \, (\mathrm{s})$", fontsize=font_label)
plt.grid(1)
leg = plt.legend([r'$x$', r'$y$',
            r'$\theta$'], loc='lower right', fontsize=12)
leg.get_frame().set_alpha(0.5)

plt.tight_layout()
#plt.savefig('ModelFreeForce.pdf')

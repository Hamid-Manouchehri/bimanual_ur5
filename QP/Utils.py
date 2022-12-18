# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 16:46:47 2016

@author: user

"""

#import matplotlib
#matplotlib.use('GTKAgg') 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rbdl
import numpy as np
from matplotlib import patches
import matplotlib as mpl


def Anim_bimanual(model, Q, time, tip_point, l_0):

    plt.close('all')
    
    def Rotation(x):
        return np.array([np.cos(x), - np.sin(x), np.sin(x), \
                         np.cos(x)]).reshape(2, 2)
    

    
    def update_j(rbdl, model, Q, i, tip_point, which):
        if which == 'a': q = Q[i, :3]
        elif which == 'b': q = Q[i, 3:6]
        p2 = rbdl.CalcBodyToBaseCoordinates(model, q, 2, np.zeros(3))
        p3 = rbdl.CalcBodyToBaseCoordinates(model, q, 3, np.zeros(3))
        p4 = rbdl.CalcBodyToBaseCoordinates(model, q, 3, tip_point)
        return p2, p3, p4
    
    
    fig = plt.figure(figsize=(6, 3))
    fig.set_dpi(100)
    ax = fig.add_subplot(111)
    ax.grid()
    ax.set_xlim([-1.1, 1.1])
    ax.set_ylim([-.15, .95])
#    ax.axis('equal')
    
    line_a, = ax.plot([], [], 'o-', lw=2)
    line_b, = ax.plot([], [], 'o-', lw=2)
    line_c, = ax.plot([], [], lw=15, color='.5', alpha=.7)
    point_o,  = ax.plot([], [], 'o', markersize=7)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    d = .7
    D = d/2
    l = np.sqrt((d/2)**2 + (D/2)**2)
    alpha = np.arctan2(d/2, D/2)
    patch = plt.Rectangle((Q[0, 4] - d/2, Q[0, 5] - D/2), d, D, fc='y')
    
#    def init():
##       patch.center = (5, 5)
#        ax.add_patch(patch)
#        return patch,

    def animate(i, rbdl, model, Q, tip_point, time, l_0):
        p2_a, p3_a, p4_a = update_j(rbdl, model[0], Q, i, tip_point[0], 'a')
        p2_b, p3_b, p4_b = update_j(rbdl, model[1], Q, i, tip_point[1], 'b')
        
        
        x_c = [p4_a[0], p4_b[0]]
        y_c = [p4_a[1], p4_b[1]]
        line_c.set_data(x_c, y_c)
        
        
        x_a = [l_0/2, p2_a[0], p3_a[0], p4_a[0]]
        y_a = [0., p2_a[1], p3_a[1], p4_a[1]]
        line_a.set_data(x_a, y_a)
        
        x_b = [-l_0/2, p2_b[0], p3_b[0], p4_b[0]]
        y_b = [0., p2_b[1], p3_b[1], p4_b[1]]
        line_b.set_data(x_b, y_b)
        
        point_o.set_data([Q[i, 6]], [Q[i, 7]])
        
#        xx, yy = Q[i, 4] - d/2, Q[i, 5] - D/2
#        x, y = np.dot(Rotation(-Q[i, 6]), [xx, yy])
#        patch.set_xy((x, y))
#        
#            
#        ts = ax.transData
#        coords = ts.transform([Q[i, 4], Q[i, 5]])
#        tr = mpl.transforms.Affine2D().rotate_deg_around(\
#                                    coords[0], coords[1], np.rad2deg(Q[i, 6]))
#        t2= ts + tr
        
#        t2 = mpl.transforms.Affine2D().rotate_deg(30) + ax.transData
#        patch.set_transform(t2)
        
        
        
        time_text.set_text(time_template % (time[i]))
        return line_c, line_a, line_b, point_o, time_text,
        
    args = (rbdl, model, Q, tip_point, time, l_0)

    ani = animation.FuncAnimation(fig, animate, len(Q),\
                                  fargs=args,
                              interval=1, blit=False)
    
    
    return ani




    
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 11 20:59:02 2021

@author: ind
"""

import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

goal = np.array([0.0 ,20.0, 2.0])
ip = np.array([0.0, 0.0, 1.0])
nplanes = 6
d = (6.0-2.0)/(nplanes-1)
print d
spacing = 0.75
xhalfangle = int(0.5*70);
yhalfangle = int(0.5*45);

ensemble, dist2goal = [], []

for i in range(nplanes):
    R = 2 + d*i
    arcx = 2*R*np.tan(xhalfangle*np.pi/180)
    arcy = 2*R*np.tan(yhalfangle*np.pi/180)
    print '1', arcx, arcy
    arcx = 2*R*xhalfangle*np.pi/180
    arcy = 2*R*yhalfangle*np.pi/180
    print '2', arcx, arcy
    Nx = int(arcx/spacing) + 1
    Ny = int(arcy/spacing) + 1
    vel = 6.0
    for j in range(Nx+1):
        thetax = -xhalfangle + (70.0/Nx)*j
        for k in range(Ny+1):            
            thetay = -yhalfangle + (45.0/Ny)*k
            point = np.array([R*np.cos(thetax*np.pi/180)*np.cos(thetay*np.pi/180), \
                              R*np.sin(thetax*np.pi/180)*np.cos(thetay*np.pi/180), ip[2]+R*np.sin(thetay*np.pi/180)])
            
            if point[2]>0.25:
                dist2goal.append(np.linalg.norm(goal - point))
                ensemble.append(point)
                              
fig = plt.figure(figsize = (10, 8))
ax = fig.add_subplot(111, projection='3d')
p = zip(*ensemble)
ax.scatter(p[0], p[1], p[2], c='r')
n = dist2goal.index(min(dist2goal))
ax.scatter(ensemble[n][0], ensemble[n][1], ensemble[n][2], c='k')
#for i in range(len(dist2goal)):
#    print i, ensemble[i], dist2goal[i]
print 'total points', len(ensemble)
plt.show()
plt.tight_layout()  
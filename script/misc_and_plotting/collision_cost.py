# -*- coding: utf-8 -*-
"""
Created on Sat Jan 16 23:41:12 2021

@author: ind
"""

import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy import ndimage

robot_radius = 0.35
SR = 3
factor = (1 + SR*SR*SR*SR)/(SR*SR*SR*SR)

dd = np.linspace(0.35, 6, 101)

cost = factor * ((dd-robot_radius)**2 - SR*SR)*((dd-robot_radius)**2 - SR*SR)/ \
                            (1 + ((dd-robot_radius)**2 - SR*SR)*((dd-robot_radius)**2 - SR*SR))

cost = factor * ((dd)**2 - SR*SR)*((dd)**2 - SR*SR)/ \
                            (1 + ((dd)**2 - SR*SR)*((dd)**2 - SR*SR))

cost = np.exp(-dd)                            
for i in range(len(cost)):
    #if dd[i] - robot_radius > SR:
    if dd[i] > SR:
        cost[i] = 0.0                            
plt.figure()
plt.plot(dd, cost, 'r')
plt.tight_layout()
plt.show()

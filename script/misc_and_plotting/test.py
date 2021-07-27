# -*- coding: utf-8 -*-
"""
Created on Sat Dec  5 10:17:46 2020

@author: ind
"""

import numpy as np 
import matplotlib.pyplot as plt
tt = np.linspace(-15, 15, 101)
a1 = -1; b1 = -3; c1 = -1; d1 = -8
a2 = 0.5; b2 = 0.5; c2 = 0.5; d2 = 1

T1, T2, T3, T = [], [], [], []
for t in tt: 
    T1.append(a1*t**3 + b1*t**2 + c1*t + d1)
    T2.append(a2*t**3 + b2*t**2 + c2*t + d2)
    T.append(T1[-1] + T2[-1])

print min(T1), min(T2), min(T)  
plt.plot(tt, np.array(T1),'r', linewidth = 2, label = 'T1')
plt.plot(tt, np.array(T2), 'g', linewidth = 2, label = 'T2')
plt.plot(tt, np.array(T), 'k', linewidth = 2, label = 'T')
plt.legend(loc = 1, fontsize = 14)
plt.show()
    
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 15:09:14 2020

@author: ind
"""


import numpy as np 
from matplotlib import pyplot as plt 
from scipy import optimize

def func(x, a, b, c):
    return a * np.exp(-b * x) + c
    
v = [1,2,3,4,5,6,7,8,9,10]
kc = [0.0005, 0.05, 1, 50, 500, 5000, 1e5, 5e5, 8e5, 1e6]
kf1 = [7.8074e-5*i**10.1143 for i in v]
#kf2 = [0.000668584*np.exp(2.04091035*i) for i in v]
popt, pcov = optimize.curve_fit(func, v, kc)
print popt
kf2 = [popt[0]*np.exp(-popt[1]*i)+popt[2] for i in v]
#kf3 = [8.8074e-5*i**10.3143 for i in v]
#kf4 = [8.8074e-5*i**10.4143 for i in v]
#kf5 = [8.8074e-5*i**10.5143 for i in v]
#kf6 = [8.8074e-5*i**10.6143 for i in v]
fig = plt.figure(figsize = (5, 4))
plt.scatter(kc, v, c = 'g')
plt.plot(kf1, v, 'g', linewidth = 2, linestyle = '--', label = 'Fitted power curve')
plt.ticklabel_format(axis='x', style='sci', scilimits=(4,4))
#plt.plot(v, kf2, 'b', linewidth = 1, label = 'fitted power curve')
#plt.plot(v, kf3, 'r', linewidth = 1, label = 'fitted power curve')
#plt.plot(v, kf4, 'm', linewidth = 1, label = 'fitted power curve')
#plt.plot(v, kf5, 'c', linewidth = 1, label = 'fitted power curve')
#plt.plot(v, kf6, 'k', linewidth = 1, label = 'fitted power curve')

plt.legend(loc = 4, fontsize =14)
plt.ylabel('Maximum Speed (m/s)', fontsize = 16)
plt.xlabel('k', fontsize = 16)
plt.xticks(size = 12);plt.yticks(size = 12)
plt.tight_layout()
plt.show()
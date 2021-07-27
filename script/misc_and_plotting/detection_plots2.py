    # -*- coding: utf-8 -*-
"""
Created on Sun Mar 21 13:04:41 2021

@author: ind
"""
import numpy as np 
import os
import matplotlib.pyplot as plt 
from mpl_toolkits.axes_grid1.inset_locator import inset_axes


        
path = '/home/ind/data_collection/TRO_dataset/detection/source/'
for subdir, dirs, files in os.walk(path, topdown=False):
    lrt_data = []
    for j in sorted(files):
        _file = open(path+j, 'r')
        for line in _file: 
            temp1 = line.split(", ")
            data = [float(i) for i in temp1]
            if data[9]>data[10] and data[10] != 0.0:
                break
        lrt_data.append([data[9], data[10]])


lrt_data = zip(*lrt_data)


path = '/home/ind/data_collection/TRO_dataset/detection/without_source/'
for subdir, dirs, files in os.walk(path, topdown=False):
    lrt_data1 = []
    for j in sorted(files):
        _file = open(path+j, 'r')
        for line in _file: 
            temp1 = line.split(", ")
            data = [float(i) for i in temp1]
            #if data[3]> 0.0:
            #    break
        lrt_data1.append([data[9], data[10]])


lrt_data1 = zip(*lrt_data1)
print lrt_data1




fig, ax = plt.subplots()
#x = range(26); y = range(26)
x = np.linspace(-0.5, 17, 100)
y = np.linspace(-0.5, 17, 100)
plt.plot(np.log(x), np.log(y), color='k', linewidth = 2, label = '$line \enspace L_T = \gamma $')
plt.scatter(np.log(lrt_data[1]), np.log(lrt_data[0]), marker='o', color = 'g')
#plt.scatter(lrt_data1[1], lrt_data1[0], marker='o', color = 'r')
plt.scatter(np.log(lrt_data1[1]), np.log(lrt_data1[0]), marker='s', color = 'r')

plt.text(np.log(0.5), np.log(25.0), '$L_T \geq \gamma$', fontsize = 14, transform=ax.transData)
plt.text(np.log(3.0), np.log(0.1), '$L_T < \gamma$', fontsize = 14, transform=ax.transData)

plt.legend(loc = 2, fontsize =14)
plt.ylabel('$\log(L_T)$', fontsize = 16)
plt.xlabel('$\log(\gamma)$', fontsize = 16)
plt.xticks(size = 12);plt.yticks(size = 12)
plt.tight_layout()


plt.show()
        
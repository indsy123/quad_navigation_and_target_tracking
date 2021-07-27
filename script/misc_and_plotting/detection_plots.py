    # -*- coding: utf-8 -*-
"""
Created on Sun Mar 21 13:04:41 2021

@author: ind
"""
import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.axes_grid1.inset_locator import inset_axes


        
loc = '/home/ind/data_collection/TRO_dataset/detection/source/detection_data16.txt'
_file = open(loc, 'r')

#plt.figure()        
data = []
for line in _file: 
        temp1 = line.split(", ")
        temp2 = [float(i) for i in temp1]
        data.append(temp2)
time_dist_mu = [[i[0], i[1], i[2]] for i in data]
time_dist_mu  = zip(*time_dist_mu)
#plt.plot(time_dist_mu[0], time_dist_mu[1], color='g', linestyle = '--', linewidth = 2, label = 'Distance')
#plt.plot(time_dist_mu[0], time_dist_mu[2], color='g', linewidth = 2, label = '$\mu$')

#plt.legend(loc = 1, fontsize =14)
#plt.ylabel('Distance to Source and $\mu$', fontsize = 16)
#plt.xlabel('Time (sec)', fontsize = 16)
#plt.xticks(size = 12);plt.yticks(size = 12)
#plt.tight_layout()

fig, ax1 = plt.subplots()
time_prob = [[i[0], i[3], i[4], i[5]] for i in data]
time_prob  = zip(*time_prob)
#plt.plot(time_prob[0], time_prob[1], color='g', linewidth = 2, label = 'p')
ax1.plot(time_prob[0], time_prob[2], color='g', linestyle = '--', linewidth = 2, label = '$P_{FA}$')
ax1.plot(time_prob[0], time_prob[3], color='g', linestyle = ':', linewidth = 2, label = '$P_{M}$')

ax1.text(1.5, 0.1, '$L_T < \gamma$', fontsize = 14, transform=ax1.transData)
ax1.text(32.5, 0.5, '$L_T \geq \gamma$', fontsize = 14, transform=ax1.transData)
ax1.text(25.5, 0.15, '$T = 24.5sec$', fontsize = 14, transform=ax1.transData)
ax1.scatter(time_prob[0][73], 0.2, c = 'k')

plt.legend(loc = 2, fontsize =14)
plt.ylabel('Probability Bounds', fontsize = 16)
plt.xlabel('Time (sec)', fontsize = 16)
plt.xticks(size = 12);plt.yticks(size = 12)

ax2 = ax1.twinx() 
#plt.figure()
detection_test = [[i[0], i[9], i[10]] for i in data]
detection_test  = zip(*detection_test)
lr = detection_test[1]
gamma = detection_test[2]
gamma = [i if i !=0 else 1.0 for i in gamma]
ratio = [detection_test[1][i]/detection_test[2][i] if detection_test[2][i] != 0 else 1.0 for i in range(len(detection_test[0]))]
print 'time for vertical line', time_prob[0][73], ratio[73], np.log(ratio[73])
ax2.plot(detection_test[0][57:], np.log(ratio)[57:] , color='g', linewidth = 2, label = '$\log(L_T/ \gamma)$')
ax2.hlines(time_prob[0][0], time_prob[0][-1], 0, 'k', linewidth=2, linestyle = '-.')
#plt.plot(detection_test[0][0:75], detection_test[2][0:75], color='g', linestyle = '--', linewidth = 2, label = '$\gamma$')

plt.legend(loc = 1, fontsize =14)
#plt.ylabel('$\frac{L_R}{\gamma}$', fontsize = 16)
plt.ylabel('$\log(L_T / \gamma)$', fontsize = 16)
plt.xlabel('Time (sec)', fontsize = 16)
plt.xticks(size = 12);plt.yticks(size = 12)
plt.tight_layout()

plt.show()
        
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 08:08:01 2020

@author: ind
"""

import numpy as np 
from matplotlib import pyplot as plt 
import time 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import itertools

fig0 = plt.figure(figsize = (5, 4))
ax0 = plt.axes(projection='3d')

fig11, axs11 = plt.subplots(3, figsize = (6, 9))
fig11.suptitle('position trajectory ')

fig, axs = plt.subplots(3, figsize = (6, 9))
fig.suptitle('velocity trajectory ')
 
fig1, axs1 = plt.subplots(3, figsize = (6, 9))
fig1.suptitle('acceleration trajectory ') 

fig3, axs3 = plt.subplots(2, figsize = (6, 6))
fig3.suptitle('thrust and angular velocity')

mass = 2.0
max_thrust = 24.0*0.9



c = [-0.0002016287135431019, 0.0002509716271253959, 0.003815684316517687, -0.0015076478541595772, -0.054572697329180506, 0.081106214679504, 0.5814901228689284, 1.3690627735433587]
d = [-0.00024195445625171952, 0.0003011659525505306, 0.004578821179821335, -0.001809177424991426, -0.06548723679501661, 0.09732745761540484, 0.697788147442714, 1.64287532825203]
e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
T = 11.692302600000007

t = np.linspace(0, T, 101)

x = c[0]*t**7/5040 - c[1]*t**6/720 + c[2]*t**5/120 - c[3]*t**4/24 + c[4]*t**3/6 + c[5]* t**2/2 + c[6]*t + c[7]
y = d[0]*t**7/5040 - d[1]*t**6/720 + d[2]*t**5/120 - d[3]*t**4/24 + d[4]*t**3/6 + d[5]* t**2/2 + d[6]*t + d[7]
z = e[0]*t**7/5040 - e[1]*t**6/720 + e[2]*t**5/120 - e[3]*t**4/24 + e[4]*t**3/6 + e[5]* t**2/2 + e[6]*t + e[7]

vx = c[0]*t**6/720 - c[1]*t**5/120 + c[2]*t**4/24 - c[3]*t**3/6 + c[4]*t**2/2 + c[5]* t + c[6]
vy = d[0]*t**6/720 - d[1]*t**5/120 + d[2]*t**4/24 - d[3]*t**3/6 + d[4]*t**2/2 + d[5]* t + d[6]
vz = e[0]*t**6/720 - e[1]*t**5/120 + e[2]*t**4/24 - e[3]*t**3/6 + e[4]*t**2/2 + e[5]* t + e[6]

ax = c[0]*t**5/120 - c[1]*t**4/24 + c[2]*t**3/6 - c[3]*t**2/2 + c[4]*t + c[5]
ay = d[0]*t**5/120 - d[1]*t**4/24 + d[2]*t**3/6 - d[3]*t**2/2 + d[4]*t + d[5]
az = e[0]*t**5/120 - e[1]*t**4/24 + e[2]*t**3/6 - e[3]*t**2/2 + e[4]*t + e[5]

jx = c[0]*t**4/24 - c[1]*t**3/6 + c[2]*t**2/2 - c[3]*t + c[4]
jy = d[0]*t**4/24 - d[1]*t**3/6 + d[2]*t**2/2 - d[3]*t + d[4]
jz = e[0]*t**4/24 - e[1]*t**3/6 + e[2]*t**2/2 - e[3]*t + e[4]

thrust = mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)

ax0.plot(x, y, z, linewidth = 2)

axs11[0].plot(t, x, linewidth = 2, label='z')
axs11[1].plot(t, y, linewidth = 2, label='z')
axs11[2].plot(t, z, linewidth = 2, label='z')

axs[0].plot(t, vx, linewidth = 2, label='z')
axs[1].plot(t, vy, linewidth = 2, label='z')
axs[2].plot(t, vz, linewidth = 2, label='z')

   # axs1[0].plot(t, a, linewidth = 2, label='z')
axs1[0].plot(t, ax, linewidth = 2, label='z')
axs1[1].plot(t, ay, linewidth = 2, label='z')
axs1[2].plot(t, az, linewidth = 2, label='z')

axs3[0].plot(t, thrust, linewidth = 2, label='z')
axs3[0].hlines(y=max_thrust, xmin=t[0], xmax=t[-1], linewidth=2, color='r')





c = [0.0004716904884297657, 0.0037509767430448926, 0.010895697485252764, 0.004224997386246709, -0.054572697329180506, 0.081106214679504, 0.5814901228689284, 1.3690627735433587]
d = [0.0005660285861157285, 0.004501172091653904, 0.013074836982303273, 0.0050699968634962955, -0.06548723679501661, 0.09732745761540484, 0.697788147442714, 1.64287532825203]
e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
T = 12.861532860000008
t = np.linspace(0, T, 101)

x = c[0]*t**7/5040 - c[1]*t**6/720 + c[2]*t**5/120 - c[3]*t**4/24 + c[4]*t**3/6 + c[5]* t**2/2 + c[6]*t + c[7]
y = d[0]*t**7/5040 - d[1]*t**6/720 + d[2]*t**5/120 - d[3]*t**4/24 + d[4]*t**3/6 + d[5]* t**2/2 + d[6]*t + d[7]
z = e[0]*t**7/5040 - e[1]*t**6/720 + e[2]*t**5/120 - e[3]*t**4/24 + e[4]*t**3/6 + e[5]* t**2/2 + e[6]*t + e[7]

vx = c[0]*t**6/720 - c[1]*t**5/120 + c[2]*t**4/24 - c[3]*t**3/6 + c[4]*t**2/2 + c[5]* t + c[6]
vy = d[0]*t**6/720 - d[1]*t**5/120 + d[2]*t**4/24 - d[3]*t**3/6 + d[4]*t**2/2 + d[5]* t + d[6]
vz = e[0]*t**6/720 - e[1]*t**5/120 + e[2]*t**4/24 - e[3]*t**3/6 + e[4]*t**2/2 + e[5]* t + e[6]

ax = c[0]*t**5/120 - c[1]*t**4/24 + c[2]*t**3/6 - c[3]*t**2/2 + c[4]*t + c[5]
ay = d[0]*t**5/120 - d[1]*t**4/24 + d[2]*t**3/6 - d[3]*t**2/2 + d[4]*t + d[5]
az = e[0]*t**5/120 - e[1]*t**4/24 + e[2]*t**3/6 - e[3]*t**2/2 + e[4]*t + e[5]

jx = c[0]*t**4/24 - c[1]*t**3/6 + c[2]*t**2/2 - c[3]*t + c[4]
jy = d[0]*t**4/24 - d[1]*t**3/6 + d[2]*t**2/2 - d[3]*t + d[4]
jz = e[0]*t**4/24 - e[1]*t**3/6 + e[2]*t**2/2 - e[3]*t + e[4]

thrust = mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)

ax0.plot(x, y, z, linewidth = 2)

axs11[0].plot(t, x, linewidth = 2, label='z')
axs11[1].plot(t, y, linewidth = 2, label='z')
axs11[2].plot(t, z, linewidth = 2, label='z')

axs[0].plot(t, vx, linewidth = 2, label='z')
axs[1].plot(t, vy, linewidth = 2, label='z')
axs[2].plot(t, vz, linewidth = 2, label='z')

   # axs1[0].plot(t, a, linewidth = 2, label='z')
axs1[0].plot(t, ax, linewidth = 2, label='z')
axs1[1].plot(t, ay, linewidth = 2, label='z')
axs1[2].plot(t, az, linewidth = 2, label='z')

axs3[0].plot(t, thrust, linewidth = 2, label='z')
axs3[0].hlines(y=max_thrust, xmin=t[0], xmax=t[-1], linewidth=2, color='r')

T = 14.147686146000009
c = [0.0005400859575245588, 0.004131660067510146, 0.011719218441991752, 0.004937137540244996, -0.054572697329180506, 0.081106214679504, 0.5814901228689284, 1.3690627735433587]
d = [0.0006481031490294664, 0.004957992081012164, 0.014063062130390103, 0.005924565048293862, -0.06548723679501661, 0.09732745761540484, 0.697788147442714, 1.64287532825203]
e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]

t = np.linspace(0, T, 101)

x = c[0]*t**7/5040 - c[1]*t**6/720 + c[2]*t**5/120 - c[3]*t**4/24 + c[4]*t**3/6 + c[5]* t**2/2 + c[6]*t + c[7]
y = d[0]*t**7/5040 - d[1]*t**6/720 + d[2]*t**5/120 - d[3]*t**4/24 + d[4]*t**3/6 + d[5]* t**2/2 + d[6]*t + d[7]
z = e[0]*t**7/5040 - e[1]*t**6/720 + e[2]*t**5/120 - e[3]*t**4/24 + e[4]*t**3/6 + e[5]* t**2/2 + e[6]*t + e[7]

vx = c[0]*t**6/720 - c[1]*t**5/120 + c[2]*t**4/24 - c[3]*t**3/6 + c[4]*t**2/2 + c[5]* t + c[6]
vy = d[0]*t**6/720 - d[1]*t**5/120 + d[2]*t**4/24 - d[3]*t**3/6 + d[4]*t**2/2 + d[5]* t + d[6]
vz = e[0]*t**6/720 - e[1]*t**5/120 + e[2]*t**4/24 - e[3]*t**3/6 + e[4]*t**2/2 + e[5]* t + e[6]

ax = c[0]*t**5/120 - c[1]*t**4/24 + c[2]*t**3/6 - c[3]*t**2/2 + c[4]*t + c[5]
ay = d[0]*t**5/120 - d[1]*t**4/24 + d[2]*t**3/6 - d[3]*t**2/2 + d[4]*t + d[5]
az = e[0]*t**5/120 - e[1]*t**4/24 + e[2]*t**3/6 - e[3]*t**2/2 + e[4]*t + e[5]

jx = c[0]*t**4/24 - c[1]*t**3/6 + c[2]*t**2/2 - c[3]*t + c[4]
jy = d[0]*t**4/24 - d[1]*t**3/6 + d[2]*t**2/2 - d[3]*t + d[4]
jz = e[0]*t**4/24 - e[1]*t**3/6 + e[2]*t**2/2 - e[3]*t + e[4]

thrust = mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)

ax0.plot(x, y, z, linewidth = 2)

axs11[0].plot(t, x, linewidth = 2, label='z')
axs11[1].plot(t, y, linewidth = 2, label='z')
axs11[2].plot(t, z, linewidth = 2, label='z')

axs[0].plot(t, vx, linewidth = 2, label='z')
axs[1].plot(t, vy, linewidth = 2, label='z')
axs[2].plot(t, vz, linewidth = 2, label='z')

   # axs1[0].plot(t, a, linewidth = 2, label='z')
axs1[0].plot(t, ax, linewidth = 2, label='z')
axs1[1].plot(t, ay, linewidth = 2, label='z')
axs1[2].plot(t, az, linewidth = 2, label='z')

axs3[0].plot(t, thrust, linewidth = 2, label='z')
axs3[0].hlines(y=max_thrust, xmin=t[0], xmax=t[-1], linewidth=2, color='r')

T = 126.68266047211563
c = [-1.465781266622062e-07, -1.0768807807468787e-05, -0.0003491810266022313, -0.006161304370765397, -0.054572697329180506, 0.081106214679504, 0.5814901228689284, 1.3690627735433587]
d = [-1.758937519946464e-07, -1.2922569368962523e-05, -0.0004190172319226762, -0.007393565244918465, -0.06548723679501661, 0.09732745761540484, 0.697788147442714, 1.64287532825203]
e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]

t = np.linspace(0, T, 101)

x = c[0]*t**7/5040 - c[1]*t**6/720 + c[2]*t**5/120 - c[3]*t**4/24 + c[4]*t**3/6 + c[5]* t**2/2 + c[6]*t + c[7]
y = d[0]*t**7/5040 - d[1]*t**6/720 + d[2]*t**5/120 - d[3]*t**4/24 + d[4]*t**3/6 + d[5]* t**2/2 + d[6]*t + d[7]
z = e[0]*t**7/5040 - e[1]*t**6/720 + e[2]*t**5/120 - e[3]*t**4/24 + e[4]*t**3/6 + e[5]* t**2/2 + e[6]*t + e[7]

vx = c[0]*t**6/720 - c[1]*t**5/120 + c[2]*t**4/24 - c[3]*t**3/6 + c[4]*t**2/2 + c[5]* t + c[6]
vy = d[0]*t**6/720 - d[1]*t**5/120 + d[2]*t**4/24 - d[3]*t**3/6 + d[4]*t**2/2 + d[5]* t + d[6]
vz = e[0]*t**6/720 - e[1]*t**5/120 + e[2]*t**4/24 - e[3]*t**3/6 + e[4]*t**2/2 + e[5]* t + e[6]

ax = c[0]*t**5/120 - c[1]*t**4/24 + c[2]*t**3/6 - c[3]*t**2/2 + c[4]*t + c[5]
ay = d[0]*t**5/120 - d[1]*t**4/24 + d[2]*t**3/6 - d[3]*t**2/2 + d[4]*t + d[5]
az = e[0]*t**5/120 - e[1]*t**4/24 + e[2]*t**3/6 - e[3]*t**2/2 + e[4]*t + e[5]

jx = c[0]*t**4/24 - c[1]*t**3/6 + c[2]*t**2/2 - c[3]*t + c[4]
jy = d[0]*t**4/24 - d[1]*t**3/6 + d[2]*t**2/2 - d[3]*t + d[4]
jz = e[0]*t**4/24 - e[1]*t**3/6 + e[2]*t**2/2 - e[3]*t + e[4]

thrust = mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)

ax0.plot(x, y, z, linewidth = 2)

axs11[0].plot(t, x, linewidth = 2, label='z')
axs11[1].plot(t, y, linewidth = 2, label='z')
axs11[2].plot(t, z, linewidth = 2, label='z')

axs[0].plot(t, vx, linewidth = 2, label='z')
axs[1].plot(t, vy, linewidth = 2, label='z')
axs[2].plot(t, vz, linewidth = 2, label='z')

   # axs1[0].plot(t, a, linewidth = 2, label='z')
axs1[0].plot(t, ax, linewidth = 2, label='z')
axs1[1].plot(t, ay, linewidth = 2, label='z')
axs1[2].plot(t, az, linewidth = 2, label='z')

axs3[0].plot(t, thrust, linewidth = 2, label='z')
axs3[0].hlines(y=max_thrust, xmin=t[0], xmax=t[-1], linewidth=2, color='r')        
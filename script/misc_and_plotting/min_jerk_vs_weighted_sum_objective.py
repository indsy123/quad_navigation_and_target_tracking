#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
generate polynonial trajectory 
@author: indrajeet
"""

import numpy as np 
from matplotlib import pyplot as plt 
import time 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


class polynomial_trajetory(object):
    def __init__(self): 
        """initializes variables"""
        self.ip = np.array([0.0, 0.0, 3.0])
        self.iv = np.array([3.0, 0.0, 0.0])
        self.ia = np.array([0.0, 0.0, 0.0])
        self.finalgoal = np.array([5.0, 5.0, 3.0])
        self.colors = ['r', 'b', 'g', 'm', 'c' ]
        self.psi0 = np.array([0.0, 0.0])

    def yaw_coefficients(self): 
        print''
        

    def combined_coefficients(self, start_conditions , end_conditions, T, k1, k2): 

        x0 = start_conditions[0]
        v0 = start_conditions[1]
        a0 = start_conditions[2]
        xT = end_conditions[0]
        vT = end_conditions[1]
        aT = end_conditions[2]
        delx = xT - (x0 + v0 * T + 0.5 * a0 * T**2)
        delv  = vT - (v0 + a0 * T)
        dela = aT - a0 
        dels = np.array([delx, delv, dela])
        
        A = (1/T**5) * np.array([[720.0, -360*T, 60*T**2], [360*T, -168*T**2, 24*T**3], [60*T**2, -24*T**3, 3*T**4]])
        #A = (1/T**5) * np.array([[-720.0, 360*T, -60*T**2], [-360*T, 168*T**2, -24*T**3], [-60*T**2, 24*T**3, -3*T**4]])
        
        first_three = (k2-k1) * np.dot(A, dels)
        c1 = first_three[0]; c2 = first_three[1]; c3 = first_three[2]
        c4 = (k2-k1) * a0; c5 = (k2-k1) * v0; c6 = (k2-k1) * x0
        
        return [c1, c2, c3, c4, c5, c6]
        
    def min_jerk_coefficients(self, start_conditions , end_conditions, T): 
        x0 = start_conditions[0]
        v0 = start_conditions[1]
        a0 = start_conditions[2]
        xT = end_conditions[0]
        vT = end_conditions[1]
        aT = end_conditions[2]
        delx = xT - (x0 + v0 * T + 0.5 * a0 * T**2)
        delv  = vT - (v0 + a0 * T)
        dela = aT - a0 
        dels = np.array([delx, delv, dela])
        
        #A = (1/T**5) * np.array([[720.0, -360*T, 60*T**2], [360*T, -168*T**2, 24*T**3], [-60*T**2, 24*T**3, -3*T**4]])
        A = (1/T**5) * np.array([[-720.0, 360*T, -60*T**2], [-360*T, 168*T**2, -24*T**3], [-60*T**2, 24*T**3, -3*T**4]])
        
        first_three = np.dot(A, dels)
        c1 = first_three[0]; c2 = first_three[1]; c3 = first_three[2]
        c4 = a0; c5 = v0; c6 = x0
        
        return [c1, c2, c3, c4, c5, c6]
       
    def plot(self, c, d, e, _c, _d, _e, T, s, plane_no, k1, k2): 
        
        t = np.linspace(0, T, 101)
        x = -c[0]*t**5/120 + c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
        y = -d[0]*t**5/120 + d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + d[5]
        z = -e[0]*t**5/120 + e[1]*t**4/24 - e[2]*t**3/6 + e[3]*t**2/2 + e[4]*t + e[5]
        
        vx = -c[0]*t**4/24 + c[1]*t**3/6 - c[2]*t**2/2 + c[3]*t + c[4]
        vy = -d[0]*t**4/24 + d[1]*t**3/6 - d[2]*t**2/2 + d[3]*t + d[4]
        vz = -e[0]*t**4/24 + e[1]*t**3/6 - e[2]*t**2/2 + e[3]*t + e[4]
        
        ax = -c[0]*t**3/6 + c[1]*t**2/2 - c[2]*t + c[3]
        ay = -d[0]*t**3/6 + d[1]*t**2/2 - d[2]*t + d[3]
        az = -e[0]*t**3/6 + e[1]*t**2/2 - e[2]*t + e[3]
        
        
        
        mycolor = (0 + s/10.0, 0, 0)
      


        _x = 1/(k2-k1) * (_c[0]*t**5/120 - _c[1]*t**4/24 + _c[2]*t**3/6 + _c[3]*t**2/2 + _c[4]*t + _c[5])
        _y = 1/(k2-k1) * (_d[0]*t**5/120 - _d[1]*t**4/24 + _d[2]*t**3/6 + _d[3]*t**2/2 + _d[4]*t + _d[5])
        _z = 1/(k2-k1) * (_e[0]*t**5/120 - _e[1]*t**4/24 + _e[2]*t**3/6 + _e[3]*t**2/2 + _e[4]*t + _e[5])
        
        _vx = 1/(k2-k1) * (_c[0]*t**4/24 - _c[1]*t**3/6 + _c[2]*t**2/2 + _c[3]*t + _c[4])
        _vy = 1/(k2-k1) * (_d[0]*t**4/24 - _d[1]*t**3/6 + _d[2]*t**2/2 + _d[3]*t + _d[4])
        _vz = 1/(k2-k1) * (_e[0]*t**4/24 - _e[1]*t**3/6 + _e[2]*t**2/2 + _e[3]*t + _e[4])
        
        _ax = 1/(k2-k1) * (_c[0]*t**3/6 - _c[1]*t**2/2 + _c[2]*t + _c[3])
        _ay = 1/(k2-k1) * (_d[0]*t**3/6 - _d[1]*t**2/2 + _d[2]*t + _d[3])
        _az = 1/(k2-k1) * (_e[0]*t**3/6 - _e[1]*t**2/2 + _e[2]*t + _e[3])
 

        fig = plt.figure(figsize = (10, 8))
        plt.plot(x, y, color = mycolor, linewidth = 3-s/10, label='min_jerk')
        plt.plot(_x, _y, color = mycolor, linewidth = 3-s/10, linestyle = ':', label='combined')
        fig = plt.figure(figsize = (10, 8))
        plt.plot(t, vx, color = 'g', linewidth = 3-s/10, label='vx_min_jerk')
        plt.plot(t, vy, color = 'b', linewidth = 3-s/10, label='vy_min_jerk')
        plt.plot(t, _vx, color = 'g', linewidth = 3-s/10, linestyle = ':', label='vx_combined')
        plt.plot(t, _vy, color = 'b', linewidth = 3-s/10, linestyle = ':', label='vy_combined')
        
        plt.xlabel('x(m)', fontsize = 16)
        plt.ylabel('y(m)', fontsize = 16)
        plt.xticks(size = 16);plt.yticks(size = 16)
        
    def generate_trajectory(self): 
        t = time.time()
        max_speed = 10.0
        nplanes = 2
        range_min = 3.0
        range_max = 6.0
        theta_y = 60
        theta_z = 60
        _del = 150
        
        d = (range_max - range_min)/(nplanes-1)
        k1 = 0.3; k2 = 0.7
        
        
        #ax = fig.add_subplot(111, projection='3d')
        # triad at the ground
        #ax.plot((0,0.5), (0,0), (0, 0), c = 'g', linewidth = 2)
        #ax.plot((0,0), (0,0.2), (0, 0), c = 'r', linewidth = 2)
        #ax.plot((0,0), (0,0), (0, 0.6), c = 'b', linewidth = 2)
        
        #ax.plot((0,0), (0,0), (0, 1), c = 'k', linewidth = 1, linestyle = ':')      
        # triad at the quad location
        #ax.plot((0,0.5), (0,0), (3, 3), c = 'g', linewidth = 2) 
        #ax.plot((0,0), (0,0.2), (3, 3), c = 'r', linewidth = 2)
        #ax.plot((0,0), (0,0), (3, 3.6), c = 'b', linewidth = 2)
        for i in range(1):             
            r  = range_min + d * i
            av_speed = (r/range_max) * max_speed
            print r, av_speed
            m = i%2 
            xx, yy, zz = [], [], []
            j = 40; k = 40
            x = self.ip[0] + r*np.cos(j*np.pi/180)*np.cos(k*np.pi/180)                    
            y = self.ip[1] + r*np.sin(j*np.pi/180)*np.cos(k*np.pi/180)                    
            z = self.ip[2] + r*np.sin(k*np.pi/180)
            #xx.append(x); yy.append(y); zz.append(z)

            xic = np.array([self.ip[0], self.iv[0], self.ia[0]])
            yic = np.array([self.ip[1], self.iv[1], self.ia[1]])
            zic = np.array([self.ip[2], self.iv[2], self.ia[2]])
            xfc = np.array([x, 0, 0])
            yfc = np.array([y, 0, 0])
            zfc = np.array([z, 0, 0])
            #T = np.linalg.norm(end-self.start)/av_speed
            T = np.sqrt((x-self.ip[0])**2 + (y-self.ip[1])**2  + (z-self.ip[2])**2)/av_speed
            
            x_coef = self.min_jerk_coefficients(xic, xfc, T)
            y_coef =  self.min_jerk_coefficients(yic, yfc, T)
            z_coef =  self.min_jerk_coefficients(zic, zfc, T)
            
            _x_coef = self.combined_coefficients(xic, xfc, T, k1, k2)
            _y_coef =  self.combined_coefficients(yic, yfc, T, k1, k2)
            _z_coef =  self.combined_coefficients(zic, zfc, T, k1, k2)
            self.plot(x_coef, y_coef, z_coef, _x_coef, _y_coef, _z_coef,T, av_speed, i, k1, k2)
                    
            theta = np.linspace(-theta_y/2, theta_y/2, 40)
            phi = np.linspace(-theta_z/2, theta_z/2, 40)
            #print theta
            
            for p in theta:
                for q in phi:
                    xx.append(self.ip[0] + r*np.cos(p*np.pi/180)*np.cos(q*np.pi/180))                   
                    yy.append(self.ip[1] + r*np.sin(p*np.pi/180)*np.cos(q*np.pi/180))                   
                    zz.append(self.ip[2] + r*np.sin(q*np.pi/180))

            #ax.scatter(xx, yy, zz, c=self.colors[i], marker='.', alpha = 0.25)
        
        #ax.tick_params(direction='out', length=6, width=2, colors='k', labelsize=18)
        #ax.set_xlabel("$x(m)$", labelpad = 13, size = 30)
        #ax.set_ylabel("$y(m)$", labelpad = 13, size = 30)
        #ax.arrow(0, 0, 0.0, 0.02, fc='g', ec='k', lw = 0.01, head_width=0.004, head_length=0.002, overhang = 0, clip_on = False)
        #ax.set_zlabel("$z(m)$", labelpad = 13, size = 30)
        
        plt.tight_layout()
        plt.show()
        print 'total time:', time.time()-t
    


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.generate_trajectory()

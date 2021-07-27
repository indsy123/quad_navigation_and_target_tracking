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
import itertools


class polynomial_trajetory(object):
    def __init__(self): 
        """initializes variables"""
        self.path = '/home/ind/data_collection/TRO_dataset/'
        open(self.path + 'min_snap_parameter_comparison.txt', 'w').close()
        open(self.path + 'min_snap_parameter_comparison2.txt', 'w').close()
        open(self.path + 'coefficients.txt', 'w').close()
        self.ip = np.array([0.0, 0.0, 2.0])
        self.iv = np.array([0.0, 0.0, 0.0])
        self.ia = np.array([0.0, 0.0, 0.0])
        self.ij = np.array([0.0, 0.0, 0.0])
        
        self.finalgoal = np.array([15.0, 18.0, 2.0])
        self.colors = ['r', 'b', 'g', 'm', 'c' ]
        self.psi0 = np.array([0.0, 0.0])
        self.mass = 2.0
        self.max_thrust = 34.0*0.8
        self.min_thrust = 14.7
        self.max_speed = 0.5

        self.max_acceleration = self.max_thrust/self.mass #- 9.8
        self.nplanes = 5
        self.range_min = 3.0
        self.range_max = 6.0
        self.theta_y = 60
        self.theta_z = 60
        self._del = 10
        self.time_horizon = 0.5

        self.max_angular_velocity = 220.0/(2*np.pi) # (using 200 deg/sec)
        #self.regerate_trajectory = True
        #self.trajectory_is_feasible = True
        #self.limit_exceeded = False
        
        
    def get_T(self, xt, yt, zt, x, y, z, vel):
        dist_to_goal = np.sqrt((xt-x)**2 + (yt-y)**2 + (zt-z)**2)
        T1 = dist_to_goal/(vel)
        T2 = np.sqrt(2*dist_to_goal/self.max_acceleration)
        T = max (T1, T2)
        return T

    def phi_coefficients(self, phi_ic, phi_fc, T): 
        phi0 = phi_ic[0]
        phir0 = phi_ic[1]
        phiT = phi_fc[0]
        phirT = phi_fc[1]
        p0 = phi0
        p1 = phir0
        p2 = -0.5 * (-(6.0/(T**2)) * (phiT-phi0) + 2.0/T * (2*phir0+phirT))
        p3 = 1.0/6 * ((-12.0/(T**3)) * (phiT-phi0) + (6.0/(T**2)) * (phir0+phirT))
        return [p0, p1, p2, p3]
        
    def min_snap_coefficients(self, start_conditions , end_conditions, T): 
        x0 = start_conditions[0]
        v0 = start_conditions[1]
        a0 = start_conditions[2]
        j0 = start_conditions[3]
        xT = end_conditions[0]
        vT = end_conditions[1]
        aT = end_conditions[2]
        jT = end_conditions[3]
        delx = xT - (x0 + v0 * T + 0.5 * a0 * T**2 + j0*T**3/6)
        delv  = vT - (v0 + a0 * T + j0*T**2/2)
        dela = aT - (a0+j0*T)
        delj = jT-j0
        dels = np.array([delx, delv, dela, delj])
        #print 'value of T:', T
        

        A = (1/T**7) * np.array([[-100800, 50400*T, -10080*T**2, 840*T**3], [-50400*T, 24480*T**2, -4680*T**3, 360*T**4], 
                                 [-10080*T**2, 4680*T**3, -840*T**4, 60*T**5], [-840*T**3, 360*T**4, -60*T**5, 4*T**6]])
        
        first_four = np.dot(A, dels)

        c1 = first_four[0]; c2 = first_four[1]; c3 = first_four[2]; c4 = first_four[3]
        c5 = j0; c6 = a0; c7 = v0; c8 = x0
        
        return [c1, c2, c3, c4, c5, c6, c7, c8]
       
    def trajectory(self, xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc): 
        k = 50
        c = self.min_snap_coefficients(xic, xfc, T)
        d = self.min_snap_coefficients(yic, yfc, T)
        e = self.min_snap_coefficients(zic, zfc, T)
        _phi = self.phi_coefficients(phi_ic, phi_fc, T)
        f = open(self.path + 'coefficients.txt', 'a')
        f.write("%s, %s\n" % ("local goal:", self.localgoal))
        f.write("%s, %s, %s, %s\n" % (T, c, d, e))   
        
        t = np.linspace(0, T, 31)
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
        
        phi = _phi[0] + _phi[1]*t + _phi[2] *t**2 + _phi[3] *t**3
        phir = _phi[1] + 2 *_phi[2] *t + 3 *_phi[3] *t**2
        v = np.sqrt(vx**2 + vy**2 + vz**2)
        a = np.sqrt(ax**2 + ay**2 + az**2)   
        thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
        #f = open(self.path + 'min_snap_parameter_comparison2.txt', 'a')
        #f.write("%s, %s, %s, %s, %s, %s\n" % (T, v[0], a[0], max(v), max(a), max(thrust)))        
        #axs[0].plot(t, vx, linewidth = 2, label='z')
        #axs[1].plot(t, vy, linewidth = 2, label='z')
        #axs[2].plot(t, vz, linewidth = 2, label='z')
        
        return c, d, e, _phi, t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, phi, phir
        
        
        

        
    def generate_trajectory(self, xic, yic, zic, xfc, yfc, zfc, phi_ic, phi_fc, av_speed): 
        tt = time.time()

        
        T = self.get_T(self.localgoal[0], self.localgoal[1], self.localgoal[2], self.ip[0], self.ip[1], self.ip[2], av_speed)

        c = [0]*6; d = [0]*6; e = [0]*6; _phi = [0]*4
        self.regerate_trajectory = True
        self.trajectory_is_feasible = True
        self.limit_exceeded = False
        f = open(self.path + 'min_snap_parameter_comparison.txt', 'a')
        f.write("%s, %s\n" % ("local goal:", self.localgoal))
        
        while self.regerate_trajectory: 
            
            c, d, e, _phi, t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc)
            
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + az**2)
            j = np.sqrt(jx**2 + jy**2 + jz**2)
            thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
            ang_vel = j/thrust
            v_max = max(v); a_max = max(a); j_max = max(j)
            ang_vel_max = max (ang_vel)
        
            #EE.append(E); EE1.append(E1); EE2.append(E2); TT.append(T) 
            f = open(self.path + 'min_snap_parameter_comparison.txt', 'a')
            f.write("%s, %s, %s, %s, %s, %s\n" % (T, v[0], a[0], v_max, a_max, max(thrust)))
            if  max(thrust) > self.max_thrust \
                or max(ang_vel) > self.max_angular_velocity or v_max > self.max_speed: 
                #print 'here:1', T, v_max
                T = T * 1.1#(a_max/self.max_acceleration)
                self.trajectory_is_feasible = False
                self.regerate_trajectory = True
                self.limit_exceeded = True
            else:
                #print 'here:2'
                #self.regerate_trajectory = False
                if not self.limit_exceeded:
                    T = T*0.9
                    self.regerate_trajectory = True
                else:
                    self.regerate_trajectory = False

        f = open(self.path + 'min_snap_parameter_comparison.txt', 'a')
        f.write("%s\n" % ('returned values..'))
        f.write("%s, %s, %s, %s, %s, %s\n" % (T, v[0], a[0], v_max, a_max, max(thrust)))
        
        f.write("%s\n" % ('trajectory iteration ended..'))
        return c, d, e, _phi, T
        #print 'total time:', time.time()-tt
    
    def RHP(self): 
        ttt = time.time()
      
        xx, yy, zz, TT = [], [], [], []   
        EE, EE1, EE2 = [], [], []
        r  = 6.0
        av_speed = self.max_speed * (r/self.range_max)
        j = 45; k = 45
        
        xf = self.finalgoal[0]#self.ip[0] + r*np.cos(j*np.pi/180)*np.cos(k*np.pi/180)                    
        yf = self.finalgoal[1]#self.ip[1] + r*np.sin(j*np.pi/180)*np.cos(k*np.pi/180)                    
        zf = self.finalgoal[2]#self.ip[2] + r*np.sin(k*np.pi/180)
        
        fig0 = plt.figure(figsize = (5, 4))
        ax0 = plt.axes(projection='3d')

        fig11, axs11 = plt.subplots(3, figsize = (6, 9))
        fig11.suptitle('position trajectory iterations')
        
        fig, axs = plt.subplots(3, figsize = (6, 9))
        fig.suptitle('velocity trajectory iterations')
 
        fig1, axs1 = plt.subplots(3, figsize = (6, 9))
        fig1.suptitle('acceleration trajectory iterations') 

        #fig2, axs2 = plt.subplots(4, figsize = (6, 12))
        #fig2.suptitle('jerk trajectory iterations')
        
        fig3, axs3 = plt.subplots(2, figsize = (6, 6))
        fig3.suptitle('thrust and angular velocity')
        
        fig4, axs4 = plt.subplots(2, figsize = (5, 4))
        fig4.suptitle('phi')
        
        if np.linalg.norm(self.finalgoal-self.ip) > self.range_max:
            self.localgoal = self.ip + (self.finalgoal-self.ip)/np.linalg.norm(self.finalgoal-self.ip) * self.range_max
            av_speed = 1
        else:
            self.localgoal = self.finalgoal
            
        xic = np.array([self.ip[0], self.iv[0], self.ia[0], self.ij[0]])
        yic = np.array([self.ip[1], self.iv[1], self.ia[1], self.ij[1]])
        zic = np.array([self.ip[2], self.iv[2], self.ia[2], self.ij[2]])
        
        xfc = np.array([self.localgoal[0], 0, 0, 0])
        yfc = np.array([self.localgoal[1], 0, 0, 0])
        zfc = np.array([self.localgoal[2], 0, 0, 0])
        
        phi_ic = np.array([0, 0])
        final_phi = np.arctan((self.localgoal[1]- self.ip[1])/(self.localgoal[0]-self.ip[0]))
        phi_fc = np.array([final_phi, 0])
            
        for n in range(500): 
            
            if np.linalg.norm(self.finalgoal-self.ip) > self.range_max: 
                c, d, e, _phi, T = self.generate_trajectory(xic, yic, zic, xfc, yfc, zfc, phi_ic, phi_fc, av_speed)
                t = np.linspace(0, self.time_horizon, 31)
                tt = np.linspace(n*self.time_horizon, (n+1)*self.time_horizon, 31)
                #print '1', t
            else: 
                c, d, e, _phi, T = self.generate_trajectory(xic, yic, zic, xfc, yfc, zfc, phi_ic, phi_fc, 1)
                t = np.linspace(0, T, 31)
                tt = np.linspace(n*self.time_horizon, n*self.time_horizon + T , 31)

                
            

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
            
            thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
            
            phi = _phi[0] + _phi[1]*t + _phi[2] *t**2 + _phi[3] *t**3
            phir = _phi[1] + 2 *_phi[2] *t + 3 *_phi[3] *t**2
            
            ax0.plot(x, y, z, linewidth = 2)
            
        
            #tt = np.linspace(n*self.time_horizon, (n+1)*self.time_horizon, 31)
            axs11[0].plot(tt, x, linewidth = 2, label='z')
            axs11[1].plot(tt, y, linewidth = 2, label='z')
            axs11[2].plot(tt, z, linewidth = 2, label='z')
            
            axs[0].plot(tt, vx, linewidth = 2, label='z')
            axs[1].plot(tt, vy, linewidth = 2, label='z')
            axs[2].plot(tt, vz, linewidth = 2, label='z')

           # axs1[0].plot(t, a, linewidth = 2, label='z')
            axs1[0].plot(tt, ax, linewidth = 2, label='z')
            axs1[1].plot(tt, ay, linewidth = 2, label='z')
            axs1[2].plot(tt, az, linewidth = 2, label='z')

            axs3[0].plot(tt, thrust, linewidth = 2, label='z')
            axs3[0].hlines(y=self.max_thrust, xmin=tt[0], xmax=tt[-1], linewidth=2, color='r')
            #axs3[1].plot(t, ang_vel, linewidth = 2, label='z')
            
            axs4[0].plot(tt, phi, linewidth = 2, label='phi')
            axs4[1].plot(tt, phir, linewidth = 2, label='phir')     
            

            self.ip = np.array([x[-1], y[-1], z[-1]])
            self.iv = np.array([vx[-1], vy[-1], vz[-1]])
            self.ia = np.array([ax[-1], ay[-1], az[-1]])
            self.ij = np.array([jx[-1], jy[-1], jz[-1]])
            
            xic = np.array([self.ip[0], self.iv[0], self.ia[0], self.ij[0]])
            yic = np.array([self.ip[1], self.iv[1], self.ia[1], self.ij[1]])
            zic = np.array([self.ip[2], self.iv[2], self.ia[2], self.ij[2]])
            
            if np.linalg.norm(self.finalgoal-self.ip) > self.range_max:
                self.localgoal = self.ip + (self.finalgoal-self.ip)/np.linalg.norm(self.finalgoal-self.ip) * self.range_max
            else:
                self.localgoal = self.finalgoal
            
            xfc = np.array([self.localgoal[0], 0, 0, 0])
            yfc = np.array([self.localgoal[1], 0, 0, 0])
            zfc = np.array([self.localgoal[2], 0, 0, 0])
            phi_ic = np.array([phi[-1], phir[-1]])
            final_phi = np.arctan((self.localgoal[1]- self.ip[1])/(self.localgoal[0]-self.ip[0]))
            phi_fc = np.array([final_phi, 0])
            print 'total time taken', time.time()-ttt

            
            if t[-1] == T:
                #print 'i m here'
                break
                
        


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.RHP()

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
        self.ip = np.array([0.0, 0.0, 2.0])
        self.iv = np.array([0.0, 0.0, 0.0])
        self.ia = np.array([0.0, 0.0, 0.0])
        
        self.finalgoal = np.array([15.0, 18.0, 2.0])
        self.colors = ['r', 'b', 'g', 'm', 'c' ]
        self.psi0 = np.array([0.0, 0.0])
        self.mass = 2.0
        self.max_thrust = 34.0*0.9
        self.min_thrust = 14.7
        self.max_speed = 6
        self.max_acceleration = 10#self.max_thrust/self.mass - 9.8
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
        T1 = dist_to_goal/vel
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
       
    def trajectory(self, xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc): 
        k = 50
        c = self.min_jerk_coefficients(xic, xfc, T)
        d = self.min_jerk_coefficients(yic, yfc, T)
        e = self.min_jerk_coefficients(zic, zfc, T)
        _phi = self.phi_coefficients(phi_ic, phi_fc, T)
     
        
        t = np.linspace(0, T, 31)
        x = -c[0]*t**5/120 + c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
        y = -d[0]*t**5/120 + d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + d[5]
        z = -e[0]*t**5/120 + e[1]*t**4/24 - e[2]*t**3/6 + e[3]*t**2/2 + e[4]*t + e[5]
        
        vx = -c[0]*t**4/24 + c[1]*t**3/6 - c[2]*t**2/2 + c[3]*t + c[4]
        vy = -d[0]*t**4/24 + d[1]*t**3/6 - d[2]*t**2/2 + d[3]*t + d[4]
        vz = -e[0]*t**4/24 + e[1]*t**3/6 - e[2]*t**2/2 + e[3]*t + e[4]
        
        ax = -c[0]*t**3/6 + c[1]*t**2/2 - c[2]*t + c[3]
        ay = -d[0]*t**3/6 + d[1]*t**2/2 - d[2]*t + d[3]
        az = -e[0]*t**3/6 + e[1]*t**2/2 - e[2]*t + e[3]

        jx = -c[0]*t**2/2 + c[1]*t - c[2]
        jy = -d[0]*t**2/2 + d[1]*t - d[2]
        jz = -e[0]*t**2/2 + e[1]*t - e[2]
        
        phi = _phi[0] + _phi[1]*t + _phi[2] *t**2 + _phi[3] *t**3
        phir = _phi[1] + 2 *_phi[2] *t + 3 *_phi[3] *t**2
        
        
        
        E1 = k * T 
        E2 = 0.5 * ((c[0]**2 * T**5/20 - c[0]*c[1] * T**4/4 + (c[1]**2 + c[0]*c[2]) * T**3/3 - c[1]*c[2] *T**2 + c[2]**2*T) +  \
                   (d[0]**2 * T**5/20 - d[0]*d[1] * T**4/4 + (d[1]**2 + d[0]*d[2]) * T**3/3 - d[1]*d[2] *T**2 + d[2]**2*T) + \
                   (e[0]**2 * T**5/20 - e[0]*e[1] * T**4/4 + (e[1]**2 + e[0]*e[2]) * T**3/3 - e[1]*e[2] *T**2 + e[2]**2*T))
        E = E1 + E2

        a1 = 60.0 * (xfc[0] - xic[0]); a2 = 60.0 * (yfc[0] - yic[0]); a3 = 60.0 * (zfc[0] - zic[0])
        b1 = -24.0 * xic[1]; b2 = -24.0 * yic[1]; b3 = -24.0 * zic[1]
        c1 = -3.0 * xic[2]; c2 = -3.0 * yic[2]; c3 = -3.0 * zic[2]
        p0 = -2*k; p1 = 0; p2 = c1**2 + c2**2 + c3**2; p3 = 2 * (b1*c1 + b2*c2 + b3*c3)
        p4 = b1**2 + b2**2 + b3**2 + 2 * (a1*c1 + a1*c2 + a3*c3)
        p5 = 2 * (a1*b1 + a2*b2 + a3*b3); p6 = a1**2 + a2**2 + a3**2
        #ttt = time.time()
        coeff = [p0, p1, p2, p3, p4, p5, p6]
        roots = np.roots(coeff)  
        #print 'time in finding roots', time.time()-ttt
        return c, d, e, _phi, t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, roots, phi, phir
        
        
        

        
    def generate_trajectory(self, xic, yic, zic, xfc, yfc, zfc, phi_ic, phi_fc, av_speed): 
        tt = time.time()
        EE, EE1, EE2, TT = [], [], [], []

        
        T = self.get_T(self.finalgoal[0], self.finalgoal[1], self.finalgoal[2], self.ip[0], self.ip[1], self.ip[2], av_speed)

        T0 = T
        c = [0]*6; d = [0]*6; e = [0]*6; _phi = [0]*4
        self.regerate_trajectory = True
        self.trajectory_is_feasible = True
        self.limit_exceeded = False
        while self.regerate_trajectory: 

            c, d, e, _phi, t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, roots, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc)
            
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + az**2)
            j = np.sqrt(jx**2 + jy**2 + jz**2)
            thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
            ang_vel = j/thrust
            v_max = max(v); a_max = max(a); j_max = max(j)
            ang_vel_max = max (ang_vel)

            
            EE.append(E); EE1.append(E1); EE2.append(E2); TT.append(T) 
            
            if  max(thrust) > self.max_thrust or max(ang_vel) > self.max_angular_velocity:
                #print 'here:1'
                T = T * 1.1#(a_max/self.max_acceleration)
                self.trajectory_is_feasible = False
                self.regerate_trajectory = True
                self.limit_exceeded = True
            else:
                #print 'here:2'
                if not self.limit_exceeded:
                    T = T*0.9
                    self.regerate_trajectory = True
                else:
                    self.regerate_trajectory = False

        return c, d, e, _phi, T
        #print 'total time:', time.time()-tt
    
    def RHP(self): 
        tt = time.time()
      
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
        
        #fig4, axs4 = plt.subplots(2, figsize = (5, 4))
        #fig4.suptitle('phi')
        if np.linalg.norm(self.finalgoal-self.ip) > self.range_max:
            self.localgoal = self.ip + (self.finalgoal-self.ip)/np.linalg.norm(self.finalgoal-self.ip) * self.range_max
        else:
            self.localgoal = self.finalgoal
            
        xic = np.array([self.ip[0], self.iv[0], self.ia[0]])
        yic = np.array([self.ip[1], self.iv[1], self.ia[1]])
        zic = np.array([self.ip[2], self.iv[2], self.ia[2]])
        
        xfc = np.array([self.localgoal[0], 0, 0])
        yfc = np.array([self.localgoal[1], 0, 0])
        zfc = np.array([self.localgoal[2], 0, 0])
        
        phi_ic = np.array([0, 0])
        final_phi = np.arctan((self.localgoal[1]- self.ip[1])/(self.localgoal[0]-self.ip[0]))
        phi_fc = np.array([final_phi, 0])
            
        for n in range(100): 
            #if np.linalg.norm(self.finalgoal-self.ip) > self.range_max: 
            print xic, yic, zic
        
            c, d, e, _phi, T = self.generate_trajectory(xic, yic, zic, xfc, yfc, zfc, phi_ic, phi_fc, av_speed)
            
            if np.linalg.norm(self.finalgoal-self.ip) > self.range_max: 
                t = np.linspace(0, self.time_horizon, 31)
                tt = np.linspace(n*self.time_horizon, (n+1)*self.time_horizon, 31)
                #print '1', t
            else: 
                t = np.linspace(0, T, 31)
                tt = np.linspace(n*self.time_horizon, n*self.time_horizon + T , 31)
                
            x = -c[0]*t**5/120 + c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
            y = -d[0]*t**5/120 + d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + d[5]
            z = -e[0]*t**5/120 + e[1]*t**4/24 - e[2]*t**3/6 + e[3]*t**2/2 + e[4]*t + e[5]
            
            vx = -c[0]*t**4/24 + c[1]*t**3/6 - c[2]*t**2/2 + c[3]*t + c[4]
            vy = -d[0]*t**4/24 + d[1]*t**3/6 - d[2]*t**2/2 + d[3]*t + d[4]
            vz = -e[0]*t**4/24 + e[1]*t**3/6 - e[2]*t**2/2 + e[3]*t + e[4]
            
            ax = -c[0]*t**3/6 + c[1]*t**2/2 - c[2]*t + c[3]
            ay = -d[0]*t**3/6 + d[1]*t**2/2 - d[2]*t + d[3]
            az = -e[0]*t**3/6 + e[1]*t**2/2 - e[2]*t + e[3] 
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
            
            #axs4[0].plot(t, phi, linewidth = 2, label='phi')
            #axs4[1].plot(t, phir, linewidth = 2, label='phir')            
            
            self.ip = np.array([x[-1], y[-1], z[-1]])
            self.iv = np.array([vx[-1], vy[-1], vz[-1]])
            self.ia = np.array([ax[-1], ay[-1], az[-1]])
            xic = np.array([self.ip[0], self.iv[0], self.ia[0]])
            yic = np.array([self.ip[1], self.iv[1], self.ia[1]])
            zic = np.array([self.ip[2], self.iv[2], self.ia[2]])
            
            if np.linalg.norm(self.finalgoal-self.ip) > self.range_max:
                self.localgoal = self.ip + (self.finalgoal-self.ip)/np.linalg.norm(self.finalgoal-self.ip) * self.range_max
            else:
                self.localgoal = self.finalgoal
            
            xfc = np.array([self.localgoal[0], 0, 0])
            yfc = np.array([self.localgoal[1], 0, 0])
            zfc = np.array([self.localgoal[2], 0, 0])
            phi_ic = np.array([phi[-1], phir[-1]])
            final_phi = np.arctan((self.localgoal[1]- self.ip[1])/(self.localgoal[0]-self.ip[0]))
            phi_fc = np.array([final_phi, 0])
            print '1', self.localgoal, t[-1]

            
            if t[-1] == T:
                print 'i m here'
                break
                
        


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.RHP()

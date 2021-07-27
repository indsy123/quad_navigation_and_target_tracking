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
        self.ip = np.array([5.0, 5.0, 2.0])
        self.iv = np.array([3.0, 4.5, 2.0])
        self.ia = np.array([2.0, 3.0, 2.0])
        
        self.finalgoal = np.array([5.0, 0.0, 2.0])
        self.colors = ['r', 'b', 'g', 'm', 'c' ]
        self.psi0 = np.array([0.0, 0.0])
        self.mass = 2.0
        self.max_thrust = 34.0*0.9
        self.min_thrust = 14.7
        self.max_speed = 10
        self.max_acceleration = self.max_thrust/self.mass - 9.8
        self.nplanes = 5
        self.range_min = 3.0
        self.range_max = 6.0
        self.theta_y = 60
        self.theta_z = 60
        self._del = 10

        self.max_angular_velocity = 220.0/(2*np.pi) # (using 200 deg/sec)
        self.regerate_trajectory = True
        self.trajectory_is_feasible = True
        self.limit_exceeded = False
        
        
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
        return t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, roots, phi, phir
        
        
        

        
    def generate_trajectory(self): 
        tt = time.time()
      
        xx, yy, zz, TT = [], [], [], []   
        EE, EE1, EE2 = [], [], []
        r  = 6.0
        av_speed = self.max_speed * (r/self.range_max)
        j = 45; k = 45
        
        x = self.ip[0] + r*np.cos(j*np.pi/180)*np.cos(k*np.pi/180)                    
        y = self.ip[1] + r*np.sin(j*np.pi/180)*np.cos(k*np.pi/180)                    
        z = self.ip[2] + r*np.sin(k*np.pi/180)

        #x = self.finalgoal[0]; y = self.finalgoal[1]; z = self.finalgoal[2] #unsupress if you want to go to a fix goal

        xic = np.array([self.ip[0], self.iv[0], self.ia[0]])
        yic = np.array([self.ip[1], self.iv[1], self.ia[1]])
        zic = np.array([self.ip[2], self.iv[2], self.ia[2]])
        xfc = np.array([x, 0, 0])
        yfc = np.array([y, 0, 0])
        zfc = np.array([z, 0, 0])
        phi_ic = np.array([0, 0])
        final_phi = np.arctan((y- self.ip[1])/(x-self.ip[0]))
        print 'final_angle is:', final_phi, '(', final_phi*57.3, ')'
        phi_fc = np.array([final_phi, 0])
        
        T = self.get_T(self.finalgoal[0], self.finalgoal[1], self.finalgoal[2], self.ip[0], self.ip[1], self.ip[2], av_speed)
        #T = 1.0
        
        fig0 = plt.figure(figsize = (5, 4))
        ax0 = plt.axes(projection='3d')

        
        fig, axs = plt.subplots(4, figsize = (6, 12))
        fig.suptitle('velocity trajectory iterations')
 
        fig1, axs1 = plt.subplots(4, figsize = (6, 12))
        fig1.suptitle('acceleration trajectory iterations') 

        fig2, axs2 = plt.subplots(4, figsize = (6, 12))
        fig2.suptitle('jerk trajectory iterations')
        
        fig3, axs3 = plt.subplots(2, figsize = (6, 6))
        fig3.suptitle('thrust and angular velocity')
        
        fig4, axs4 = plt.subplots(2, figsize = (5, 4))
        fig4.suptitle('phi')
        
        T0 = T
        while self.regerate_trajectory: 
            ratio = T0/T
            t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, roots, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc)
            
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + az**2)
            j = np.sqrt(jx**2 + jy**2 + jz**2)
            thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
            ang_vel = j/thrust
            v_max = max(v); a_max = max(a); j_max = max(j)
            ang_vel_max = max (ang_vel)
            
            ax0.plot(x, y, z, linewidth = 2)
            
            axs[0].plot(t, v, linewidth = 2, label='z')
            axs[1].plot(t, vx, linewidth = 2, label='z')
            axs[2].plot(t, vy, linewidth = 2, label='z')
            axs[3].plot(t, vz, linewidth = 2, label='z')

            axs1[0].plot(t, a, linewidth = 2, label='z')
            axs1[1].plot(t, ax, linewidth = 2, label='z')
            axs1[2].plot(t, ay, linewidth = 2, label='z')
            axs1[3].plot(t, az, linewidth = 2, label='z')

            axs2[0].plot(t, j, linewidth = 2, label='z')
            axs2[1].plot(t, jx, linewidth = 2, label='z')
            axs2[2].plot(t, jy, linewidth = 2, label='z')
            axs2[3].plot(t, jz, linewidth = 2, label='z')
            
            axs3[0].plot(t, thrust, linewidth = 2, label='z')
            axs3[0].hlines(y=self.max_thrust, xmin=0, xmax=T, linewidth=2, color='r')
            axs3[1].plot(t, ang_vel, linewidth = 2, label='z')
            
            axs4[0].plot(t, phi, linewidth = 2, label='phi')
            axs4[1].plot(t, phir, linewidth = 2, label='phir')

            
            EE.append(E); EE1.append(E1); EE2.append(E2); TT.append(T) 
            
            #print 'time, maximum acc and ratio are:', T, a_max, a_max/self.max_acceleration
            #print min(thrust), max(thrust), min(ang_vel), max(ang_vel)
            #if a_max > self.max_acceleration or v_max >self.max_speed: 
            if  max(thrust) > self.max_thrust or max(ang_vel) > self.max_angular_velocity or a_max > self.max_acceleration: 
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
                            
        
        fig = plt.figure(figsize = (5, 4))
        plt.plot(TT, EE1, color = 'm', linewidth = 2, label='time')
        plt.plot(TT, EE2, color = 'c', linewidth = 2, label='energy')
        plt.plot(TT, EE, color = 'k', linewidth = 2, label='total')
        plt.legend(loc = 2, fontsize =14)
        plt.xlabel('time(s)', fontsize = 16)
        plt.ylabel('Cost', fontsize = 16)
        plt.xticks(size = 12);plt.yticks(size = 12)
        

        
        
        partialE, EE1, EE2, EE = [], [], [], []
        Tmax = 6
        TT = np.arange(T0, Tmax, 0.1)
        for i in range(len(TT)): 
            #print TT[i]
            t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, roots, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, TT[i], phi_ic, phi_fc)
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + (az)**2)
            EE.append(E); EE1.append(E1); EE2.append(E2)
            realroots = [rr.real if rr.imag == 0 else np.nan for rr in roots]
            realroots = [p for p in realroots if p > 0 and p is not np.nan]
            if i >= 1:
                partialE.append((EE[i] - EE[i-1]) /(TT[i]-TT[i-1]))

        fig = plt.figure(figsize = (5, 4))
        plt.plot(TT, EE1, color = 'm', linewidth = 2, label='Time Cost')
        plt.plot(TT, EE2, color = 'c', linewidth = 2, label='Jerk Integral')
        plt.plot(TT, EE, color = 'k', linewidth = 2, label='Total Cost')
        min_EE = EE.index(min(EE))
        min_EE2 = EE2.index(min(EE2))
        print '1', min_EE, min_EE2
        plt.vlines(x = T0, ymin = 0, ymax = EE[min_EE]+50, color = 'b', linewidth = 1)
        plt.vlines(x = T, ymin = 0, ymax = EE[min_EE]+50, color = 'g', linewidth = 1)
        plt.vlines(x = TT[min_EE], ymin = 0, ymax = EE[min_EE]+50, color = 'k', linewidth = 2)
        plt.vlines(x = TT[min_EE2], ymin = 0, ymax = EE[min_EE2]+50, color = 'c', linewidth = 1)
        for root in realroots:
            plt.vlines(x = root, ymin = 0, ymax = EE[min_EE]+50, color = 'm', linewidth = 2)
            print 'time corresponds to min energy: numerical: ', TT[min_EE], 'theoretical: ', root
        plt.legend(loc = 1, fontsize =14)
        plt.xlim(T0, Tmax); plt.ylim(0, max(EE))
        plt.xlabel('time(s)', fontsize = 16)
        plt.ylabel('Costs', fontsize = 16)

        plt.xticks(size = 12);plt.yticks(size = 12)
        
        
        plt.tight_layout()
        plt.show()
        print 'total time:', time.time()-tt
        


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.generate_trajectory()

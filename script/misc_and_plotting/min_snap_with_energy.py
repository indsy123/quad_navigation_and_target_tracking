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
        self.iv = np.array([3.0, 4.2, 0.0])
        self.ia = np.array([3.5, 2.3, 0.0])
        self.ij = np.array([1.0, 1.0, 0.0])
        
        self.finalgoal = np.array([8.0, 8.0, 2.0])
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
        k = 5e2
        c = self.min_snap_coefficients(xic, xfc, T)
        d = self.min_snap_coefficients(yic, yfc, T)
        e = self.min_snap_coefficients(zic, zfc, T)
        _phi = self.phi_coefficients(phi_ic, phi_fc, T)
        #f = open(self.path + 'coefficients.txt', 'a')
        #f.write("%s, %s\n" % ("local goal:", self.localgoal))
        #f.write("%s, %s, %s, %s\n" % (T, c, d, e))   
        
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

        E1 = k * T 
        f1 = c[0]**2/252; f2 = -c[0]*c[1]/36; f3 = (3*c[1]**2 + 4*c[0]*c[2])/60; f4 = - (c[0]*c[3]  + 3*c[1]*c[2])/12; f5 = (c[2]**2 + c[1]*c[3])/3; f6 = -c[2]*c[3]; f7 = c[3]**2
        g1 = d[0]**2/252; g2 = -d[0]*d[1]/36; g3 = (3*d[1]**2 + 4*d[0]*d[2])/60; g4 = - (d[0]*d[3]  + 3*d[1]*d[2])/12; g5 = (d[2]**2 + d[1]*d[3])/3; g6 = -d[2]*d[3]; g7 = d[3]**2
        h1 = e[0]**2/252; h2 = -e[0]*e[1]/36; h3 = (3*e[1]**2 + 4*e[0]*e[2])/60; h4 = - (e[0]*e[3]  + 3*e[1]*e[2])/12; h5 = (e[2]**2 + e[1]*e[3])/3; h6 = -e[2]*e[3]; h7 = e[3]**2
        
        k0 = c[0]; k1 = c[1]; k2 = c[2]; k3 = c[3]
        l0 = d[0]; l1 = d[1]; l2 = d[2]; l3 = d[3]
        m0 = e[0]; m1 = e[1]; m2 = e[2]; m3 = e[3]

        E2 = 0.5 * ((f1+g1+h1) * T**7 + (f2+g2+h2) * T**6 + (f3+g3+h3) * T**5 + (f4+g4+h4) * T**4 + (f5+g5+h5) * T**3 + (f6+g6+h6) * T**2 + (f7+g7+h7) * T)

        E3 = (1.0/2520) * T * (5* k0**2 * T**6 + 21 * (3*k1**2*T**4 - 5*k1*T**2 * (3*k2*T - 4*k3) + 20*(k2**2*T**2- 3*k3*k2*T +3*k3**2)) - 7*k0*T**3*(5*k1*T**2-12*k2*T+15*k3)) +\
             (1.0/2520) * T * (5* l0**2 * T**6 + 21 * (3*l1**2*T**4 - 5*l1*T**2 * (3*l2*T - 4*l3) + 20*(l2**2*T**2- 3*l3*l2*T +3*l3**2)) - 7*l0*T**3*(5*l1*T**2-12*l2*T+15*l3)) +\
             (1.0/2520) * T * (5* m0**2 * T**6 + 21 * (3*m1**2*T**4 - 5*m1*T**2 * (3*m2*T - 4*m3) + 20*(m2**2*T**2- 3*m3*m2*T +3*m3**2)) - 7*m0*T**3*(5*m1*T**2-12*m2*T+15*m3)) 
        E = E1 + E2
        # energy is ok
        
        #print 'where the fuck is the problem!!!!!!!!!'
        a1 = 840.0 * (xic[0] - xfc[0]); a2 = 840.0 * (yic[0] - yfc[0]); a3 = 840.0 * (zic[0] - zfc[0])
        b1 = 360.0 * xic[1]; b2 = 360.0 * yic[1]; b3 = 360.0 * zic[1]
        c1 = 60.0 * xic[2]; c2 = 60.0 * yic[2]; c3 = 60.0 * zic[2]
        d1 = 4.0 * xic[3]; d2 = 4.0 * yic[3]; d3 = 4.0 * zic[3]
        
        p0 = -2*k
        p1 = 0
        p2 = d1**2 + d2**2 + d3**2
        p3 = 2 * (c1*d1 + c2*d2 + c3*d3)
        p4 = c1**2 + c2**2 + c3**2 + 2 * (b1*d1 + b2*d2 + b3*d3)
        p5 = 2 * (a1*d1 + b1*c1 + a2*d2 + b2*c2 + a3*d3 + b3*c3)
        p6 = b1**2 + b2**2 + b3**2 + 2 * (a1*c1 + a2*c2 + a3*c3)
        p7 = 2 * (a1*b1 + a2*b2 + a3*b3)
        p8 = a1**2 + a2**2 + a3**2
        #print p0, p1, p2, p3, p4, p5, p6, p7, p8
        coeff = [p0, p1, p2, p3, p4, p5, p6, p7, p8]
        roots = np.roots(coeff)
        #print roots

        return t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, E3, roots, phi, phir
        
        
        

        
    def generate_trajectory(self): 
        tt = time.time()
      
        xx, yy, zz, TT = [], [], [], []   
        EE, EE1, EE2 = [], [], []
        r  = 6.0
        av_speed = self.max_speed * (r/self.range_max)
        j = 45; k = 45
        
        #x = self.ip[0] + r*np.cos(j*np.pi/180)*np.cos(k*np.pi/180)                    
        #y = self.ip[1] + r*np.sin(j*np.pi/180)*np.cos(k*np.pi/180)                    
        #z = self.ip[2] + r*np.sin(k*np.pi/180)

        x = self.finalgoal[0]; y = self.finalgoal[1]; z = self.finalgoal[2] #unsupress if you want to go to a fix goal

        xic = np.array([self.ip[0], self.iv[0], self.ia[0], self.ij[0]])
        yic = np.array([self.ip[1], self.iv[1], self.ia[1], self.ij[1]])
        zic = np.array([self.ip[2], self.iv[2], self.ia[2], self.ij[2]])
        
        xfc = np.array([x, 0, 0, 0])
        yfc = np.array([y, 0, 0, 0])
        zfc = np.array([z, 0, 0, 0])
        phi_ic = np.array([0, 0, 0])
        final_phi = np.arctan((y- self.ip[1])/(x-self.ip[0]))
        print 'final_angle is:', final_phi, '(', final_phi*57.3, ')'
        phi_fc = np.array([final_phi, 0])
        
        T = self.get_T(x, y, z, self.ip[0], self.ip[1], self.ip[2], av_speed)
        #T = 1.0
        
        #fig0 = plt.figure(figsize = (5, 4))
        #ax0 = plt.axes(projection='3d')

        fig5, axs5 = plt.subplots(3, figsize = (6, 9))
        fig5.suptitle('position trajectory iterations')
        
        fig, axs = plt.subplots(4, figsize = (6, 12))
        fig.suptitle('velocity trajectory iterations')
 
        fig1, axs1 = plt.subplots(4, figsize = (6, 12))
        fig1.suptitle('acceleration trajectory iterations') 

        fig2, axs2 = plt.subplots(4, figsize = (6, 12))
        fig2.suptitle('jerk trajectory iterations')
        
        #fig3, axs3 = plt.subplots(2, figsize = (6, 6))
        #fig3.suptitle('thrust and angular velocity')
        
        #fig4, axs4 = plt.subplots(2, figsize = (5, 4))
        #fig4.suptitle('phi')
        
        T0 = T
        while self.regerate_trajectory: 
            ratio = T0/T
            t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, E3, roots, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, T, phi_ic, phi_fc)
            
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + az**2)
            j = np.sqrt(jx**2 + jy**2 + jz**2)
            thrust = self.mass * np.sqrt(ax**2 + ay**2 + (az+9.8)**2)
            ang_vel = j/thrust
            v_max = max(v); a_max = max(a); j_max = max(j)
            ang_vel_max = max (ang_vel)
            """
            #ax0.plot(x, y, z, linewidth = 2)
 
            axs5[0].plot(t, x, linewidth = 2, label='z')
            axs5[1].plot(t, y, linewidth = 2, label='z')
            axs5[2].plot(t, z, linewidth = 2, label='z')
            
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
            
            #axs3[0].plot(t, thrust, linewidth = 2, label='z')
            #axs3[0].hlines(y=self.max_thrust, xmin=0, xmax=T, linewidth=2, color='r')
            #axs3[1].plot(t, ang_vel, linewidth = 2, label='z')
            
            #axs4[0].plot(t, phi, linewidth = 2, label='phi')
            #axs4[1].plot(t, phir, linewidth = 2, label='phir')
            """
            
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
        plt.plot(TT, EE1, color = 'm', linewidth = 2, linestyle = ':', label='time')
        plt.plot(TT, EE2, color = 'c', linewidth = 2, linestyle = '--',label='energy')
        plt.plot(TT, EE, color = 'k', linewidth = 2, label='total')
        plt.legend(loc = 2, fontsize =14)
        plt.xlabel('time(s)', fontsize = 16)
        plt.ylabel('Cost', fontsize = 16)
        plt.xticks(size = 12);plt.yticks(size = 12)
        

        
        
        partialE, EE1, EE2, EE, EE3 = [], [], [], [], []
        EE_new = []
        Tmax = 6
        TT = np.arange(T0, Tmax, 0.05)

        for i in range(len(TT)): 
            #print TT[i]
            t, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, E1, E2, E, E3, roots, phi, phir = self.trajectory(xic, yic, zic, xfc, yfc, zfc, TT[i], phi_ic, phi_fc)
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + az**2)
            j = np.sqrt(jx**2 + jy**2 + jz**2)           
            axs5[0].plot(t, x, linewidth = 2, label='z')
            axs5[1].plot(t, y, linewidth = 2, label='z')
            axs5[2].plot(t, z, linewidth = 2, label='z')
            
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
            v = np.sqrt(vx**2 + vy**2 + vz**2)
            a = np.sqrt(ax**2 + ay**2 + (az)**2)
            EE.append(E); EE1.append(E1); EE2.append(E2); EE3.append(E3)
            EE_new.append(E1+E3)
            #print roots
            realroots = [rr.real if rr.imag == 0 else np.nan for rr in roots]

            realroots = [p for p in realroots if p > 0 and p is not np.nan]
            if i >= 1:
                partialE.append((EE[i] - EE[i-1]) /(TT[i]-TT[i-1]))

        fig = plt.figure(figsize = (5, 4))
        plt.plot(TT, EE1, color = 'g', linewidth = 2, linestyle = ':', label='Time Cost')
        plt.plot(TT, EE2, color = 'g', linewidth = 2, linestyle = '--', label='Snap Integral')
        plt.plot(TT, EE, color = 'g', linewidth = 2, label='Total Cost')
        #plt.plot(TT, EE_new, color = 'g', linewidth = 2, linestyle = '--', label='Snap Integral2')

        min_EE = EE.index(min(EE))
        min_EE_new = EE_new.index(min(EE_new))
        min_EE2 = EE2.index(min(EE2))
        print '1', min_EE, min_EE2
        #plt.vlines(x = T0, ymin = 0, ymax = EE[min_EE]+50, color = 'b', linewidth = 1)
        #plt.vlines(x = T, ymin = 0, ymax = EE[min_EE]+50, color = 'g', linewidth = 1)
        #plt.vlines(x = TT[min_EE], ymin = 0, ymax = EE[min_EE]+50, color = 'k', linewidth = 2)
        #plt.vlines(x = TT[min_EE_new], ymin = 0, ymax = EE[min_EE2]+50, color = 'c', linewidth = 1)
        print realroots
        for root in realroots:
            plt.vlines(x = root, ymin = 0, ymax = EE[min_EE]+50, color = 'g', linewidth = 2)
            print 'time corresponds to min energy: numerical: ', TT[min_EE], 'theoretical: ', root
        plt.legend(loc = 1, fontsize =14)
        plt.xlim(T0, Tmax); plt.ylim(0, max(EE))
        plt.xlabel('time (s)', fontsize = 16)
        plt.ylabel('Cost', fontsize = 16)

        plt.xticks(size = 12);plt.yticks(size = 12)
        
        
        plt.tight_layout()
        plt.show()
        print 'total time:', time.time()-tt
        


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.generate_trajectory()

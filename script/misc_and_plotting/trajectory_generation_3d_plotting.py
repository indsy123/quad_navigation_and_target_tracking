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
from scipy.spatial import cKDTree


class polynomial_trajetory(object):
    def __init__(self): 
        """initializes variables"""
        self.ip = np.array([0.0, 0.0, 3.0])
        self.iv = np.array([4.0, 0.0, 0.0])
        self.ia = np.array([2.0, 0.0, 0.0])
        self.finalgoal = np.array([8.0, 0.0, 3.0])
        self.colors = ['r', 'b', 'g', 'm', 'c' ]
        self.psi0 = np.array([0.0, 0.0])
        self.safety_radius = 0.35
        self.dist2goalcost = []
        self.collisioncost = []
        self.validendpoints = []

    def generate_collision_points(self):
        x = 3.5
        #nx = int()
        #y = np.linspace(0.5, 1.0, 21); z = np.linspace(0, 1.5, 41)
        y = np.linspace(0.1, 1.2, 21); z = np.linspace(1.5, 3.5, 41)
        pp = np.zeros((0,3))
        yy, zz = np.meshgrid(y, z)
        p = np.stack((yy.ravel(), zz.ravel()), axis = -1)
        p = np.column_stack((np.ones(len(p))*x, p))  
        pp = np.concatenate((pp, p), 0)
        return pp, (0, 20, 420, 440, 840, 860)
        
       
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


    def getcost(self, c, d, e, T, s, axx, plane_no, occ_points, indexes): 
        SR =  0.5#a safety radius for collision cost
        factor = (1 + SR**4)/SR**4
       
        #mycolor = (0 + s/10.0, 0, 0)
        #axx.plot(x, y, z, color ='0.5', linewidth = 1)
        # check collision
        interval = 30 
        t = np.linspace(0, T, interval+1)
        xx = -c[0]*t**5/120 + c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
        yy = -d[0]*t**5/120 + d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + c[5]
        zz = -e[0]*t**5/120 + e[1]*t**4/24 - e[2]*t**3/6 + e[3]*t**2/2 + e[4]*t + e[5]  
        points_on_traj = np.column_stack((xx[np.newaxis].T, yy[np.newaxis].T, zz[np.newaxis].T))
            
        kdt_op = cKDTree(occ_points)

        #indices = np.array(kdt_p.query_ball_tree(kdt_op, self.safety_radius, eps = 0.1))
        indices = np.array(kdt_op.query_ball_point(points_on_traj, self.safety_radius, eps = 0.1))
        remove = []
        for i in range(len(indices)): 
            if any(indices[i]): 
                remove.append(i)
                
        if len(remove) == 0: 
            #axx.plot(x, y, z, color ='g', linewidth = 1)
            distance = kdt_op.query(points_on_traj, k = 1, eps = 0.1, p = 2, n_jobs = -1)
            distance = distance[0].reshape(-1, interval+1)
            distance = np.min(distance, axis = 1)
            if distance[0]-self.safety_radius < SR: 
                Wcollision = factor* ((distance[0]-self.safety_radius)**2- SR**2)**2/ (1 + ((distance[0]-self.safety_radius)**2- SR**2)**2)
            else:
                Wcollision = 0
                
            self.collisioncost.append(Wcollision)
            dist = np.sqrt((self.finalgoal[0] - xx[-1])**2 + (self.finalgoal[1] - yy[-1])**2 + (self.finalgoal[2] - zz[-1])**2)
            #dist = dist/np.linalg.norm(self.finalgoal-self.ip)
            self.dist2goalcost.append(dist)
            self.validendpoints.append([xx[-1], yy[-1], zz[-1]])
                
        #else:
        #    Wcollision = np.nan
        #    axx.plot(x, y, z, color ='r', linewidth = 1)

        
        #plt.plot(x, y, color = mycolor, linewidth = 3-s/10, label='trajectory')
        #plt.xlabel('x(m)', fontsize = 16)
        #plt.ylabel('y(m)', fontsize = 16)
        #plt.xticks(size = 16);plt.yticks(size = 16)


       
    def min_jerk_plot(self, c, d, e, T, s, axx, point_no): 
       
        
        t = np.linspace(0, T, 101)
        x = -c[0]*t**5/120 + c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
        y = -d[0]*t**5/120 + d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + c[5]
        z = -e[0]*t**5/120 + e[1]*t**4/24 - e[2]*t**3/6 + e[3]*t**2/2 + e[4]*t + e[5]
        
        vx = -c[0]*t**4/24 + c[1]*t**3/6 - c[2]*t**2/2 + c[3]*t + c[4]
        vy = -d[0]*t**4/24 + d[1]*t**3/6 - d[2]*t**2/2 + d[3]*t + d[4]
        vz = -e[0]*t**4/24 + e[1]*t**3/6 - e[2]*t**2/2 + e[3]*t + e[4]
        
        ax = -c[0]*t**3/6 + c[1]*t**2/2 - c[2]*t + c[3]
        ay = -d[0]*t**3/6 + d[1]*t**2/2 - d[2]*t + d[3]
        az = -e[0]*t**3/6 + e[1]*t**2/2 - e[2]*t + e[3]
        
        
        #if self.collisioncost[point_no] >= 0.5:
        #    mycolor = (0 , 0 + self.collisioncost[point_no], 0)
        #else:
        #    mycolor = (0, 0.5, 0)
        print '1', self.collisioncost[point_no], self.dist2goalcost[point_no]
        
        #axx.plot(x, y, z, color = str(self.collisioncost[point_no]), linewidth = 2)
        #alpha = 0.1
        #if self.totalcost[point_no] == 0:
        #    alpha = 0.1
        if self.totalcost[point_no] >= 0 and self.totalcost[point_no] < 0.1:
            alpha = 0.1
        else:
            alpha = self.totalcost[point_no]
        axx.plot(x, y, z, color = 'g', alpha = alpha, linewidth = 2)
        #axx.plot(x, y, z, color = mycolor, linewidth = 2)
        
        plt.xlabel('x(m)', fontsize = 16)
        plt.ylabel('y(m)', fontsize = 16)
        plt.xticks(size = 16);plt.yticks(size = 16)


    def plot_sphere(self, center, fig, ax): 
        
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = self.safety_radius * np.outer(np.cos(u), np.sin(v)) + center[0]
        y = self.safety_radius * np.outer(np.sin(u), np.sin(v)) + center[1]
        z = self.safety_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center [2]
        ax.plot_surface(x, y, z, edgecolor="r", color="w", alpha=0.01, linewidth=0.1)
        
    def generate_trajectory(self): 
        t = time.time()
        max_speed = 10.0
        nplanes = 3
        range_min = 3.0
        range_max = 6.0
        theta_y = 60
        theta_z = 60
        _del = 20
        
        d = (range_max - range_min)/(nplanes-1)
        
        fig = plt.figure(figsize = (10, 8))
        ax = fig.add_subplot(111, projection='3d')
        # triad at the ground
        ax.plot((0,1), (0,0), (0, 0), c = 'g', linewidth = 2)
        ax.plot((0,0), (0,1), (0, 0), c = 'r', linewidth = 2)
        ax.plot((0,0), (0,0), (0, 1), c = 'b', linewidth = 2)
        
        ax.plot((0,0), (0,0), (0, 1), c = 'k', linewidth = 1, linestyle = ':')      
        # triad at the quad location
        ax.plot((0,1), (0,0), (3, 3), c = 'g', linewidth = 2) 
        ax.plot((0,0), (0,1), (3, 3), c = 'r', linewidth = 2)
        ax.plot((0,0), (0,0), (3, 4), c = 'b', linewidth = 2)
        
        # triad at the final goal location
        ax.plot((self.finalgoal[0],self.finalgoal[0]+1), (self.finalgoal[1],self.finalgoal[1]), (self.finalgoal[2],self.finalgoal[2]), c = 'g', linewidth = 2) 
        ax.plot((self.finalgoal[0],self.finalgoal[0]), (self.finalgoal[1],self.finalgoal[1]+1), (self.finalgoal[2],self.finalgoal[2]), c = 'r', linewidth = 2)
        ax.plot((self.finalgoal[0],self.finalgoal[0]), (self.finalgoal[1],self.finalgoal[1]), (self.finalgoal[2],self.finalgoal[2]+1), c = 'b', linewidth = 2)
        
        ax.text(-0.2, 0.5, 3.2, "quad", color='k', fontsize = 16)
        ax.text(self.finalgoal[0]-0.2, self.finalgoal[1]+0.5, self.finalgoal[2]+0.2, "goal", color='k', fontsize = 16)
        
        occ_points, indexes = self.generate_collision_points()
                    
                
        
        for i in range(nplanes):             
            r  = range_min + d * i
            av_speed = (r/range_max) * max_speed
            print r, av_speed
            m = i%2 
            xx, yy, zz = [], [], []
            for j in range(-theta_y/2 + m*_del/2, theta_y/2+1, _del):
            #for j in range(-theta_y/2 , theta_y/2+1, _del):
                for k in range(-theta_z/2 + m*_del/2 , theta_z/2+1, _del):
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
                    
                    x_coefficients = self.min_jerk_coefficients(xic, xfc, T)
                    y_coefficients =  self.min_jerk_coefficients(yic, yfc, T)
                    z_coefficients =  self.min_jerk_coefficients(zic, zfc, T)
                    self.getcost(x_coefficients, y_coefficients, z_coefficients, T, av_speed, ax, i, occ_points, indexes)
                    
            #theta = np.linspace(-theta_y/2, theta_y/2, 40)
            #phi = np.linspace(-theta_z/2, theta_z/2, 40)

            #for p in theta:
            #    for q in phi:
            #        xx.append(self.ip[0] + r*np.cos(p*np.pi/180)*np.cos(q*np.pi/180))                   
            #        yy.append(self.ip[1] + r*np.sin(p*np.pi/180)*np.cos(q*np.pi/180))                   
            #        zz.append(self.ip[2] + r*np.sin(q*np.pi/180))

            #ax.scatter(xx, yy, zz, c=self.colors[i], marker='.', alpha = 0.25)

        #print self.collisioncost, self.dist2goalcost, self.validendpoints
        index = 0    
        ip_index = self.dist2goalcost.index(min(self.dist2goalcost))
        print 'intermediate point', ip_index, self.validendpoints[ip_index]
        ax.scatter(self.validendpoints[ip_index][0], self.validendpoints[ip_index][1], self.validendpoints[ip_index][2], color='k')
        ax.text(-0.2+self.validendpoints[ip_index][0], 0.1 + self.validendpoints[ip_index][1], 0.1 + self.validendpoints[ip_index][2], "IP", color='k', fontsize = 16)


        
        self.dist2goalcost = self.dist2goalcost/max(self.dist2goalcost)
        self.totalcost = map(lambda i, j: 1.0*i+1.0*j, self.dist2goalcost, self.collisioncost)
        self.totalcost = self.totalcost/max(self.totalcost)

        lp_index = np.argmin(self.totalcost)
        print 'local_goal', lp_index, self.validendpoints[lp_index]
        ax.scatter(self.validendpoints[lp_index][0], self.validendpoints[lp_index][1], self.validendpoints[lp_index][2], color='g')
        ax.text(-0.2+self.validendpoints[lp_index][0], 0.1 + self.validendpoints[lp_index][1], 0.2 + self.validendpoints[lp_index][2], "LG", color='k', fontsize = 16)


        
        

        for i in self.validendpoints:
            xic = np.array([self.ip[0], self.iv[0], self.ia[0]])
            yic = np.array([self.ip[1], self.iv[1], self.ia[1]])
            zic = np.array([self.ip[2], self.iv[2], self.ia[2]])
            xfc = np.array([i[0], 0, 0])
            yfc = np.array([i[1], 0, 0])
            zfc = np.array([i[2], 0, 0])
            #T = np.linalg.norm(end-self.start)/av_speed
            T = np.sqrt((i[0]-self.ip[0])**2 + (i[1]-self.ip[1])**2  + (i[2]-self.ip[2])**2)/av_speed
            
            x_coefficients = self.min_jerk_coefficients(xic, xfc, T)
            y_coefficients =  self.min_jerk_coefficients(yic, yfc, T)
            z_coefficients =  self.min_jerk_coefficients(zic, zfc, T)
            
            self.min_jerk_plot(x_coefficients, y_coefficients, z_coefficients, T, av_speed, ax, index)
            index += 1
            
        
        for i in indexes: 
            self.plot_sphere(occ_points[i], fig, ax)
            
        ax.scatter(occ_points[:, 0], occ_points[:, 1], occ_points[:, 2], c = 'r', marker = 'o', s = 1.5)
        
        ax.tick_params(direction='out', length=6, width=2, colors='k', labelsize=18)
        ax.set_xlabel("$x(m)$", labelpad = 13, size = 30)
        ax.set_ylabel("$y(m)$", labelpad = 13, size = 30)
        #ax.arrow(0, 0, 0.0, 0.02, fc='g', ec='k', lw = 0.01, head_width=0.004, head_length=0.002, overhang = 0, clip_on = False)
        ax.set_zlabel("$z(m)$", labelpad = 13, size = 30)
        fig = plt.figure(figsize = (5, 4))
        plt.plot(range(len(self.validendpoints)), self.collisioncost, 'r', range(len(self.validendpoints)), self.dist2goalcost, 'b', range(len(self.validendpoints)), self.totalcost, 'g')
        
        
        plt.tight_layout()
        plt.show()
        print 'total time:', time.time()-t
    


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.generate_trajectory()

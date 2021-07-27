#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 09:59:56 2020
first play with RANSAC in python 
@author: ind
"""
import time, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from mpl_toolkits.mplot3d import Axes3D
from ast import literal_eval



class box_plot(object):
    def __init__(self): 
        """initializes various paramters"""
        directory = '/home/ind/data_collection/TRO_dataset/new/'    
        
        f1 = os.path.join(directory,'f1.txt')
        f2 = os.path.join(directory,'f2.txt')
        f3 = os.path.join(directory,'f3.txt')
        f4 = os.path.join(directory,'f4.txt')
        f5 = os.path.join(directory,'f5.txt')
        files = [f1, f2, f3, f4, f5]
        self.planning_time = []
        for i in files: 
            pt = []       
            data = open(i, 'r')
            for line in data: 
                temp = line.split(",")
                print temp
                pt.append(float(temp[2]))
            self.planning_time.append(pt)           


    def boxes(self): # just for test

        bp0 = plt.boxplot(self.planning_time, 0, '', whis = [5,95])
        for i in bp0: 
            print i
        for box in bp0['boxes']: 
            box.set(color='k', linewidth=2)
        for median in bp0['medians']: 
            median.set(color='red', linewidth=2)
        for flier in bp0['fliers']: 
            flier.set(color='k', linewidth=2)
        for whisker in bp0['whiskers']: 
            whisker.set(color='k', linewidth=2)
        for cap in bp0['caps']: 
            cap.set(color='k', linewidth=2)
            
        plt.xlabel('Run Number', fontsize = 18)
        plt.ylabel('Replanning Execution Time(s)', fontsize = 18)
        plt.xticks(size=16); plt.yticks(size=16)
        plt.tight_layout()
        
    def velocity_vs_obs_density(self): 
        vel = [2, 3, 4, 5, 6, 7]
        prob18 = [1.0, 1.0, 1.0, 0.9, 0.8, 0.7]
        prob36 = [0.9, 0.9, 0.8, 0.7, 0.6, 0.5]
        fig, ax1 = plt.subplots()
        color = 'tab:red'
        ax1.set_xlabel('Velocity (m/s)', fontsize = 18)
        ax1.set_ylabel('Probability (0.18 obstacles per $m^2$) ', color=color, fontsize = 18)
        ax1.plot(vel, prob18, color=color)
        ax1.tick_params(axis='y', labelcolor=color)
        plt.ylim(0,1.2)
        plt.xticks(size=16); plt.yticks(size=16)
        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
         
        color = 'tab:blue'
        ax2.set_ylabel('Probability (0.36 obstacles per $m^2$)', color=color, fontsize = 18)  # we already handled the x-label with ax1
        ax2.plot(vel, prob36, color=color)
        ax2.tick_params(axis='y', labelcolor=color)
        plt.ylim(0,1.2)
        plt.xticks(size=16); plt.yticks(size=16)
        plt.tight_layout()   
        
 
if __name__ == '__main__':

    f = box_plot()
    f.boxes()
    f.velocity_vs_obs_density()

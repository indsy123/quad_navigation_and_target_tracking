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
from scipy import stats



class k_vs_vel(object):
    def __init__(self): 
        """initializes various paramters"""
        directory = '/home/ind/data_collection/TRO_dataset/k_vs_vel/'    
        
        f1 = os.path.join(directory,'data_for_k_1.txt')
        f2 = os.path.join(directory,'data_for_k_2.txt')
        f3 = os.path.join(directory,'data_for_k_3.txt')
        f4 = os.path.join(directory,'data_for_k_4.txt')
        f5 = os.path.join(directory,'data_for_k_5.txt')
        f6 = os.path.join(directory,'data_for_k_6.txt')
        f7 = os.path.join(directory,'data_for_k_7.txt')
        files = [f1, f2, f3, f4, f5, f6, f7]
        self.velocity = []
        self.k = []
        for i in files: 
            vel, k = [], []       
            data = open(i, 'r')
            for line in data: 
                temp = line.split(",")
                #print temp
                vel.append(float(temp[0]))
                k.append(float(temp[4]))
            self.k.append(k)   
            self.velocity.append(vel)


    def boxes(self): # just for test
        k_array = np.array(self.k)
        vel_array = [1,2,3,4,5,6,7]
        k_mean, k_median, k_mode = [],[],[]
        
        for i in range(len(k_array)): 
            a = np.mean(k_array[i])
            k_mean.append(a)
            b = np.median(k_array[i])
            k_median.append(b)
            c = stats.mode(k_array[i])
            k_mode.append(c)
        print vel_array, k_mean, k_median
        
        bp0 = plt.boxplot(self.k, 0, '', whis = [25,75])

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
        plt.ylabel('k', fontsize = 18)
        plt.xticks(size=16); plt.yticks(size=16)
        plt.tight_layout()
        

 
if __name__ == '__main__':

    f = k_vs_vel()
    f.boxes()
    
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
#import time
import numpy as np
import rospy, math, random, bisect
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import time
from quad_navigation_and_target_tracking.msg import DetectionData
from matplotlib import pyplot as plt


class detection_calculations(object): 
    def __init__(self, run_number, odometry_topic, target_topic, count_topic, freq, a, beta, chi, alpha, counts_freq):
        """initialization"""
        rospy.init_node('detection_calculations', anonymous=True, log_level=rospy.DEBUG)
        self.file = '/home/pelican/data_collection/TRO/detection_data'+str(run_number)+'.txt'
        self.file2 = '/home/pelican/data_collection/TRO/counts_vs_reldist'+str(run_number)+'.txt' 
        #open(self.path + 'detection_data.txt', 'w').close()
        open(self.file, 'w').close()

        # detetion parameters
        self.detection_counter = 0
        self.odometry_topic = str(odometry_topic)
	self.target_topic = str(target_topic)
	self.count_topic = str(count_topic)
        self.a = a
        self.b = beta
        self.chi = chi
        self.alpha = alpha
        self.odometry_freq = freq
        self.dt = 1.0/counts_freq

        # variables and lists
        self.start_time = time.time()
        self.source_available = False
        self.quad_available = False
        self.mu = [] 
        self.counts = []
        self.actual_counts = []
        self.counts_time = []  
        self.pmd_vector = [] # list that stores the values of probability of missed detection 
        self.detection_threshold_vector = [] # list that stores the detection metric
        self.LR_vector= []

        
        self.detection_data = rospy.Publisher('/udrone1/detection_data', DetectionData, queue_size = 1, tcp_nodelay = True)
        try:
            rospy.Subscriber(self.odometry_topic, Odometry, self.quadCb, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber(self.target_topic, Odometry, self.targetCb, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber(self.count_topic, PointStamped, self.countsCb, queue_size = 1, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the topic above')

        try:
            while not rospy.is_shutdown():             
                rospy.spin()
        except rospy.ROSInterruptException(): 
            pass


    def targetCb(self, data):
        self.source_location = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]) 
        self.source_available = True


    def quadCb(self, data):
        if self.source_available: 
            self.quad_location = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]) 
            self.d = np.linalg.norm(self.quad_location-self.source_location)
            self.instant_mu = 1+((self.chi*self.a/self.b)/(2*self.chi+self.d*self.d))
            self.quad_available = True
        
        
    def find_p(self, _mu):
        pp = 0.0; self.got_p = False
        while pp < 1.0: 
            pfa_exp_list = [(j**pp - pp* j**pp * np.log(j) - 1)*self.b*self.dt for j in _mu]
            #print 'alpha', np.exp(sum(pfa_exp_list)), self.alpha
            if np.exp(sum(pfa_exp_list)) < self.alpha: 
                self.got_p = True; #_p = pp
                break
            else: 
                pp = pp + 0.075

        if self.got_p == True: 
            return pp, np.exp(sum(pfa_exp_list))
        else: 
            #print 'could not find a suitable value of parameter p for this set of mu'
            return 0.0, 1.0


         
    def countsCb(self, data):
        """Detection calculations, publish the data on a topic"""
        if self.source_available and self.quad_available:
            if data.point.x >= 1:
                self.actual_counts.append(data.point.x)
                f1 = open(self.file2, 'a')
                f1.write("%s, %2.2f, %2.2f, %2.2f\n" % (time.time(), self.d, self.instant_mu, len(self.actual_counts)))

            self.counts.append(data.point.x)
            self.counts_time.append(time.time()-self.start_time)
            #self.total_counts = sum(self.counts)
            #print 'total obtained counts are:', self.total_counts

            self.mu.append(self.instant_mu)            
            count_multiplication = 1.0
            vi_integration = 0.0
            p, pfa = self.find_p(self.mu)
            vi_vector = [(1-m)*self.dt*self.b for m in self.mu]
            vi_integration = sum(vi_vector)
            for i in range(len(self.counts)):                
                if self.counts[i] >= 1:
                    #print 'counts and mu:', self.counts[i], self.mu[i], self.mu[i]**self.counts[i]
                    count_multiplication = count_multiplication * self.mu[i]**self.counts[i]
		    #self.actual_counts.append(1)
            #print '2:', np.exp(vi_integration), count_multiplication
            LR = np.exp(vi_integration)*count_multiplication
            self.LR_vector.append(LR) 
            if p == 0: 
                self.pmd_exponent = 0.0; self.detection_threshold = 0.0
                self.pmd_vector.append(1.0)
                self.detection_threshold_vector.append(0.0)
               
            else:                    
                pmd_exponent_vector = [(mu**p+(1-p)*mu**p*np.log(mu)-mu)*self.b*self.dt for mu in self.mu]
                self.pmd_exponent = sum(pmd_exponent_vector)
                self.pmd_vector.append(np.exp(self.pmd_exponent))
                _detection_threshold_vector = [(mu**p*np.log(mu)-mu+1)*self.b*self.dt for mu in self.mu]
                self.detection_threshold = np.exp(sum(_detection_threshold_vector))                
                self.detection_threshold_vector.append(self.detection_threshold) 
                
                if LR > self.detection_threshold:
                    print 'The souce is detected'
            
            f = open(self.file, 'a')
            f.write("%s, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n" \
                    % (time.time(), self.counts_time[-1], self.d, self.instant_mu, p, pfa, np.exp(self.pmd_exponent), len(self.actual_counts), vi_integration, count_multiplication, LR, self.detection_threshold))
            

            msg = DetectionData()    	    
            msg.header.stamp = data.header.stamp

            msg.mu = self.instant_mu
            msg.p = p
            msg.BackgroundRate = self.b
            msg.DetectorCrossSection = self.chi
            msg.DetectorTime = self.counts_time[-1]
            msg.PmdExponent = np.exp(self.pmd_exponent)
            msg.DetectionThreshold = self.detection_threshold
            msg.LikelihoodRatio = LR
            self.detection_data.publish(msg)
    


# if python says run, then we should run
if __name__ == '__main__':
    run_number = rospy.get_param('run_number')
    odometry_topic = rospy.get_param('odometry_topic')
    target_topic = rospy.get_param('target_topic')
    count_topic = rospy.get_param('count_topic')
    freq = rospy.get_param('odometry_frequency')
    a = rospy.get_param('source_strength_cps')
    beta = rospy.get_param('background_cps')
    chi = rospy.get_param('sensor_cross_section')
    alpha = rospy.get_param('alpha')
    counts_freq = rospy.get_param('counts_frequency')

    #r = rospy.Rate(30)    
    traj = detection_calculations(run_number, odometry_topic, target_topic, count_topic, freq, a, beta, chi, alpha, counts_freq)

    

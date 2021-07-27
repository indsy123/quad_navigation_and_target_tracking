#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
#import time
import numpy as np
import rospy, math, random, bisect
from nav_msgs.msg import Odometry
import time
from quad_navigation_and_target_tracking.msg import DetectionData
from matplotlib import pyplot as plt


class detection_calculations(object): 
    def __init__(self, odometry_topic, freq, a, beta, chi, alpha):
        """initialization"""
	rospy.init_node('detection_calculations', anonymous=True, log_level=rospy.DEBUG)
	self.file = '/home/ind/data_collection/TRO_dataset/detection/detection_data50.txt'
	#open(self.path + 'detection_data.txt', 'w').close()
	open(self.file, 'w').close()

        # detetion parameters
        self.detection_counter = 0
        #self.uav = name
        self.odometry_topic = str(odometry_topic)
        self.source_intensity = a
        self.background = beta
        self.sensor_crosssection = chi
        self.alpha = alpha
        self.odometry_freq = freq
        self.dt_detection = (1.0/freq)*20.0 # becasue I am doing calculations every 0.2 sec
        #self.end_point = np.asarray(goal)
        # for now source is assumed at the ground level of the goal point, add another vector in launch file if different
        self.source_available = False
        self.mu = [] # its a list that keeps accumulating the values of the mu as per the current location, after p is available 
        self.pmd_vector = [] # list that stores the values of probability of missed detection 
        self.detection_threshold_vector = [] # list that stores the detection metric
        self.time_vector = []
	#self.time_at_detection = []
        self.sensor_position_vector = []
	self.source_position_vector = []
        self.LR_vector= []
	self.tau = [] # a list that stores counts
	self.source_pv_previous = []
	self.sensor_pv_previous = []
	self.time_vector_previous = []
        self.vi_integration = 0
        self.count_multiplication = 1.0
        
        self.detection_data = rospy.Publisher('/firefly1/detection_data', DetectionData, queue_size = 1, tcp_nodelay = True)
        try:
            rospy.Subscriber(self.odometry_topic, Odometry, self.detection_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/jackal/ground_truth/state', Odometry, self.target_callback, queue_size = 1, tcp_nodelay = True)

        except:
            print('problem subscribing to one of the topic above')

        try:
            while not rospy.is_shutdown():             
                rospy.spin()
        except rospy.ROSInterruptException(): 
            pass


    def target_callback(self, data):
	self.source_location = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]) 
	self.source_available = True

    def find_p(self, _mu):
        pp = 0.0; self.got_p = False
        while pp < 1.0: 
            pfa_exp_list = [(j**pp - pp* j**pp * np.log(j) - 1)*self.background*self.dt_detection for j in _mu]
            #print 'alpha', np.exp(sum(pfa_exp_list)), self.alpha
            if np.exp(sum(pfa_exp_list)) < self.alpha: 
                self.got_p = True; #_p = pp
		#print 'pp1:', pp
                break
            else: 
		#print 'pp2:', pp
                pp = pp + 0.01

        if self.got_p == True: 
            return pp, np.exp(sum(pfa_exp_list))
        else: 
            #print 'could not find a suitable value of parameter p for this set of mu'
            return 0.0, 1.0


    def generate_counts_onlybackground(self, spos_, tpos_, T):
        """This function generates number of counts for each sensor based on its 
        position and the position of the moving source"""

	T = [i-T[0] for i in T]

        max_possible_nu = ((self.source_intensity*self.sensor_crosssection)/(2*self.sensor_crosssection + 0.25**2))
        vimax = max_possible_nu#self.source_intensity/2 

        t_nh = 0;  n_nh = 0 # initialize a number variable         
        ET_nh = []; ET_h = [] # initialise an array for storing event times
       
        t_h = -math.log(random.uniform(0,1))/self.background; n_h = 0     
        while t_h < T[-1]:
            n_h = n_h + 1; ET_h.append(t_h)
            t_h = t_h - math.log(random.uniform(0,1))/self.background
            
        Event_time = sorted(ET_nh + ET_h)
        #print ET_h, ET_nh
        return Event_time

 
    def generate_counts(self, spos_, tpos_, T):
        """This function generates number of counts for each sensor based on its 
        position and the position of the moving source"""

	T = [i-T[0] for i in T]

        max_possible_nu = ((self.source_intensity*self.sensor_crosssection)/(2*self.sensor_crosssection + 0.25**2))
        vimax = max_possible_nu#self.source_intensity/2 

        t_nh = 0;  n_nh = 0 # initialize a number variable         
        ET_nh = []; ET_h = [] # initialise an array for storing event times
        
        while t_nh < T[-1]:  
            t_nh = t_nh - (math.log(random.uniform(0,1)) / vimax)
            index = bisect.bisect_left(T, t_nh)
            if index == 0: 
                pos_at_t_nh = spos_[0]
		tpos_at_t_nh = tpos_[0]
            elif index > len(T)-2: 
                pos_at_t_nh = spos_[-1]
		tpos_at_t_nh = tpos_[-1]
            else: 
                pos_at_t_nh = (spos_[index] + spos_[index+1])/2.0
                tpos_at_t_nh = (tpos_[index] + tpos_[index+1])/2.0

            ratio = ((self.source_intensity*self.sensor_crosssection)/(2*self.sensor_crosssection + (np.linalg.norm(pos_at_t_nh-tpos_at_t_nh))**2))/vimax 
            if random.uniform(0,1) <= ratio: 
                n_nh = n_nh + 1; ET_nh.append(t_nh)
               
        t_h = -math.log(random.uniform(0,1))/self.background; n_h = 0     
        while t_h < T[-1]:
            n_h = n_h + 1; ET_h.append(t_h)
            t_h = t_h - math.log(random.uniform(0,1))/self.background
            
        Event_time = sorted(ET_nh + ET_h)
        #print ET_h, ET_nh
        return Event_time
         
    def detection_callback(self, data):
        """Does the detection calculations, plot the data and publish the data on a topic"""
	if self.source_available:
    	    a = time.time()
            if self.detection_counter == 0:
                self.start_time = data.header.stamp.to_sec()
                #self.fig, self.ax = plt.subplots(3)
    	        self.time = 0.0
    	    else:
    	        self.time = data.header.stamp.to_sec()-self.start_time
                
            self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])        
    
    
            self.sensor_position_vector.append(self.Pglobal)
    	    self.source_position_vector.append(self.source_location)
    	    dist2source = np.linalg.norm(self.Pglobal-self.source_location)
            mu = 1+((self.sensor_crosssection*self.source_intensity/self.background)/(2*self.sensor_crosssection+dist2source*dist2source))
            self.mu.append(mu)
            self.time_vector.append(self.time)
    	    self.dt_detection = 1.0/self.odometry_freq
    
    
           
            msg = DetectionData()
	    #print 'not in the loop'
	    #print 'detection counter:', self.detection_counter, self.odometry_freq/5.0, self.detection_counter%(self.odometry_freq/5.0)
	    #print 'len of vectors:', len(self.sensor_position_vector), len(self.sensor_pv_previous), len(self.source_position_vector), len(self.source_pv_previous), len(self.time_vector), len(self.time_vector_previous)
    
            if self.detection_counter%(self.odometry_freq/5.0) == 0.0:
		#print 'in the loop' 
    	        b = time.time()
                count_multiplication = 1.0
                vi_integration = 0.0
    	        #self.time_at_detection.append(self.time)
                #self.sensor_position_vector.append(self.Pglobal)
    	        #self.source_position_vector.append(self.source_location)
    	        #dist2source = np.linalg.norm(self.Pglobal-self.source_location)
                #mu = 1+((self.sensor_crosssection*self.source_intensity/self.background)/(2*self.sensor_crosssection+dist2source*dist2source))
                #self.mu.append(mu)
    	        #print '1', time.time()-b
                p, pfa = self.find_p(self.mu)
    	        #print '2', time.time()-b
                #self.time_vector.append(self.time)
    	    
    	        source_positions = self.source_position_vector[len(self.source_pv_previous):]
    	        sensor_positions = self.sensor_position_vector[len(self.sensor_pv_previous):]
    	        time_vector = self.time_vector[len(self.time_vector_previous):]
    
    	        #print 'length of vectors:', len(sensor_positions), len(source_positions), len(time_vector)
    
    	        tau = self.generate_counts(sensor_positions, source_positions, time_vector)
    
                #tau = self.generate_counts(self.sensor_position_vector, self.source_position_vector, self.time_vector)
    	        tau = [i+time_vector[0] for i in tau]	    
    	        self.tau.extend(tau)
    
                #for m in self.mu: 
                #    vi_integration = vi_integration + (1-m)*self.dt_detection*self.background
    
    	        vi_vector = [(1-m)*self.dt_detection*self.background for m in self.mu]
    	        vi_integration = sum(vi_vector)
    
                if p == 0: 
                    self.pmd_exponent = 0.0; self.detection_threshold = 0.0
                    self.pmd_vector.append(1.0)
                    self.detection_threshold_vector.append(0.0)
    		    LR = np.exp(vi_integration)
                    self.LR_vector.append(LR)		               
                    
                else:
    		    pmd_exponent_vector = [(mu**p+(1-p)*mu**p*np.log(mu)-mu)*self.background*self.dt_detection for mu in self.mu]
                    self.pmd_exponent = sum(pmd_exponent_vector)
    		    detection_threshold_vector = [(mu**p*np.log(mu)-mu+1)*self.background*self.dt_detection for mu in self.mu]
                    self.detection_threshold = np.exp(sum(detection_threshold_vector))
                    self.pmd_vector.append(np.exp(self.pmd_exponent))
                    self.detection_threshold_vector.append(self.detection_threshold)
    	            #print '5', time.time()-b
                    if len(self.tau) != 0:  
                        for t in self.tau: 
                            index = bisect.bisect_left(self.time_vector, t)
                            if index == 0: 
                                mu_at_tau = self.mu[0]
                            elif index > len(self.mu)-2: 
                                mu_at_tau = self.mu[-1]
                            else: 
                                mu_at_tau = (self.mu[index] + self.mu[index+1])/2.0
    
                            count_multiplication = count_multiplication*mu_at_tau
                    
                        LR = np.exp(vi_integration)*count_multiplication
                        self.LR_vector.append(LR)
    		        #print '6', time.time()-b
    		    else: 
    		        LR = np.exp(vi_integration)
                        self.LR_vector.append(LR)
    
    	        f = open(self.file, 'a')
    	        f.write("%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n" % (self.time_vector[-1], dist2source, mu, p, pfa, np.exp(self.pmd_exponent), len(self.tau), vi_integration, count_multiplication, LR, self.detection_threshold))
    
    
    
    	    
                msg.header.stamp = data.header.stamp
                #msg.QuadName = str(self.uav+self.no)
                msg.mu = mu
                msg.p = p
                msg.BackgroundRate = self.background
                msg.DetectorCrossSection = self.sensor_crosssection
                msg.DetectorTime = self.time
                msg.PmdExponent = self.pmd_exponent
                msg.DetectionThreshold = self.detection_threshold
                msg.LikelihoodRatio = self.LR_vector[-1]
                self.detection_data.publish(msg)
    
    	        self.source_pv_previous = self.source_position_vector[:]
    	        self.sensor_pv_previous = self.sensor_position_vector[:]
    	        self.time_vector_previous = self.time_vector[:]
    
    	        print 'total time in detection:', time.time()-a
                """
    	        self.ax[0].plot(self.time_vector, self.mu, '--', color='r', label="mu")
    	        self.ax[0].title.set_text('mu')
    
                self.ax[1].plot(self.time_vector, self.pmd_vector, '--', color='b', label="Pmd") 
    	        self.ax[1].title.set_text('Pmd')
               
                self.ax[2].plot(self.time_vector, self.LR_vector, '--', color='g', label="logrithmic_ratio")
                self.ax[2].plot(self.time_vector, self.detection_threshold_vector, '--', color='k', label="detection_threshold") 
                self.ax[2].title.set_text('Logrithmic Ratio Test')
    
                plt.xlabel('Time from Start (Sec)')
                plt.ylabel('Detection Performance Metric')
    	        plt.savefig('detection_parameters.png')
    
                plt.pause(0.00001)
    	        """
        
            self.detection_counter += 1
	


# if python says run, then we should run
if __name__ == '__main__':
    odometry_topic = rospy.get_param('odometry_topic')
    #goal = rospy.get_param('final_goal')
    freq = rospy.get_param('odometry_frequency')
    a = rospy.get_param('source_strength_cps')
    beta = rospy.get_param('background_cps')
    chi = rospy.get_param('sensor_cross_section')
    alpha = rospy.get_param('alpha')

    print 'odometry_topic', odometry_topic
    #print 'final_goal', final_goal
    print 'odometry_frequency', freq
    print 'source_strength_cps', a
    print 'background_cps', beta
    print 'sensor_cross_section', chi
    print 'alpha', alpha
        
    
    #r = rospy.Rate(30)    
    traj = detection_calculations(odometry_topic, freq, a, beta, chi, alpha)

    

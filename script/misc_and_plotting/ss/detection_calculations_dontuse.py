#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
#import time
import numpy as np
import time
import rospy, math, random, bisect
from nav_msgs.msg import Odometry
from quad_navigation_and_target_tracking.msg import DetectionData
from matplotlib import pyplot as plt


class detection_calculations(object): 
    def __init__(self, odometry_topic, goal, freq, a, beta, chi, detection_dt):
        """initialization"""
	rospy.init_node('detection_calculations', anonymous=True, log_level=rospy.DEBUG)
	self.path = '/home/ind/data_collection/TRO_dataset/detection/'
	open(self.path + 'detection_data.txt', 'w').close()

        # detetion parameters
        self.detection_counter = 0
        #self.uav = name
        self.odometry_topic = str(odometry_topic)
        self.source_intensity = a
        self.background = beta
        self.sensor_crosssection = chi
        self.alpha = 1e-1
        self.odometry_freq = freq
        self.dt_detection = detection_dt
        self.end_point = np.asarray(goal)
        # for now source is assumed at the ground level of the goal point, add another vector in launch file if different
        self.source_location = np.array([self.end_point[0], self.end_point[1], 2.0])
        self.mu = [] # its a list that keeps accumulating the values of the mu as per the current location, after p is available 
        self.pmd_vector = [] # list that stores the values of probability of missed detection 
        self.detection_threshold_vector = [] # list that stores the detection metric
        self.time_vector = []
        self.sensor_position_vector = []
        self.LR_vector= []
	self.LR_times = []
        self.vi_integration = 0
        self.count_multiplication = 1.0
	#self.start_time = time.time()
        
        self.detection_data = rospy.Publisher('/firefly1/detection_data', DetectionData, queue_size = 1, tcp_nodelay = True)
        try:
            rospy.Subscriber(self.odometry_topic, Odometry, self.detection_callback, tcp_nodelay = True)
            rospy.Subscriber('/jackal/ground_truth/state', Odometry, self.target_callback, tcp_nodelay = True)

        except:
            print('problem subscribing to one of the topic above')

        try:
            while not rospy.is_shutdown():             
                rospy.spin()
        except rospy.ROSInterruptException(): 
            pass


    def target_callback(self, data):

	self.source_location = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z+2.0]) 
	

    def find_p(self, _mu):
        pp = 0; self.got_p = False
        while pp < 1.0: 
            pfa_exp_list = [(j**pp - pp* j**pp * np.log(j) - 1)*self.background*self.dt_detection for j in _mu]
            #print np.exp(sum(pfa_exp_list))
            if np.exp(sum(pfa_exp_list)) < self.alpha: 
                self.got_p = True; #_p = pp
                break
            else: 
                pp = pp + 0.01
        if self.got_p == True: 
            return pp, np.exp(sum(pfa_exp_list))
        else: 
            print 'could not find a suitable value of parameter p for this set of mu'
            return 0,0
 
    def generate_counts(self, spos_, tpos_, T):
        """This function generates number of counts for each sensor based on its 
        position and the position of the moving source"""
        #print(spos_, tpos_)
        max_possible_nu = ((self.source_intensity*self.sensor_crosssection)/(2*self.sensor_crosssection + 0.25**2))
        vimax = max_possible_nu#self.source_intensity/2 
        #vimax = So*chi/(2*chi + ((spos_[0]-tpos_[0][0])**2 + (spos_[1]-tpos_[0][1])**2))
        t_nh = 0;  n_nh = 0 # initialize a number variable         
        ET_nh = []; ET_h = [] # initialise an array for storing event times
        
        while t_nh < T[-1]:  
            t_nh = t_nh - (math.log(random.uniform(0,1)) / vimax)
            index = bisect.bisect_left(T, t_nh)
            if index == 0: 
                pos_at_t_nh = spos_[0]
            elif index > len(T)-2: 
                pos_at_t_nh = spos_[-1]
            else: 
                pos_at_t_nh = (spos_[index] + spos_[index+1])/2.0
            #Pt = [tpos_[0][0] + Vt*t_nh, tpos_[0][1]]
            ratio = ((self.source_intensity*self.sensor_crosssection)/(2*self.sensor_crosssection + (np.linalg.norm(pos_at_t_nh-tpos_))**2))/vimax 
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
	a = time.time()
        if self.detection_counter == 0:
            self.start_time = data.header.stamp.to_sec()
            self.fig, self.ax = plt.subplots(3)

            
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])        
        self.time = data.header.stamp.to_sec()-self.start_time
        
	self.dt_detection = 0.01
        self.sensor_position_vector.append(self.Pglobal)
        mu = 1+((self.sensor_crosssection*self.source_intensity/self.background)/(2*self.sensor_crosssection+np.linalg.norm(self.Pglobal-self.source_location)**2))
	#print 'mu:', mu
        self.mu.append(mu)
        p, pfa = self.find_p(self.mu)
        self.time_vector.append(self.time)
        

        if p == 0: 
            self.pmd_exponent = 0.0; self.detection_threshold = 0.0
            self.pmd_vector.append(1.0)
            self.detection_threshold_vector.append(0.0)
        else:
            self.pmd_exponent = self.pmd_exponent + (mu**p+(1-p)*mu**p*np.log(mu)-mu)*self.background*self.dt_detection
            self.detection_threshold = self.detection_threshold + (mu**p*np.log(mu)-mu+1)*self.background*self.dt_detection
            self.pmd_vector.append(np.exp(self.pmd_exponent))
            self.detection_threshold_vector.append(self.detection_threshold)
        
        msg = DetectionData()

        if self.detection_counter%self.odometry_freq == 0.0: 
            count_multiplication = 1.0
            vi_integration = 0.0

            tau = self.generate_counts(self.sensor_position_vector, self.source_location, self.time_vector)
            for m in self.mu: 
                vi_integration = vi_integration + (1-m)*self.dt_detection*self.background
            print 'total number of counts received:', len(tau)
            if len(tau) != 0:  
                for t in tau: 
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
                self.LR_times.append(self.time)
            else: 
		LR = 1.0
                self.LR_vector.append(LR)
		self.LR_times.append(self.time)
            
	    f = open(self.path + 'detection_data.txt', 'a')
	    f.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (mu, p, pfa, np.exp(self.pmd_exponent), len(tau), vi_integration, count_multiplication, LR, self.detection_threshold))

            #print 'counts received are:', tau 
            #print 'lR and threshold are:', count_multiplication, vi_integration, self.LR_vector[-1], self.detection_threshold_vector[-1]
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
            
	    self.ax[0].plot(self.time_vector, self.mu, '--', color='r', label="mu")
	    self.ax[0].title.set_text('mu')

            self.ax[1].plot(self.time_vector, self.pmd_vector, '--', color='b', label="Pmd") 
	    self.ax[1].title.set_text('Pmd')
           
            self.ax[2].plot(self.LR_times, self.LR_vector, '--', color='g', label="logrithmic_ratio")
            self.ax[2].plot(self.time_vector, self.detection_threshold_vector, '--', color='k', label="detection_threshold") 
            self.ax[2].title.set_text('Logrithmic Ratio Test')

            #self.fig.suptitle('Detection Parameters', fontsize=16)
            plt.xlabel('Time from Start (Sec)')
            plt.ylabel('Detection Performance Metric')
            #if self.detection_counter == 0: 
            #    plt.legend(ncol=1, loc=1)
	    #plt.savefig('detection_parameters.png')
	    print 'time taken in detection calculations:', time.time()-a
            plt.pause(0.00001)
        
        self.detection_counter += 1


# if python says run, then we should run
if __name__ == '__main__':
    odometry_topic = rospy.get_param('odometry_topic')
    #name = rospy.get_param('uav_name')
    #number = rospy.get_param('number')
    goal = rospy.get_param('final_goal')
    freq = rospy.get_param('odometry_frequency')
    a = rospy.get_param('source_strength_cps')
    beta = rospy.get_param('background_cps')
    chi = rospy.get_param('sensor_cross_section')
    detection_dt = rospy.get_param('detection_dt')
        
    
    #r = rospy.Rate(30)    
    traj = detection_calculations(odometry_topic, goal, freq, a, beta, chi, detection_dt)

    

#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import time 

class counts_vs_relative_distance(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, run_number, odometry_topic, target_topic, count_topic):
	rospy.init_node('counts_vs_relative_distance', anonymous=False, log_level=rospy.DEBUG)
	self.no = str(run_number)
        self.counts_available = False
        self.jackalpose_available = False  
        self.logpath = '/home/pelican/data_collection/TRO/'
        open(self.logpath+'counts_vs_relative_distance'+self.no+'.txt', 'w').close()

        try:
            rospy.Subscriber(count_topic, PointStamped, self.countsCb, tcp_nodelay = True)
	    rospy.Subscriber(target_topic, Odometry, self.jackalCb, tcp_nodelay = True)
	    rospy.Subscriber(odometry_topic, Odometry, self.quadCb, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the odometry topic of state estimator')

        try: 
            while not rospy.is_shutdown():             
                rospy.spin()
        except rospy.ROSInterruptException(): 
            pass

    def quadCb(self, data):
        time = data.header.stamp.to_sec()
	self.quad_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

	if self.counts_available and self.jackalpose_available:   
	    rel_dist = self.quad_pose - self.jackal_pose
            f = open(self.logpath+'counts_vs_relative_distance'+self.no+'.txt', 'a')
            f.write('%s, %s, %s, %s, %s, %s\n' % (time, np.linalg.norm(rel_dist), self.counts, rel_dist[0], rel_dist[1], rel_dist[2]))  

    def jackalCb(self, data):
	self.jackal_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
 	self.jackalpose_available = True

    def countsCb(self, data):
	self.counts = data.point.x
	self.counts_available = True
          

if __name__ == '__main__':
    run_number = rospy.get_param('run_number')
    odometry_topic = rospy.get_param('odometry_topic')
    target_topic = rospy.get_param('target_topic')
    count_topic = rospy.get_param('count_topic')
    calc = counts_vs_relative_distance(run_number, odometry_topic, target_topic, count_topic)

                


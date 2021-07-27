#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy
#from isy_geometric_controller.msg import Desired_Trajectory
#from isy_geometric_controller.msg import modifiedodometry
from nav_msgs.msg import Odometry
import time 
from pyquaternion import Quaternion

class record_odometry(object): 
    
    def __init__(self):
	self.counter = 0
	self.path = '/home/ind/data_collection/TRO_dataset/trajectories/'
	open(self.path + "quad_jack_pos.txt", "w").close()
	open(self.path + "quad_jack_vel.txt", "w").close()

	
        try:
	    quad = rospy.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, self.quadcallback, queue_size = 10, tcp_nodelay = True)
            jack = rospy.Subscriber('/jackal/ground_truth/state', Odometry, self.jackcallback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the odometry topic')


    def quadcallback(self, q):
	self.xq = np.array([q.pose.pose.position.x, q.pose.pose.position.y, q.pose.pose.position.z])
	self.vq = np.array([q.twist.twist.linear.x, q.twist.twist.linear.y, q.twist.twist.linear.z])		
 
    def jackcallback(self, j):

        t = j.header.stamp

	x = np.array([j.pose.pose.position.x, j.pose.pose.position.y, j.pose.pose.position.z]) # position
	vx = np.array([j.twist.twist.linear.x, j.twist.twist.linear.y, j.twist.twist.linear.z]) # velocity
	

	f2 = open(self.path + 'quad_jack_pos.txt', 'a')
	f2.write("%s, %s, %s, %s, %s, %s, %s\n" % (t, self.xq[0], self.xq[1], self.xq[2], x[0], x[1], x[2]))

	f3 = open(self.path + 'quad_jack_vel.txt', 'a')
	f3.write("%s, %s, %s, %s, %s, %s, %s\n" % (t, self.vq[0], self.vq[1], self.vq[2], vx[0], vx[1], vx[2]))



if __name__ == '__main__':
    rospy.init_node('record_odometry', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = record_odometry()
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                


#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time 
import message_filters
from pyquaternion import Quaternion

class visionpose_localpose(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self):
	self.vp_available = False
        self.od_available = False
	self.path = '/home/pelican/data_collection/'
	open(self.path + "ov_mv_odom_comparison.txt", "w").close()
        self.tvp = 0

        try:
	    
            rospy.Subscriber('/udrone1/mavros/odometry/in', Odometry, self.mvCb, tcp_nodelay = True)
            rospy.Subscriber('/udrone1/final_odometry', Odometry, self.ovCb, tcp_nodelay = True)
 
        except:
            print('problem subscribing to one of the odometry topic')


    def mvCb(self, od):
        tod = od.header.stamp
	xod = np.array([od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z])
        vod = np.array([od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z])
	qod = Quaternion(od.pose.pose.orientation.w, od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z)
        if self.od_available == True: 
	    f3 = open(self.path + 'ov_mv_odom_comparison.txt', 'a')
	    f3.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (xod[0], self.xod[0], xod[1], self.xod[1], xod[2], self.xod[2], qod[0], self.qod[0], qod[1], self.qod[1], qod[2], self.qod[2], qod[3], self.qod[3], vod[0], self.vod[0], vod[1], self.vod[1], vod[2], self.vod[2]))	    



    def ovCb(self, od):
        self.tod = od.header.stamp
	self.xod = np.array([od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z])
        self.vod = np.array([od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z])
	self.qod = Quaternion(od.pose.pose.orientation.w, od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z)
        self.od_available = True	

if __name__ == '__main__':
    rospy.init_node('pose_comparison', anonymous=False, log_level=rospy.DEBUG)
    traj = visionpose_localpose()
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                


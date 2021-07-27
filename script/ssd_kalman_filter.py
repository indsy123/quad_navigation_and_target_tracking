#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 09:59:56 2020
first play with RANSAC in python 
@author: ind
"""
import cv2, time
import numpy as np
import ros_numpy
import rospy
import message_filters
from matplotlib import pyplot as plt
from quad_navigation_and_target_tracking.msg import publish_bounding_boxes
from sensor_msgs.msg import Image
import std_msgs.msg
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from pyquaternion import Quaternion


class ssd_kalman_filter(object):
    def __init__(self, run_number): 
        """initializes various paramters"""
        self.logpath = '/home/pelican/data_collection/TRO/'
        self.no = run_number 
        #open(self.logpath+'positions_from_NN_tracking.txt', 'w').close()
        #open(self.logpath+'positions_after_KF.txt', 'w').close()
        open(self.logpath+'positions_wrt_cog'+self.no+'.txt', 'w').close()
	open(self.logpath+'positions_wrt_world'+self.no+'.txt', 'w').close()
	open(self.logpath+'velocities_wrt_world'+self.no+'.txt', 'w').close()
	#open(self.logpath+'velocities_wrt_world2.txt', 'w').close()
	#open(self.logpath+'velocities_after_KF.txt', 'w').close()

    
        self.tracking_counter = 0
	self.matching_counter = 0
        #self.box_update_counter = 1
        self.estimates_available = False
        self.gotbox = False
	self.got_quad_position = False
        #self.bridge = CvBridge()
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create(contrastThreshold = 0.02, sigma = 1.0)
        #self.orb = cv2.ORB_create(nfeatures = 1000, scaleFactor = 1.5, scoreType=cv2.ORB_FAST_SCORE)
        #self.orb = cv2.ORB_create(nfeatures = 1000)
        
        # FLANN parameters
        #FLANN_INDEX_LSH = 6
        #index_params = dict(algorithm = FLANN_INDEX_LSH, table_number = 8, key_size = 20, multi_probe_level = 2)
        #the lower the table number, the faster the speed, key_size is the same but not getting very good results
        # multiprobe_lavel = 1 disable multiprobing
        #search_params = dict(checks=50)   # or pass empty dictionary
        
        # for sift, but I find it very slow
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)   
        
        self.flann = cv2.FlannBasedMatcher(index_params,search_params)        
        
        
        
        #self.tracker = cv2.TrackerKCF_create()
        self.min_match_count = 5
        
        self.intrinsics = np.array([599.6348915124814, 598.2834935949271, 316.66192693644587, 234.737560066737]) # my calibration 
        self.distortion_coeff = np.array([0.111864, -0.212956, 0.00042, -0.0053118]) # my calibration 


	self.cRcog = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0],[1.0, 0.0, 0.0]]) # 0 degree
	#self.cRcog = np.array([[0.0, -1.0, 0.0], [-0.0523360, 0.0, -0.9986295],[0.9986295, 0.0, -0.0523360]]) # 3 degree
        #self.cRcog = np.array([[0.0, -1.0, 0.0], [-0.1218693, 0.0, -0.9925461],[0.9925461, 0.0, -0.1218693]]) # 7 degree
        self.cPcog = np.array([[0.12], [0], [0.01]])
        
        self.scale = 0.0010000000474974513
        
        #------------- kalman filter parameters for constant acceleration target model----------------------
        dt = 1.0/15 # how fast you can track
        self.a = np.array([[1, 0, 0, dt, 0, 0, 0.5*dt**2, 0, 0], 
                           [0, 1, 0, 0, dt, 0, 0, 0.5*dt**2, 0],
                           [0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt**2], 
                           [0, 0, 0, 1, 0, 0, dt, 0, 0], 
                           [0, 0, 0, 0, 1, 0, 0, dt, 0], 
                           [0, 0, 0, 0, 0, 1, 0, 0, dt],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1]])
     
        self.h  = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 1, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0, 0], # tried 9x9
                            [0, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0]]) 
	self.dt = dt
        # process noise covariance
        self.q = np.identity(9)
        self.q[:3] = self.q[:3]*1e-3; self.q[-6:] = self.q[-6:] * 1e-3
        # measurement noise covariance       
        self.r = np.identity(9) 
        self.r[:3] = self.r[:3]*2e-2; self.r[3:] = self.r[3:] * 2e-2        
        self.P = np.identity(9)*1e-2 
        self.i = np.identity(9)        
        #------------- kalman filter parameters for constant acceleration target model----------------------

    
        
        
        self.coord_pub = rospy.Publisher('/udrone1/target_position', Odometry, queue_size = 1, tcp_nodelay=True)
        rospy.Subscriber('/udrone1/detection_boxes', publish_bounding_boxes, self.get_boxes, tcp_nodelay=True)
        rospy.Subscriber('/udrone1/mavros/odometry/in', Odometry, self.quad_odometry, tcp_nodelay=True)
        try:              
            color = message_filters.Subscriber('/udrone1/d435/color/image_raw', Image, tcp_nodelay=True)
            depth = message_filters.Subscriber('/udrone1/d435/aligned_depth_to_color/image_raw', Image, tcp_nodelay=True)
            ts = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 0.005)
            ts.registerCallback(self.target_tracking)
            #rospy.spin()
        except: 
            print('problem subscribing to camera image topic')

        try: 
            while not rospy.is_shutdown():
                rospy.spin()
        except rospy.ROSInterruptException(): 
            pass

    def quad_odometry(self,odom): 
        """get the current position of the quad in the world frame"""
        
        self.quad_position = np.array([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        self.quadR = q.rotation_matrix
        self.quad_linvel = np.array([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
	w = np.array([odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z])
	self.quad_w_hat = np.array([[0, -w[2], w[1]],[w[2], 0, -w[0]], [-w[1], w[0], 0]]) 
	self.got_quad_position = True


    def get_boxes(self,d): 
        """get all the bounding boxes from the NN, currently only stores the box 
        with max score, assuming this is the quad, CHECK IF THIS IS TRUE"""
        self.bbox = map(lambda i: [d.coordinates[i].score, int(d.coordinates[i].x), int(d.coordinates[i].y), \
                                   int(d.coordinates[i].w), int(d.coordinates[i].h)], range(d.no_of_boxes))
        
        self.update_box = True
        self.gotbox = True
        
        
    def kalman_filter(self, meas, run_update): 
        """assuming the x, y, z position of the box center as measurement, this
        function runs a kalman filter to track the box. The measurement for 
        x, y and z are the mean coordinates of the pixels"""
        A = self.a; H = self.h; R = self.r; Q = self.q; I = self.i
        if run_update == True:             
            Zk = meas
            Xk_ = np.dot(A, self.X)
            Pk_ = np.dot(A, np.dot(self.P, A.T)) + Q
            intermetiate = np.dot(H, np.dot(Pk_, H.T)) + R
            Kk = np.dot(np.dot(Pk_, H.T), intermetiate)
            self.X = Xk_ + np.dot(Kk, (Zk - np.dot(H, Xk_))) 
            self.P = np.dot(I - np.dot(Kk, H), Pk_)
        else: 
            #Zk = self.X; R = np.identity(3)*0.5 # a high measurement noise with last position selected as measurement
            Xk_ = np.dot(A, self.X)
            Pk_ = np.dot(A, np.dot(self.P, A.T)) + Q
            #intermetiate = np.dot(H, np.dot(Pk_, H.T)) + R
            #Kk = np.dot(np.dot(Pk_, H.T), intermetiate)
            self.X = Xk_ #+ np.dot(Kk, (Zk - np.dot(H, Xk_))) 
            self.P = Pk_#np.dot(I - np.dot(Kk, H), Pk_)
            
        self.estimates_available = True
        return self.X, self.P
        
    def apply_ransac(self, img1, img2, depth_array, x, y, w, h): 
        """function applies the ransac between every two images"""   

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(img1,None)
        kp2, des2 = self.sift.detectAndCompute(img2,None)

        #des1 = np.float32(des1)
 	#des2 = np.float32(des2)
	#des2.convertTo(des2, CV_32F)
	#des2.convertTo(des2, CV_32F) 
               
        matches = self.flann.knnMatch(des1,des2,k=2)
        # ratio test
        good = []        
        
        for i, m_n in enumerate(matches):
            if len(m_n) != 2: 
                continue
            (m,n) = m_n
            if m.distance < 0.75*n.distance:
                good.append(m)
        #print('total length of good matches is:', len(good))

        if len(good) > self.min_match_count:

            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            
            inliers = []; outliers = []
            for i in range(len(good)): 
                if matchesMask[i] != 0: 
                    img1_idx = good[i].queryIdx
                    (x1,y1) = kp1[img1_idx].pt
                    inliers.append([int(x1+self.final_box[0]), int(y1+self.final_box[1])])
                else: 
                    img1_idx = good[i].queryIdx
                    (x1,y1) = kp1[img1_idx].pt
                    outliers.append([int(x1+self.final_box[0]), int(y1+self.final_box[1])])

            ci = self.intrinsics 
            xx = []; yy = []; zz = []
            for i in inliers: 
                zz.append(depth_array[i[1], i[0]]*self.scale)
                xx.append(zz[-1]*(i[0] - ci[2])/ci[0])
                yy.append(zz[-1]*(i[1] - ci[3])/ci[1])


            xmean = np.mean(xx); ymean = np.mean(yy); zmean = np.mean(zz)
            #"""
            #h,w,d = img1.shape
            #pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
            #dst = cv2.perspectiveTransform(pts,M)
        
            #img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
	    #cv2.imshow('matched_image0', img2)
            #draw_params = dict(matchColor = (0,255,0), # draw matches in green color
            #                   singlePointColor = None,
            #                   matchesMask = matchesMask, # draw only inliers
            #                   flags = 2)

            #img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
            #cv2.imshow('matched_image', img3)
            #cv2.waitKey(1)
            #"""
            return np.array([[xmean], [ymean], [zmean], [0], [0], [0], [0], [0], [0]]), True 
        
        else:
            print("Not enough matches are found - %d/%d" % (len(good),self.min_match_count))
            matchesMask = None
            return np.array([[False], [False], [False], [False], [False], [False], [False], [False], [False]]), False

    def target_tracking(self, color, depth): 
        """apply ransac on two images"""
        a = time.time()
        if  self.gotbox and self.got_quad_position:
            if self.tracking_counter == 0: 
                self.previous_image = ros_numpy.numpify(color)
            else: 
                self.current_image = ros_numpy.numpify(color)
                depth_array = ros_numpy.numpify(depth)
    
                # now get the bounding boxes and either pick the one with max score or closest to 
                # the existing target, at first time step, no option but to pick the one with the best score 

                # NOTE that boxes are published sorted wrt scores
                if len(self.bbox) == 1: 
                    # if no other box, pick the one you have and hope that it is correct
                    self.final_box = tuple(self.bbox[0][1:]) 
                elif len(self.bbox) > 1 and self.estimates_available == False: 
                    # if no estimates are available you cant compare, so pick the box with max score and hope it is correct
                    self.final_box = tuple(self.bbox[0][1:])
                elif len(self.bbox) > 1 and self.estimates_available == True:  
                    # you have kalman filter estimates to compare to 
                    ci = self.intrinsics 
                    uv_of_box_centers = map(lambda j: [int(j[1]+j[3]/2.0), int(j[2]+j[4]/2.0)], self.bbox)

                    z_boxes = map(lambda i: depth_array[i[1], i[0]]*self.scale, uv_of_box_centers)
                    x_boxes = map(lambda i: z_boxes[i]*(uv_of_box_centers[i][0] - ci[2])/ci[0], range(len(uv_of_box_centers)))   
                    y_boxes = map(lambda i: z_boxes[i]*(uv_of_box_centers[i][1] - ci[3])/ci[1], range(len(uv_of_box_centers)))
                    xyz  = zip(x_boxes, y_boxes, z_boxes)
                    #print(xyz, self.X[:3])
                    cov = np.sqrt(self.P[0][0]**2 + self.P[1][1]**2 + self.P[2][2]**2)
                    dist = np.linalg.norm(xyz-self.X[:3].ravel(), axis = 1)
                    tolerance = max(0.15, cov)
                    if min(dist) <= tolerance: 
                        # if the bounding box is within the tolerance
                        index, value = min(enumerate(dist), key=lambda x: abs(x[1]-tolerance))
                        self.final_box = tuple(self.bbox[index][1:])

    
                x = int(self.final_box[0]); y = int(self.final_box[1]); w = int(self.final_box[2]); h = int(self.final_box[3])
                
                x = x if x%2 == 0 else x-1
                y = y if y%2 == 0 else y-1
                w = w if w%2 == 0 else w-1
                h = h if h%2 == 0 else h-1
                
                #p1 = (x, y);  p2 = (x+w, y+h)
                #ci = self.intrinsics # of the form [fx, fy, cx, cy], assuming no distortion  
                img1 = self.previous_image[y:y+h, x:x+w]
                img2 = self.current_image[y:y+h, x:x+w]
                #img1 = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
                #img2 = cv2.normalize(img2, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        

                try: 
                    match_found = False
                    self.measurement, match_found = self.apply_ransac(img1, img2, depth_array, x, y, w, h)
                    self.previous_image = self.current_image
                except: 
                    if match_found == False:     
                        print('match with the previous image is not found, the kalman filter will not impliment update')
                        #xmean, ymean, zmean = self.get_cog_of_tracked_box(depth_array, x, y, w, h)
                        self.previous_image = self.current_image

                # the measurement for kalman filter is the 3D location of tracked box
                #if self.tracking_counter == 1: 
                #    self.X = np.array([[self.measurement[0][0]], [self.measurement[1][0]], [self.measurement[2][0]], [0], [0], [0], [0], [0], [0]]) 
                    
                if match_found == True:
		    if self.matching_counter == 0: 
                        self.X = np.array([[self.measurement[0][0]], [self.measurement[1][0]], [self.measurement[2][0]], [0], [0], [0], [0], [0], [0]])

                    state, cov = self.kalman_filter(self.measurement, match_found)
                    position_wrt_cog = self.cPcog + np.dot(self.cRcog.T, state[0:3])
		    velocity_wrt_cog = np.dot(self.cRcog.T, state[3:6])
		    position_wrt_world = self.quad_position + np.dot(self.quadR, position_wrt_cog)
		    velocity_wrt_world2 = self.quad_linvel + np.dot(self.quadR, velocity_wrt_cog) + np.dot(np.dot(self.quadR, self.quad_w_hat), position_wrt_cog)


		    if self.matching_counter == 0: 
                        velocity_wrt_world = np.array([[0], [0], [0]])
		    else:
			velocity_wrt_world = (position_wrt_world - self.previous_postion)/self.dt
			

		    
		    self.previous_postion = position_wrt_world
                    """
                    f1 = open(self.logpath+'positions_from_NN_tracking.txt', 'a')
                    f1.write('%s, %s, %s\n' % (self.measurement[0][0], self.measurement[1][0], self.measurement[2][0]))
        
                    f2 = open(self.logpath+'positions_after_KF.txt', 'a')
                    f2.write('%s, %s, %s\n' % (state[0][0], state[1][0], state[2][0]))

                    f22 = open(self.logpath+'velocities_after_KF.txt', 'a')
                    f22.write('%s, %s, %s\n' % (state[3][0], state[4][0], state[5][0]))
             	    """
                    f3 = open(self.logpath+'positions_wrt_cog'+self.no+'.txt', 'a')
                    f3.write('%s, %s, %s, %s\n' % (time.time(), position_wrt_cog[0][0], position_wrt_cog[1][0], position_wrt_cog[2][0]))
		    
                    f4 = open(self.logpath+'positions_wrt_world'+self.no+'.txt', 'a')
                    f4.write('%s, %s, %s, %s\n' % (time.time(), position_wrt_world[0][0], position_wrt_world[1][0], position_wrt_world[2][0]))
                    
                    f5 = open(self.logpath+'velocities_wrt_world'+self.no+'.txt', 'a')
                    f5.write('%s, %s, %s, %s\n' % (time.time(), velocity_wrt_world[0][0], velocity_wrt_world[1][0], velocity_wrt_world[2][0]))
		    """
                    f5 = open(self.logpath+'velocities_wrt_world2.txt', 'a')
                    f5.write('%s, %s, %s\n' % (velocity_wrt_world2[0][0], velocity_wrt_world2[1][0], velocity_wrt_world2[2][0]))
		    """
                    msg = Odometry()
                    msg.header.stamp = color.header.stamp
                    msg.header.frame_id = 'world'
                    msg.pose.pose.position.x = position_wrt_world[0][0]; msg.pose.pose.position.y = position_wrt_world[1][0]; msg.pose.pose.position.z = position_wrt_world[2][0]
                    msg.pose.pose.orientation.x = 0.0; msg.pose.pose.orientation.y = 0.0; msg.pose.pose.orientation.z = 0.0
                    msg.pose.pose.orientation.w = 1.0 # a unit orientation for now.   
                    msg.twist.twist.linear.x = velocity_wrt_world[0][0]; msg.twist.twist.linear.y = velocity_wrt_world[1][0]; msg.twist.twist.linear.z = velocity_wrt_world[2][0]
                    self.coord_pub.publish(msg)
		    self.matching_counter += 1
            
            #print 'total execution time for ransac script is:', time.time()-a
            
            self.tracking_counter += 1
if __name__ == '__main__':
    rospy.init_node('ssd_kalman_filter', anonymous=False, log_level=rospy.DEBUG)
    run_number = rospy.get_param('run_number')
    solve = ssd_kalman_filter(run_number)



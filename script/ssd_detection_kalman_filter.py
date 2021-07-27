#!/usr/bin/env python
"""
original script created by Michael. The script detects a ground robot, puts  
a bounding box around it and also returns detection confidence. 
It is under active modification to convert it into a ros node and enhance the 
speed to get a real time performance. The aim is to track a quad from the 
other quad. 
Modifications in R0: changed to class structure
Modification in R1: change to take data from a ros bag

author- Michael Seebok
Modified by- Indrajeet Yadav
"""
#import sys
import os
import cv2, time
import numpy as np
import tensorflow as tf
import ros_numpy
import rospy
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt
from object_detection.utils import label_map_util
from utils import visualization_utils as vis_util
#from quad_navigation_and_target_tracking.msg import publish_bounding_boxes, box_coordinates
import message_filters
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
#from tensorflow.python.compiler.tensorrt import trt_convert as trt
#from tensorflow.python.compiler.tensorrt import trt_convert as trt
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class ssd_detection_kalman_filter(object):
    def __init__(self, ckpt_path, labels_path, n_classes, run_number): 
        """initializes various paramters"""
        #---------------------------ssd mobilenet detection------------------------------------------------------------
        rospy.init_node('detection_using_ssd_mobilenet', anonymous=False, log_level=rospy.DEBUG)
        self.start = time.time()

        self.PATH_TO_CKPT = ckpt_path
        self.PATH_TO_LABELS = labels_path
        self.NUM_CLASSES = n_classes
        output_names = ['detection_boxes', 'detection_scores', 'detection_classes', 'num_detections']
        #input_names = ['image_tensor'] 
        tf.logging.set_verbosity(tf.compat.v1.logging.ERROR) 
        #self.bridge = CvBridge()

        self.tracking_counter = 0
        self.detection_counter = 0
        #self.update_box = False
        self.tt = time.time()

        tf_config = tf.ConfigProto()
        tf_config.gpu_options.allow_growth = True
        tf_config.allow_soft_placement = False
        tf_config.gpu_options.per_process_gpu_memory_fraction = 0.6
        self.session = tf.Session(config=tf_config)
            
        with tf.io.gfile.GFile(PATH_TO_CKPT, "rb") as f:
            trt_graph = tf.GraphDef()
            trt_graph.ParseFromString(f.read())

        tf.import_graph_def(trt_graph, name='', return_elements = output_names)
        self.detection_graph = trt_graph



        label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        
        image_np = np.zeros((480,640,3))
        image_np_expanded = np.expand_dims(image_np, axis=0)

        image_tensor = self.session.graph.get_tensor_by_name('image_tensor:0')
        boxes = self.session.graph.get_tensor_by_name('detection_boxes:0')
        scores = self.session.graph.get_tensor_by_name('detection_scores:0')
        classes = self.session.graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.session.graph.get_tensor_by_name('num_detections:0')        
        
        (boxes, scores, classes, num_detections) = self.session.run([boxes, scores, classes, num_detections], feed_dict={image_tensor: image_np_expanded})        
        
        #---------------------------ssd mobilenet detection------------------------------------------------------------  
        #---------------------------kalman filter----------------------------------------------------------------------
	self.no = str(run_number)  
        self.logpath = '/home/pelican/data_collection/TRO/ssd_mobilenet/'
        #open(self.logpath+'positions_from_NN_tracking.txt', 'w').close()
        #open(self.logpath+'positions_after_KF.txt', 'w').close()
        open(self.logpath+'positions_wrt_cog'+self.no+'.txt', 'w').close()
	open(self.logpath+'positions_wrt_world'+self.no+'.txt', 'w').close()
	open(self.logpath+'velocities_wrt_world'+self.no+'.txt', 'w').close()
    
        self.tracking_counter = 0
        self.matching_counter = 0
        #self.box_update_counter = 1
        self.estimates_available = False
        self.gotbox = False
        self.got_quad_position = False
        #self.bridge = CvBridge()
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create(contrastThreshold = 0.04, sigma = 1.0)
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
        #self.distortion_coeff = np.array([0.111864, -0.212956, 0.00042, -0.0053118]) # my calibration 


        #self.cRcog = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0],[1.0, 0.0, 0.0]]) # 0 degree
        #self.cRcog = np.array([[0.0, -1.0, 0.0], [-0.0523360, 0.0, -0.9986295],[0.9986295, 0.0, -0.0523360]]) # 3 degree
        self.cRcog = np.array([[0.0, -1.0, 0.0], [-0.1305262, 0.0, -0.9914449],[0.9914449, 0.0, -0.1305262]]) # 7 degree
        self.cPcog = np.array([[0.12], [0.02], [0.01]])        
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
        self.r[:3] = self.r[:3]*2e-3; self.r[3:] = self.r[3:] * 2e-3        
        self.P = np.identity(9)*1e-2 
        self.i = np.identity(9)        
        #------------- kalman filter parameters for constant acceleration target model----------------------
        #---------------------------kalman filter---------------------------------------------------------------------- 


        self.coord_pub = rospy.Publisher('/udrone1/target_position', Odometry, queue_size = 1, tcp_nodelay=True)
        #rospy.Subscriber('/udrone1/detection_boxes', publish_bounding_boxes, self.get_boxes, tcp_nodelay=True)
        rospy.Subscriber('/udrone1/mavros/odometry/in', Odometry, self.quad_odometry, tcp_nodelay=True)
        try:              
            color = message_filters.Subscriber('/udrone1/d435/color/image_raw', Image, tcp_nodelay=True)
            depth = message_filters.Subscriber('/udrone1/d435/aligned_depth_to_color/image_raw', Image, tcp_nodelay=True)
            ts = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 0.01)
            ts.registerCallback(self.target_tracking)
            #rospy.spin()
        except: 
            print 'problem subscribing to camera image topic' 

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
               
        matches = self.flann.knnMatch(des1,des2,k=2)
        # ratio test
        good = []        
        for i, m_n in enumerate(matches):
            if len(m_n) != 2: 
                continue
            (m,n) = m_n
            if m.distance < 0.75*n.distance:
                good.append(m)


        if len(good) > self.min_match_count:

            src_pts = np.float32([kp1[p.queryIdx].pt for p in good]).reshape(-1,1,2)
            dst_pts = np.float32([kp2[p.trainIdx].pt for p in good]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            inliers = []; outliers = []
            for i in range(len(good)): 
                if matchesMask[i] != 0: 
                    img1_idx = good[i].queryIdx
                    (x1,y1) = kp1[img1_idx].pt
                    inliers.append([int(x1+x), int(y1+y)])
                else: 
                    img1_idx = good[i].queryIdx
                    (x1,y1) = kp1[img1_idx].pt
                    outliers.append([int(x1+x), int(y1+y)])

            ci = self.intrinsics 
            xx = []; yy = []; zz = []
            ci0 = 1.0/ci[0]; ci1 = 1.0/ci[1]
            for i in inliers: 
                zz.append(depth_array[i[1], i[0]]*self.scale)
                xx.append(zz[-1]*(i[0] - ci[2])*ci0)
                yy.append(zz[-1]*(i[1] - ci[3])*ci1)

            xmean = np.mean(xx); ymean = np.mean(yy); zmean = np.mean(zz)
            """
            h,w,d = img1.shape
            pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
        
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

            draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                               singlePointColor = None,
                               matchesMask = matchesMask, # draw only inliers
                               flags = 2)

            img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
            cv2.imshow('matched_image', img3)
            cv2.waitKey(1)
	    """
            print("Number of good matches and min match count - %d/%d" % (len(good),self.min_match_count))
            return np.array([[xmean], [ymean], [zmean], [0], [0], [0], [0], [0], [0]]), True 
        
        else:
            print("Not enough matches are found - %d/%d" % (len(good),self.min_match_count))
            matchesMask = None
            return np.array([[False], [False], [False], [False], [False], [False], [False], [False], [False]]), False

            
    def detect_alert(self, boxes, classes, scores, category_index, max_boxes_to_draw=20, min_score_thresh=0.5):
        r = []
        for i in range(min(max_boxes_to_draw, boxes.shape[0])):
            if scores[i] is None or scores[i] > min_score_thresh:
                test1 = None
                test2 = None
    
                if category_index[classes[i]]['name']:
                    test1 = category_index[classes[i]]['name']
                    test2 = int(100 * scores[i])
    
                line = {}
                line[test1] = test2
                r.append(line)
    
        return r
    
           

    def target_tracking(self, color, depth): 
        start  = time.time()
	self.current_image =  ros_numpy.numpify(color)

        #scale_percent = 60
	#width = int(640*scale_percent/100)
        #height = int(480*scale_percent/100)
	#dim = (width, height)
        #resized = cv2.resize(self.current_image, dim, interpolation = cv2.INTER_AREA)
	#self.current_image = ros_numpy.numpify(resized)
           

        if self.got_quad_position:
            #msg = publish_bounding_boxes()
            image_np_expanded = np.expand_dims(self.current_image, axis=0)        
            image_tensor = self.session.graph.get_tensor_by_name('image_tensor:0')
            boxes = self.session.graph.get_tensor_by_name('detection_boxes:0')
            scores = self.session.graph.get_tensor_by_name('detection_scores:0')
            classes = self.session.graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.session.graph.get_tensor_by_name('num_detections:0')
            (boxes, scores, classes, num_detections) = self.session.run([boxes, scores, classes, num_detections], feed_dict={image_tensor: image_np_expanded})
            alert_array = self.detect_alert(np.squeeze(boxes), np.squeeze(classes).astype(np.int32), np.squeeze(scores), self.category_index)  

            if len(alert_array) != 0: 
                bb = boxes[0][0]
                x = int(bb[1]*640)
                y = int(bb[0]*480)
                w = int((bb[3] - bb[1])*640)
                h = int((bb[2] - bb[0])*480)
                # make these values even
                x = x if x%2 == 0 else x-1
                y = y if y%2 == 0 else y-1
                w = w if w%2 == 0 else w-1
                h = h if h%2 == 0 else h-1
                self.gotbox = True
                
       		"""
                if alert_array[0]['jackal'] > 50: 
                    vis_util.visualize_boxes_and_labels_on_image_array(
                    self.current_image,
                    np.squeeze(boxes[0]),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    self.category_index,
                    max_boxes_to_draw=1,
                    min_score_thresh=.5,
                    use_normalized_coordinates=True,
                    line_thickness=4)
                    cv2.imshow("tagged_image", self.current_image)
                    cv2.waitKey(1)
                else:
                    print('detection confidence is below 50%')
                """
                if self.tracking_counter == 0: 
                    self.previous_image = self.current_image
                    self.X = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0]])
                else: 
                    #self.current_image = ros_numpy.numpify(color)
                    depth_array = ros_numpy.numpify(depth)
                    img1 = self.previous_image[y:y+h, x:x+w]
                    img2 = self.current_image[y:y+h, x:x+w]
                    
                    try: 
                        match_found = False
                        self.measurement, match_found = self.apply_ransac(img1, img2, depth_array, x, y, w, h)
                        self.previous_image = self.current_image
                    except: 
                        if match_found == False:     
                            print 'match with the previous image is not found, the kalman filter will not impliment update'
                            #xmean, ymean, zmean = self.get_cog_of_tracked_box(depth_array, x, y, w, h)
                            self.previous_image = self.current_image
                            
                    #if match_found == True:
                  
                        
                    state, cov = self.kalman_filter(self.measurement, match_found)
                    position_wrt_cog = self.cPcog + np.dot(self.cRcog.T, state[0:3])
                    velocity_wrt_cog = np.dot(self.cRcog.T, state[3:6])
                    position_wrt_world = self.quad_position + np.dot(self.quadR, position_wrt_cog)
                    velocity_wrt_world = self.quad_linvel + np.dot(self.quadR, velocity_wrt_cog) + np.dot(np.dot(self.quadR, self.quad_w_hat), position_wrt_cog)
    
               
                    f3 = open(self.logpath+'positions_wrt_cog'+self.no+'.txt', 'a')
                    f3.write('%s, %s, %s\n' % (position_wrt_cog[0][0], position_wrt_cog[1][0], position_wrt_cog[2][0]))
    
                    f4 = open(self.logpath+'positions_wrt_world'+self.no+'.txt', 'a')
                    f4.write('%s, %s, %s\n' % (position_wrt_world[0][0], position_wrt_world[1][0], position_wrt_world[2][0]))
                        
                    f5 = open(self.logpath+'velocities_wrt_world'+self.no+'.txt', 'a')
                    f5.write('%s, %s, %s\n' % (velocity_wrt_world[0][0], velocity_wrt_world[1][0], velocity_wrt_world[2][0]))
    
                    msg = Odometry()
                    msg.header.stamp = color.header.stamp
                    msg.header.frame_id = 'world'
                    msg.pose.pose.position.x = position_wrt_world[0][0]; msg.pose.pose.position.y = position_wrt_world[1][0]; msg.pose.pose.position.z = position_wrt_world[2][0]
                    msg.pose.pose.orientation.x = 0; msg.pose.pose.orientation.y = 0
                    msg.pose.pose.orientation.z = 0; msg.pose.pose.orientation.w = 1
                    msg.twist.twist.linear.x = velocity_wrt_world[0][0]; msg.twist.twist.linear.y = velocity_wrt_world[1][0]; msg.twist.twist.linear.z = velocity_wrt_world[2][0]
                    self.coord_pub.publish(msg)                
            
                print 'Total time taken in inference+kalman filter is:', time.time()-start
                self.tracking_counter += 1


if __name__ == '__main__':
    package_path = '/home/pelican/catkin_ws/src/quad_navigation_and_target_tracking/dnn/'
    PATH_TO_CKPT = package_path + 'FIG_michael_05082021_trt.pb'
    PATH_TO_LABELS = package_path + 'pascal_label_map.pbtxt'
    NUM_CLASSES = 1 
    run_number = rospy.get_param('run_number')
    ssd_detection_kalman_filter(PATH_TO_CKPT, PATH_TO_LABELS, NUM_CLASSES, run_number)


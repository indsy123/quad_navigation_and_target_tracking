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
import sys
import os
import cv2, time
import numpy as np
import tensorflow as tf
import ros_numpy
import rospy
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt
from object_detection.utils import label_map_util
from utils import visualization_utils as vis_util
from quad_navigation_and_target_tracking.msg import publish_bounding_boxes, box_coordinates
import message_filters
from tensorflow.python.compiler.tensorrt import trt_convert as trt
#from tensorflow.python.compiler.tensorrt import trt_convert as trt
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class ssd_detection_jackal(object):
    def __init__(self, ckpt_path, labels_path, n_classes): 
        """initializes various paramters"""
        rospy.init_node('detection_using_ssd_mobilenet', anonymous=False, log_level=rospy.DEBUG)
        self.start = time.time()
        self.logpath = '/home/pelican/data_collection/TRO/'
        #open(self.logpath+'inference_confidence.txt', 'w').close()

        self.PATH_TO_CKPT = ckpt_path
        self.PATH_TO_LABELS = labels_path
        self.NUM_CLASSES = n_classes
        output_names = ['detection_boxes', 'detection_scores', 'detection_classes', 'num_detections']
        input_names = ['image_tensor'] 
        tf.logging.set_verbosity(tf.compat.v1.logging.ERROR) # after first attemptipn is doesnt print all messages, tried to save time
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
        
        self.publish_boxes = rospy.Publisher('/udrone1/detection_boxes', publish_bounding_boxes, queue_size = 30, latch=True)
        
        rospy.Subscriber('/udrone1/d435/color/image_raw', Image, self.detect_jackal, buff_size = 2**24)

        try: 
            while not rospy.is_shutdown(): 
                 rospy.spin()
        except rospy.ROSInterruptException(): 
            pass


            
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
    
           

    def detect_jackal(self, data): 
        start  = time.time()
        image =  ros_numpy.numpify(data)#self.bridge.imgmsg_to_cv2(data, "bgr8")
        msg = publish_bounding_boxes()
        image_np_expanded = np.expand_dims(image, axis=0)        
        image_tensor = self.session.graph.get_tensor_by_name('image_tensor:0')
        boxes = self.session.graph.get_tensor_by_name('detection_boxes:0')
        scores = self.session.graph.get_tensor_by_name('detection_scores:0')
        classes = self.session.graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.session.graph.get_tensor_by_name('num_detections:0')
        (boxes, scores, classes, num_detections) = self.session.run([boxes, scores, classes, num_detections], feed_dict={image_tensor: image_np_expanded})
        alert_array = self.detect_alert(np.squeeze(boxes), np.squeeze(classes).astype(np.int32), np.squeeze(scores), self.category_index)  

        #print('alert_array:', alert_array)
        if len(alert_array) != 0: 
            msg.header.stamp = data.header.stamp
            msg.no_of_boxes = len(alert_array)
            
            for i in range(len(alert_array)):                     
                msg.label_and_confidence.append(str(alert_array[i]))
                bb = boxes[0][i]
                x = int(bb[1]*640)
                y = int(bb[0]*480)
                w = int((bb[3] - bb[1])*640)
                h = int((bb[2] - bb[0])*480)
                # make these values even
                x = x if x%2 == 0 else x-1
                y = y if y%2 == 0 else y-1
                w = w if w%2 == 0 else w-1
                h = h if h%2 == 0 else h-1
                
                s = scores[0][i]
                
                msg1 = box_coordinates()
                msg1.x = x; msg1.y = y; msg1.w = w; msg1.h = h; msg1.score = s
           	     
                msg.coordinates.append(msg1)
            self.publish_boxes.publish(msg)

            """
            if alert_array[0]['jackal'] > 50: 
                vis_util.visualize_boxes_and_labels_on_image_array(
                  image,
                  np.squeeze(boxes[0]),
                  np.squeeze(classes).astype(np.int32),
                  np.squeeze(scores),
                  self.category_index,
                  max_boxes_to_draw=1,
                  min_score_thresh=.5,
                  use_normalized_coordinates=True,
                  line_thickness=4)
                cv2.imshow("tagged_image", image)
                cv2.waitKey(1)
	    
            else: 
                print('detection confidence is below 50%')
            """
            
        print('Total time taken in inference is:', time.time()-start)


if __name__ == '__main__':
    package_path = '/home/pelican/catkin_ws/src/quad_navigation_and_target_tracking/dnn/'
    PATH_TO_CKPT = package_path + 'FIG_michael_05082021_trt.pb'
    PATH_TO_LABELS = package_path + 'pascal_label_map.pbtxt'
    NUM_CLASSES = 1 
    
    ssd_detection_jackal(PATH_TO_CKPT, PATH_TO_LABELS, NUM_CLASSES)


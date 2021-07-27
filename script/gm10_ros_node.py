#!/usr/bin/env python
# Dedicated to Adam who helped (actually made) the Domino Driver Board working, 10/24/2017
import rospy
import serial 
import numpy as np 
import time
from geometry_msgs.msg import PointStamped
#from matplotlib import pyplot as plt 


class GM10_Counts(object):
    def __init__(self, run_number, count_topic):
        """initialize the parameters"""
        rospy.init_node('GM10_counter', anonymous=False, log_level=rospy.DEBUG)
	self.count_topic = str(count_topic)
        self.pub = rospy.Publisher(self.count_topic, PointStamped, queue_size = 1)


        #self.logpath = '/home/pelican/data_collection/'
        #open(self.logpath + "gammaray_counts.txt", "w").close()
        
        ser = serial.Serial()
        ser.port = "/dev/ttyUSB0"
        ser.baudrate = 57600
        ser.bytesize = serial.EIGHTBITS 
        ser.parity = serial.PARITY_NONE 
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = None         
        ser.timeout = 1              
        ser.xonxoff = False    
        ser.rtscts = False    
        ser.dsrdtr = False       
        ser.writeTimeout = 2  

        try:
            ser.open()
        except Exception, e:
            print "error opening serial port: " + str(e)
            exit()    

        start_time = time.time()    
        if ser.isOpen():
            try:
                counts = []
                ser.flushInput()        
                ser.flushOutput()
                numOfLines = 0
                while True:
                    response = ser.readline()           
            	    counts.append(len(response))
            	    numOfLines = numOfLines + 1
            	    print 'time elapsed and total counts observed:', time.time()-start_time, len(response), sum(counts)
                    msg = PointStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.point.x = len(response)
                    self.pub.publish(msg)    
                print 'finished reading..now closing the serial port'
                ser.close()
            except Exception, e1:
                print "error communicating...: " + str(e1)

        else:
            print "cannot open serial port "

if __name__ == '__main__':
    run_number = rospy.get_param('run_number')
    count_topic = rospy.get_param('count_topic') 
    GM10_Counts(run_number, count_topic)    





import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String, Float64MultiArray, Header
from aauship.msg import *
from aauship.srv import Waypoint
import time
import os 
import message_filters
import numpy as np
import scipy.io as sio
import math

class GPS_test(object):

    CENTER_lat = 57.014607991772024
    CENTER_lng = 9.986213184893131
    SCALE = 1 / 0.00001
        
    def __init__(self):
        self.i = 0
        self.first = True
        self.lat = [0, 0]
        self.long = [0, 0]
        self.psi = [0, 0]
        
        rospy.init_node('GPS_test')
        
        rospy.wait_for_service('wp')
        self.wp_srv = rospy.ServiceProxy('wp', Waypoint)
        self.wp = self.wp_srv()
        print self.wp
        
        data_sub = message_filters.Subscriber('kf_statesnew', KFStates)
        gps_sub = message_filters.Subscriber('gps2', GPS)
        self.pub = rospy.Publisher('lli_input', LLIinput, queue_size=1)
        ts = message_filters.TimeSynchronizer([data_sub, gps_sub], 1)
        ts.registerCallback(self.callback)

	
    def callback(self, data, gps):
        if self.first:
            self.x_k_1 = (gps.latitude - GPS_test.CENTER_lat)*GPS_test.SCALE
            self.y_k_1 = (gps.longitude - GPS_test.CENTER_lng)*GPS_test.SCALE
            self.first = False
            return

        self.psi[self.i % 2] = data.psi
        self.lat[self.i % 2] = (gps.latitude - GPS_test.CENTER_lat)*GPS_test.SCALE
        self.long[self.i % 2] = (gps.longitude - GPS_test.CENTER_lng)*GPS_test.SCALE

        print '#########################'
        print [gps.latitude,gps.longitude]
        print [self.lat[self.i % 2],self.long[self.i % 2]]
        self.i += 1

if __name__ == '__main__':
    gps_test = GPS_test()
    while not rospy.is_shutdown():
        pass
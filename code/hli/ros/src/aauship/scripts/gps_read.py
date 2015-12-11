#!/usr/bin/env python2

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String, Float64MultiArray, Header
import numpy as np

class GpsRead(object):
	
	def __init__(self):

		gps = rospy.Subscriber('gps2', GPS, callback)

	def callback(self, gps):

		self.lat = gps.latitude
        self.long = gps.longitude

        print('#'*80)
        print("N:", self.lat)
        print("E:", self.long)

if __name__ == '__main__':
    gps = GpsRead()
    while not rospy.is_shutdown():
        pass
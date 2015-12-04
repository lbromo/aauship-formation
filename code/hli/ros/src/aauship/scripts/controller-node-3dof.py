#!/usr/bin/env python2

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String, Float64MultiArray, Header
from aauship.msg import *
import time
import os 
import message_filters
import numpy as np

class Controller(object):
	def __init__(self):
		self.i = 0
		self.lat = [0, 0]
		self.long = [0, 0]
		self.psi = [0, 0]
		rospy.init_node('Controller')
		#self.sub = rospy.Subscriber('kf_statesnew', Float64MultiArray, self.angle_callback)
		#self.sub = rospy.Subscriber('gps2', GPS, self.gps2_callback)
		
		# Loading Matrices from MATLAB Model and Controller
		matrices = sio.loadmat('../../../../../matlab/3_dof.mat')
		# State Space Model Matrices
		self.A = matrices['Ad']
		self.B = matrices['Bd']
		self.C = matrices['Cd']
		# LQR Proportional Controller
		self.LQR = matrices['LQR']
		
		data_sub = message_filters.Subscriber('kf_statesnew', KFStates)
		gps_sub = message_filters.Subscriber('gps2', GPS)

		ts = message_filters.TimeSynchronizer([data_sub, gps_sub], 1)
		ts.registerCallback(self.callback)
	
	def callback(self, data, gps):
		self.psi[self.i % 2] = data.psi
		self.lat[self.i % 2] = gps.latitude
		self.long[self.i % 2] = gps.longitude
		self.r = self.psi[self.i % 2] - self.psi[(self.i-1) % 2]
		self.u = self.lat[self.i % 2] - self.lat[(self.i-1) % 2]
		self.v = self.long[self.i % 2] - self.long[(self.i-1) % 2]
		self.i += 1

		self.x = np.matrix([[self.lat], [self.long], [self.psi], [self.u], [self.v], [self.r]])

		print('#'*80)
		print("N:", self.lat[self.i % 2])
		print("E:", self.long[self.i % 2])
		print("PSI:", self.psi[self.i % 2])
		print("u:", self.u)
		print("v:", self.v)
		print("r:", self.r)

	def plos_EBS(x_k,x_k_1,y_k,y_k_1,x,y,n,L):
		# Constant slope between to points:
		#
		#                   yk+1 - yk       ylos - yk
		# tan(alpha_k) =  ------------- = ------------- = constant
		#                   xk+1 - xk       xlos - xk
		# 
		# Pythagoras Theorem: (Radius of search)
		#
		# [xlos - x(t)]^2 + [ylos - y(t)]^2 = (nL)^2

		delta_x = x_k - x_k_1
		delta_y = y_k - y_k_1
		

		if delta_x == 0: 
			x_los = x_k_1 #x_los = x_k;
			if delta_y > 0: 
				y_los = y + n*L
			else:#delta_y < 0
				y_los = y - n*L

		else: #delta_x~=0
			d = delta_y/delta_x
			e = x_k_1
			f = y_k_1
			g = -d*e + f
			a = 1 + d**2
			b = 2*(d*g - d*y - x)
			c = x**2 + y**2 + g**2 - (n*L)**2 - 2*y*g

			#ERROR HANDLING
			if (b**2 - 4*a*c) < 0:
				print 'ERROR'
				print '[W]',x_k,y_k
				print '[B]',x,y

			if delta_x > 0:
				x_los = (-b + math.sqrt(b**2 - 4*a*c) )/(2*a)
			else:  #delta_x < 0
				x_los = (-b - math.sqrt(b**2 - 4*a*c) )/(2*a);
		
			y_los = d*(x_los - e) + f;
		
		# If we're going UP in North
	    if (x_k > x_k_1):
	        if x_los > x_k:
	            x_los = x_k
	    # If we're going DOWN in North
	    if (x_k < x_k_1):
	        if x_los < x_k:
	            x_los = x_k

	    # If we're going RIGHT in East
	    if (y_k > y_k_1):
	        if y_los > y_k:
	            y_los = y_k
	    # If we're going LEFT in East
	    if (y_k < y_k_1):
	        if y_los < y_k:
	            y_los = y_k

		return x_los,y_los;


if __name__ == '__main__':
	controller = Controller()
	while not rospy.is_shutdown():
		pass

#!/usr/bin/env python2

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

class Controller(object):
        
    acceptance = 2
    n = 3 # Boat search radius
    L = 1 # Boat Length
    CENTER_lat = 57.01531876758453
    CENTER_lng = 9.977239519357681
    SCALE = 1 / 0.00001
        
    def __init__(self):
        self.i = 0
        self.first = True
        self.lat = [0, 0]
        self.long = [0, 0]
        self.psi = [0, 0]
        
        # Loading Matrices from MATLAB Model and Controller
        matrices = sio.loadmat('../../../../../matlab/3_dof.mat')
        # State Space Model Matrices
        self.A = matrices['Ad']
        self.B = matrices['Bd']
        self.C = matrices['Cd']
        # LQR Proportional Controller
        self.LQR = matrices['LQR']
        
        rospy.init_node('Controller')
        
        rospy.wait_for_service('wp')
        self.wp_srv = rospy.ServiceProxy('wp', Waypoint)
        self.wp = self.wp_srv()
        print self.wp
        
        data_sub = message_filters.Subscriber('kf_statesnew', KFStates)
        gps_sub = message_filters.Subscriber('gps2', GPS)
        self.pub = rospy.Publisher('lli_input', LLIinput, queue_size=1)
        ts = message_filters.TimeSynchronizer([data_sub, gps_sub], 1)
        ts.registerCallback(self.callback)

        
        self.ref = np.matrix('%s; %s; 0; 0; 0; 0' % (self.wp.lat, self.wp.long))
        self.x_k = (self.wp.lat - Controller.CENTER_lat)*Controller.SCALE
        self.y_k = (self.wp.long - Controller.CENTER_lng)*Controller.SCALE
        self.x_k_1 = 0
        self.y_k_1 = 0

	
    def callback(self, data, gps):
        if self.first:
            self.x_k_1 = (gps.latitude - Controller.CENTER_lat)*Controller.SCALE
            self.y_k_1 = (gps.longitude - Controller.CENTER_lng)*Controller.SCALE
            self.first = False
            return

        self.psi[self.i % 2] = data.psi
        self.lat[self.i % 2] = (gps.latitude - Controller.CENTER_lat)*Controller.SCALE
        self.long[self.i % 2] = (gps.longitude - Controller.CENTER_lng)*Controller.SCALE
        self.r = self.psi[self.i % 2] - self.psi[(self.i-1) % 2]
        self.u = self.lat[self.i % 2] - self.lat[(self.i-1) % 2]
        self.v = self.long[self.i % 2] - self.long[(self.i-1) % 2]
        
        self.x = np.matrix([[self.lat[self.i % 2]], [self.long[self.i % 2]], [self.psi[self.i % 2]], [self.u], [self.v], [self.r]])
        self.send_controller_output()
        self.i += 1
        #print('#'*80)
        #print("N:", self.lat[self.i % 2])
        #print("E:", self.long[self.i % 2])
        #print("PSI:", self.psi[self.i % 2])
        #print("u:", self.u)
        #print("v:", self.v)
        #print("r:", self.r)

    def send_controller_output(self):
        m = [0, 0]
        R = np.matrix( [
            [math.cos(self.psi[self.i % 2]), -math.sin(self.psi[self.i % 2]), 0, 0, 0, 0],
            [math.sin(self.psi[self.i % 2]), math.cos(self.psi[self.i % 2]), 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ] )
        print self.x
        y = R * self.C*self.x
        
        distance = math.sqrt((self.x_k-y[0])**2+(self.y_k-y[1])**2)
        if distance < Controller.acceptance:
            self.wp = self.wp_srv()
            print self.wp
            self.x_k_1 = self.x_k
            self.y_k_1 = self.y_k
            self.x_k = (self.wp.lat - Controller.CENTER_lat)*Controller.SCALE
            self.y_k = (self.wp.long - Controller.CENTER_lng)*Controller.SCALE
            
        x_los,y_los = self.plos_EBS(self.x_k,self.x_k_1,self.y_k,self.y_k_1,float(y[0]),float(y[1]),Controller.n,Controller.L)
        ref = np.matrix('%s; %s; 0; 0; 0; 0' % (x_los, y_los))
        e = R.transpose() * (self.ref - y)
        
        u = self.LQR * e
        
        print (distance, x_los, y_los)
        
        msg = LLIinput()
        
        if(u[0] > 0):
            m[0] = (u[0] + 26.84) / 0.2746
            if m[0] < 70:
                m[0] = 70
        elif(u[0] < 0):
            m[0] = (u[0] + 7.318) / 0.1152
            if m[0] > -70:
                m[0] = -70

        if(u[1] > 0):
            m[1] = (u[1] + 26.84) / 0.2746
            if m[1] < 70:
                m[1] = 70
        elif(u[1] < 0):
            m[1] = (u[1] + 7.318) / 0.1152
            if m[1] > -70:
                m[1] = -70
                                    
            
        msg.DevID = 10
        msg.MsgID = 3
        msg.Data = m[0]
        self.pub.publish(msg)
        msg.DevID = 10
        msg.MsgID = 5
        msg.Data = m[1]
        self.pub.publish(msg)
                

    def plos_EBS(self,x_k,x_k_1,y_k,y_k_1,x,y,n,L):
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

#!/usr/bin/env python2 

##########################################
# 02/12/15
# Pick up the angle from the magnetometor
# Lasse/Kelvin/Kenny
##########################################

import rospy
from std_msgs.msg import Float64MultiArray
    
def callback(data):
        rospy.loginfo("I heard %s",data.data[6])
        print(data)

rospy.init_node('node_name')
Sub_Angle = rospy.Subscriber("kf_statesnew", Float64MultiArray, callback)
# spin() simply keeps python from exiting until this node is stopped
# print(Sub_Angle)      	
rospy.spin()



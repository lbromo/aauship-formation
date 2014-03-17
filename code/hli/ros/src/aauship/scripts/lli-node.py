#!/usr/bin/env python

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import serial
import struct
import time

import Queue
import fapsParse  # fork of packetparser
import fapsPacket # fork of packetHandler
import numpy as np

class LLI(object):
    def callback(self, data):
        # write data to serial
        print "Requesting buildinfo " + str(time.time())
        jeppe = self.packet.package([],0,9)
        self.packet.lli_send(jeppe)
        
        rospy.loginfo(data.data)
        pass

    def run(self):
        self.qu = Queue.Queue()
        self.packet = fapsPacket.packetHandler('/dev/lli', 57600, 0.02, self.qu)

#        self.parser = fapsParse.packetParser

        time.sleep(5)
        self.packet.start()
        pub = rospy.Publisher('samples', Faps)
#        pub = rospy.Publisher('samples', String)
        sub = rospy.Subscriber('lli_input', String, self.callback)
        rospy.init_node('lli')
        r = rospy.Rate(100) # Rate in Hz

        while not rospy.is_shutdown():
            try:
                data = self.qu.get(False)
                print "RAW:" + str((data['DevID']))
                pub.publish(ord(data['DevID']),ord(data['MsgID']),''.join(data['Data']))
            except Queue.Empty:
                pass
            r.sleep()

        self.packet.close()
        self.packet.join()
        
if __name__ == '__main__':
    w = LLI()
    w.run()

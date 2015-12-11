#!/usr/bin/env python2

import rospy
from aauship.srv import *


class WP():
    
    def __init__(self, wps):
        rospy.init_node('wp')
        self.wps = wps
        self.i = 0
        self.wp = rospy.Service('wp', Waypoint, self.serv)

    def serv(self, req):
        wp = Waypoint._response_class()
        if self.i < len(wps):
            wp.lat = self.wps[self.i][0]
            wp.long = self.wps[self.i][1]
        else:
            wp.lat = self.wps[-1][0]
            wp.long = self.wps[-1][0]

        self.i += 1
        return wp

    def run(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    wps = [[57.015207789616156,9.977458789944649],[57.01529686406438,9.977692812681198], [57.01541295257421,9.97741587460041], [57.01531876758453,9.977204650640488], [57.015207789616156, 9.977458789944649]]
    wp = WP(wps)
    wp.run()


        

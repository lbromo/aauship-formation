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
    wps = [[1,1], [2,2]]
    wp = WP(wps)
    wp.run()


        

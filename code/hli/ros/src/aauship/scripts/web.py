import rospy
import gpsfunctions

from bottle import route, run
from aauship.msg import *

last_lat, last_lng = (0, 0)

@route('/')
def index():
    f = open('gps_map.html', 'r')
    page = f.read()
    f.close()
    return page

def callback(gps_msg):
    global last_lat, last_lng
    latitude, longitude = gpsfunctions.nmea2decimal(gps_msg.latitude, 'N', gps_msg.longitude, 'E')
    gps_msg.latitude = latitude
    gps_msg.longitude = longitude
    if (abs(last_lat - gps_msg.latitude) > 0.000009 or abs(last_lng - gps_msg.longitude) > 0.000009):
        last_lat = gps_msg.latitude
        last_lng = gps_msg.longitude
        pub.publish(gps_msg)
        print gps_msg

        

if __name__ == '__main__':
    rospy.init_node('web')
    sub = rospy.Subscriber('gps2', GPS, callback)
    pub = rospy.Publisher('web', GPS, queue_size=1)
    run(host='localhost', port=8080, debug=True)

import threading
import rospy
import time
import Queue

from aauship.msg import *

msg = LLIinput()
lli = rospy.Publisher('lli_input', LLIinput, queue_size=10)

class Logger(threading.Thread):
    
    def __init__(self, log_path):
        threading.Thread.__init__(self)
        self.log_path = log_path
        self.input_queue = Queue.Queue(10)

        rospy.Subscriber('gps2', GPS, self.gps2_cb, queue_size=1)
        rospy.Subscriber('gps1', GPS, self.gps1_cb, queue_size=1)
        rospy.Subscriber('lli_input', LLIinput, self.lli_cb, queue_size=1)    
        rospy.Subscriber('imu', ADIS16405, self.imu_cb, queue_size=1)
        

    def gps2_cb(self, data):
        line = "GPS2, %f, %f, %f" % (data.date, data.latitude, data.longitude)
        self.input_queue.put(line)

    def gps1_cb(self, data):
        line = "GPS1, %f, %f, %f" % (data.date, data.latitude, data.longitude)
        self.input_queue.put(line)

    def lli_cb(self, data):
        line = "LLI, %d, %d, %d" % (data.DevID, data.MsgID, data.Data)
        self.input_queue.put(line)

    def imu_cb(self, data):
        line = "IMU, %f, %f, %f, %f, %f, %f, %f, %f, %f" % (data.xgyro, data.ygyro, data.zgyro,
                                                        data.xaccl, data.yaccl, data.zaccl,
                                                        data.xmagn, data.ymagn, data.zmagn)
        self.input_queue.put(line)

    def run(self):
        with open(self.log_path, "w") as f:
            while not rospy.is_shutdown():
                line = self.input_queue.get()
                print line
                f.write(line + "\n")



def constant(thrust_r, thrust_l, run_time):
    while time.time() < run_time and not rospy.is_shutdown():
        msg.DevID = 10
        msg.MsgID = 3
        msg.Data = thrust_r
        lli.publish(msg)
        
        print msg
        
        msg.DevID = 10
        msg.MsgID = 5
        msg.Data = thrust_l
        lli.publish(msg)

        print msg

        time.sleep(1)

def alternating(t1, t2, run_time):
    while time.time() < run_time and not rospy.is_shutdown():
        msg.DevID = 10
        msg.MsgID = 3
        msg.Data = t1
        lli.publish(msg)
        
        print msg

        msg.DevID = 10
        msg.MsgID = 5
        msg.Data = t2
        lli.publish(msg)

        print msg

        tmp = t1
        t1 = t2
        t2 = tmp
        time.sleep(3)

    

if __name__ == '__main__':
    # (-100% = -500 to +100% = 500)
    # right thruster, devid 10, msgid 3
    # left thruster, devid 10, msgid 5
    rospy.init_node("step")

    t1 = int(sys.argv[2])
    t2 = int(sys.argv[3])
    run_time = time.time() + int(sys.argv[4])

    l = Logger(sys.argv[5])
    l.daemon = True
    l.start()

    if sys.argv[1] == 'c':
        constant(t1, t2, run_time)
    else:
        alternating(t1, t2, run_time)


    print("Stopping")
    msg.DevID = 10
    msg.MsgID = 3
    msg.Data = 0
    lli.publish(msg)
    
    msg.DevID = 10
    msg.MsgID = 5
    msg.Data = 0
    lli.publish(msg)
    
    print "done"
    time.sleep(0.5)


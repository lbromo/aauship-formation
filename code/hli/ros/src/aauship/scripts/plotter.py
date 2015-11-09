import numpy as np
import matplotlib.pylab as plt
import rospy
from aauship.msg import *

N = 100
gps_idx = 0
imu_idx = 0

gps_data = {'lat': np.array([None]*N), 'lng': np.array([None]*N)}
imu_data =  {'x_gyro': np.array([None]*N), 
             'y_gyro': np.array([None]*N),
             'z_gyro': np.array([None]*N),
             'x_accel': np.array([None]*N),
             'y_accel': np.array([None]*N),
             'z_accel': np.array([None]*N),
             'x_mag': np.array([None]*N),
             'y_mag': np.array([None]*N),
             'z_mag': np.array([None]*N)
}
fig = plt.figure()
gps_plt = fig.add_subplot(1,2,1)
imu_plt = fig.add_subplot(1,2,2)

def callback(msg):
    if type(msg) == GPS:
        update_gps(msg)
    elif type(msg) == ADIS16405:
        update_imu(msg)

def update_gps(gps_msg):
    global gps_idx, gps_data
    gps_data['lat'].put(gps_idx, float(gps_msg.latitude))
    gps_data['lng'].put(gps_idx, float(gps_msg.longitude))
    gps_idx = (gps_idx + 1) % N
    
    gps_plt.clear()
    gps_plt.scatter(gps_data['lat'], gps_data['lng'])
    gps_plt.set_ylim([0,10])
    gps_plt.set_xlim([0,10])
    #fig.canvas.draw()

def update_imu(imu_msg):
    global imu_idx, gps_data
    imu_data['x_accel'].put(imu_idx, float(imu_msg.xaccl))
    imu_data['y_accel'].put(imu_idx, float(imu_msg.yaccl))
    imu_data['z_accel'].put(imu_idx, float(imu_msg.zaccl))
    imu_idx = (imu_idx + 1) % N

    imu_plt.clear()
    imu_plt.plot(imu_data['x_accel'])
    imu_plt.plot(imu_data['y_accel'])
    imu_plt.plot(imu_data['z_accel'])
    fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("plotter")
    gps_sub = rospy.Subscriber('gps1', GPS, callback)
    imu_sub = rospy.Subscriber('imu', ADIS16405, callback)
    plt.show()
    rospy.spin()


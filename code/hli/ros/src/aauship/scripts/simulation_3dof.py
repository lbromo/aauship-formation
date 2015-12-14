import pylab
import math
import scipy.io as sio
import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray
from aauship.msg import *

rospy.init_node('sim')

pos_pub = rospy.Publisher('simgps', SimGPS, queue_size=10)
wp_pub = rospy.Publisher('wp', wp, queue_size=10)

CENTER_lat = 57.01531876758453
CENTER_lng = 9.977239519357681

SCALE = 1 / 0.00001

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
	return x_los,y_los;



# Number of iterations
runs = 120

# Loading Matrices from MATLAB Model and Controller
matrices = sio.loadmat('../../../../../matlab/3_dof.mat')
# State Space Model Matrices
A = matrices['Ad']
B = matrices['Bd']
C = matrices['Cd']
# LQR Proportional Controller
LQR = matrices['LQR']

# State Space vector: [N E Psi u v r] 
x = [ np.matrix('0; 0; 0; 0; 0; 0') ]
# Output Vector: [N E Psi] 
y = []
# Input Vector: [Fx Fy Torque] POSIBLE ERROR
u = []
# Error Vector: [N-error E-error Psi-error]
e = []
# Reference Vector: Updates with LOS pathing
ref = []
# Waypoint Table
#waypoint_table = [[9.1,9.1], [9.2,9.0], [9.1,8.9], [9.0, 9.0]]
waypoint_table = [
    [57.01532606875507, 9.97702494263649],
    [57.015415142919984, 9.9772609770298], 
    #[57.015431205451584, 9.977575466036797], 
    [57.01542098384136, 9.977424591779709], 
    [57.015414412804724, 9.977578148245811],
    [57.01540711165156, 9.977733716368675],
    [57.015453839007094, 9.977870173752308],
    [57.015408936939984, 9.97797679156065],
    [57.015379002198465, 9.977833963930607],
    [57.01527204005926, 9.977700859308243],
    [57.01515084026695, 9.977680072188377],
    [57.01514791978515, 9.97740849852562],
    [57.015242835326156, 9.977218061685562],
    [57.01532606875507, 9.97702494263649],
]
# Initial conditions 
#x[0] = np.matrix('9; 9; 0; 0; 0; 0')
x_0 = 0
y_0 = 0
x[0] = np.matrix('%s; %s; 0; 0; 0; 0' % (x_0, y_0))
distance = []
n = 5 # Boat search radius
L = 1 # Boat Length
x_k = (waypoint_table[0][0] - CENTER_lat) * SCALE
y_k = (waypoint_table[0][1] - CENTER_lng) * SCALE 
x_k_1 = x_0
y_k_1 = y_0
acceptance = 1

i = 0
j = 0
while True:
    psi = x[-1][2]
    # ROTATION MATRIX
    R = np.matrix( [
                    [math.cos(psi), -math.sin(psi), 0, 0, 0, 0],
                    [math.sin(psi), math.cos(psi), 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1
]                    ] );
    # Y in VP coordinates to NED
    y.append(R * (C*x[-1])) 

    # Are we in the acceptance radius now? 
    # Yes > Update waypoint before LOS algorithm
    # No > Calculate LOS position
    print "[W]", x_k, y_k

    distance.append(math.sqrt((x_k-y[-1][0])**2+(y_k-y[-1][1])**2))
    print '[D]', distance[-1]
    if distance[-1] < acceptance:
        i += 1
        if i > len(waypoint_table)-1:
            break
        x_k = (waypoint_table[i][0] - CENTER_lat) * SCALE
        y_k = (waypoint_table[i][1] - CENTER_lng) * SCALE
        x_k_1 = (waypoint_table[i-1][0] - CENTER_lat) * SCALE 
        y_k_1 = (waypoint_table[i-1][1] - CENTER_lng) * SCALE 

        msg = wp()
        msg.lat = waypoint_table[i][0]
        msg.long = waypoint_table[i][1]
        msg.aoa = acceptance
        wp_pub.publish(msg)
        print msg
        plt.gca().add_artist(plt.Circle((y_k,x_k),acceptance,color='r', alpha=.9))

        print '###################################'
        print 'Loop number', i 
        print '[W]', x_k, y_k
        print '###################################'


    
    # Reference by LOS Pathing
    x_los,y_los = plos_EBS(x_k,x_k_1,y_k,y_k_1,float(y[-1][0]),float(y[-1][1]),n,L);    
    
    print "[N]", x_los,y_los

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

    print "[T]", x_los,y_los
    print "[B]", float(y[-1][0]), float(y[-1][1])
    
    if j % 15 == 0:
        plt.gca().add_artist(plt.Circle((float(y[-1][1]),float(y[-1][0])),n*L,color='g', alpha=0.1))

    msg = SimGPS()
    msg.latitude = float((y[-1][0] / SCALE) + CENTER_lat)
    msg.longitude = float((y[-1][1] / SCALE) + CENTER_lng)
    msg.aos = L*n
    pos_pub.publish(msg)

    j += 1

    #Reference equal to the LOS position
    ref.append(np.matrix('%s; %s; 0; 0; 0; 0' % (x_los, y_los)))

    # Error in VP coordinates
    e.append(R.transpose() * (ref[-1] - y[-1])) 
    
    # WHERE THE MAGIC HAPPENS    
    u.append(LQR*e[-1])

    
    if (u > 0):
            to_motors = [(u[-1][0] + 26.84) / 0.2746,  (u[-1][1] + 26.84) / 0.2746]

    elif (u < 0):
            to_motors = [(u[-1][0] + 7.318) / 0.1152,  (u[-1][1] + 7.318) / 0.1152]

    print "[M]", to_motors

    x.append(A*x[-1] + B*u[-1])
    

plt.gca().set_xlim((-30,80))
plt.gca().set_ylim((-20,20))

# Export signals
e_north = [float(e[i][0]) for i in range(len(e)-1) ]
e_east = [float(e[i][1]) for i in range(len(e)-1) ]
e_psi = [float(e[i][2]) for i in range(len(e)-1) ]

north = [float(y[i][0]) for i in range(len(y)-1)]
east = [float(y[i][1]) for i in range(len(y)-1)]
psi = [float(y[i][2]) for i in range(len(y)-1)]

vel_u = [float(x[i][3]) for i in range(len(x)-1)]
vel_v = [float(x[i][4]) for i in range(len(x)-1)]
vel_r = [float(x[i][5]) for i in range(len(x)-1)]

Fx = [float(u[i][0]) for i in range(len(u)-1)]
Fy = [float(u[i][1]) for i in range(len(u)-1)]
torque = [float(u[i][2]) for i in range(len(u)-1)]

reference = [float(ref[i][0]) for i in range(len(ref)-1) ]

#print Fx
'''
# Representation of errors
plt.plot(e_north, label='North error')
plt.plot(e_east, label='East error')
plt.plot(e_psi, label='$\psi$ error')
plt.grid()
pylab.legend(loc='upper right')
pylab.xlabel('Time [s]')
pylab.ylabel('Relative amplitude')
pylab.title('Error signals')
plt.show()
plt.savefig('step_err.eps', format='eps', dpi=1000, bbox_inches='tight')

# Representation of Y
plt.plot(north, label='N')
plt.plot(east, label='E')
plt.plot(psi, label='$\psi$')
plt.grid()
pylab.legend(loc='upper right')
pylab.xlabel('Time [s]')
pylab.ylabel('Relative amplitude')
pylab.title('Step response')
plt.savefig('step.eps', format='eps', dpi=1000, bbox_inches='tight')
'''
# Representation of Position in NED FRAME
plt.gca().scatter(east, north)
pylab.ylabel('North')
pylab.xlabel('East')
plt.grid(True,'major')
plt.grid(True,'minor')
pylab.title('Response in the NED frame')
#plt.savefig('simulation_plot.eps', format='eps', dpi=1000, bbox_inches='tight')
plt.savefig('LOS_simulation.eps', format='eps', dpi=1000, bbox_inches='tight')
#plt.show()
'''
# Heading Vector Representation
Eih,Nih,Eoh,Noh = zip([east,north, [0.1*(east[i]+math.sin(psi[i])) for i in range(len(psi))], [0.1*(north[i]+math.cos(psi[i])) for i in range(len(psi))]])
plt.quiver(Eih,Nih,Eoh,Noh,angles='xy',scale_units='xy',scale=1)


# Velocities Representation
plt.plot(vel_u, label='Velocity X')
plt.plot(vel_v, label='Velocity Y')
plt.plot(vel_r, label='Angular Velocity')
plt.grid()
pylab.legend(loc='upper right')
pylab.xlabel('Time [s]')
pylab.ylabel('Relative amplitude')
pylab.title('Velocities Body frame')
plt.savefig('step.eps', format='eps', dpi=1000, bbox_inches='tight')

# Representation Inputs
plt.plot(Fx, label='Fx')
plt.plot(Fy, label='Fy')
plt.plot(torque, label='Torque')
plt.grid()
pylab.legend(loc='upper right')
pylab.xlabel('Time [s]')
pylab.ylabel('Newtons')
pylab.title('Forces Body Frame')
plt.savefig('step.eps', format='eps', dpi=1000, bbox_inches='tight')
'''

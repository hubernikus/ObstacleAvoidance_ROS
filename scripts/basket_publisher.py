#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''

import sys 
lib_string = "/home/lukas/catkin_ws/src/obstacle_recognition/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)
    
from lib_obstacleAvoidance import * # rotation matrix

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Pose, Point
from obstacle_recognition.msg import Obstacle
from matplotlib.patches import Polygon

import numpy as np
from math import pi, floor

import tf

import warnings

class obstaclePublisher():
    def __init__(self, a=[1,1,1], p=[1,1,1], x0=[0,0,0], th_r=[0,0,0], sf=1, sigma=1, dx=[0,0,0], w=[0,0,0], init_disp=[0,0,0]):
        # Initialize variables -- currently constant obstacle geometry
        self.a=a
        self.p = p
        self.sf = sf
        self.sigma = sigma
        #self.gamma = gamma ?!

        # Time varying parameters for moving obstacle
        # TODO - change to initial parameters to easy initialization of KALMAN FILTER
        self.th_r = th_r
        self.th_r_rel = th_r # Relative orientation w.r.t BRF

        # Movement parameters
        self.dx = dx
        self.w = w

        # Dimension of 
        self.dim=len(x0)
        if self.dim != 3:
            print('WARNING -- Simualtion implemented for only 3D, not D=', self.dim)

        # Calculate rotation matrix
        self.rotMatrix = compute_rotMat(self.th_r, self.dim)

        print(init_disp)
        self.x0 = self.rotMatrix.T.dot(np.array(init_disp)) + np.array(x0)
        # Relative postition w.r.t. body reference frame (BRF)
        self.x0_rel = self.rotMatrix.T.dot(np.array(init_disp)) + np.array(x0)

        # Create point cloud outline
        self.draw_ellipsoid_centered()

        # Initialize node
        rospy.init_node('attractor_publisher', anonymous=True)
        rate = rospy.Rate(1) # Frequency

        # Create publishers
        elli_pub = rospy.Publisher('attr_poly', PolygonStamped, queue_size=5)
        obs_pub = rospy.Publisher('basket', Obstacle, queue_size=5)
        br_obs_pub = tf.TransformBroadcaster()
        
        # Create listener
        pose_sub = rospy.Subscriber("attr/pose", PoseStamped, self.callback)

        self.time = rospy.get_time()

        self.itCount = 0
        
        # Enter main loop
        #while False:
        while not rospy.is_shutdown():
            # Obstacle for algorithm
            ell_poly = PolygonStamped()
            ell_poly.header.stamp = rospy.Time.now()
            ell_poly.header.frame_id = 'world'

            # Publish new frame to new frame
            x_obs = self.x_obs
            ell_poly.polygon.points = [Point32(self.x_obs[i][0], self.x_obs[i][1], self.x_obs[i][2]) for i in range(len(self.x_obs))]
            elli_pub.publish(ell_poly)

            # Obstacle for algorithm
            # TODO -- for constant shape, keep those factors out of loop
            obstacle = Obstacle()
            obstacle.header = ell_poly.header

            # Time varying parameters - constant because staying in body reference frame
            obstacle.x0 = self.x0_rel 
            obstacle.th_r = self.th_r_rel

            # Dynamics properties
            obstacle.xd = self.dx
            obstacle.w = self.w

            # Constant obstacle properties
            obstacle.a = self.a
            obstacle.p = self.p
            obstacle.sigma = self.sigma
            obstacle.sf = self.sf
            
            obs_pub.publish(obstacle)
            
            rate.sleep()  # Wait zzzz*

            self.itCount += 1
            N_pubInterval = 100
            if self.itCount > N_pubInterval:
                # Loginfo concerning publishing
                rospy.loginfo("100 attractors published".format(N_pubInterval))
                self.itCount=0
                

    def callback(self, msg): # Subscriber callback
        # Time iteration
        timeOld = self.time
        self.time = rospy.get_time()
        dt = self.time-timeOld
        
        # Position values
        x0_old = self.x0
        self.x0 = msg.pose.position # TODO -- KALMAN FILTERING FOR POSITION
        self.x0 = [self.x0.x, self.x0.y, self.x0.z]  # Compatibility with original function



        deltaX = np.array([self.x0[0] - x0_old[0],
                           self.x0[1] - x0_old[1],
                           self.x0[2] - x0_old[2]])
        dx_new = 1.0*deltaX/dt
        
        k_dx = 0.6 # kalman filter constant
        
        self.dx = k_dx*np.array(self.dx) + (1.-k_dx)*dx_new

        # Orientation values
        q = msg.pose.orientation
        th_r_old = self.th_r # TODO -- KALMAN FILTERING FOR ORIENTATION
        self.th_r = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        deltaTH = np.array(( [self.th_r[i]-th_r_old[i] for i in range(self.dim)] ))
        
        for ii in range(3): # CHECK direction of angle
            if np.abs(deltaTH[ii]) > pi:
                deltaTH[ii] = np.copysign(2*pi - (deltaTH[ii]), deltaTH[ii])
        w_new = deltaTH/dt

        k_w = 0.6 # kalman filter constant

        self.w = k_w*np.array(self.w) + (1-k_w)*w_new

    def draw_ellipsoid_centered(self, numPoints=32, a_temp = [0,0], draw_sfObs = False, x0=[0,0,0]):
        self.numPoints = numPoints 
        if self.dim == 2:
            theta = np.linspace(-pi,pi, num=numPoints)
            #numPoints = numPoints
        else:
            theta, phi = np.meshgrid(np.linspace(-pi,pi, num=numPoints),np.linspace(-pi/2,pi/2,num=floor(numPoints/2) ) ) #

            #numPoints = numPoints[0]*numPoints[1]
            numPoints = numPoints*floor(numPoints/2)
            self.resolution = numPoints
            theta = theta.T
            phi = phi.T

        # For an arbitrary shap, the next two lines are used to find the shape segment
        if hasattr(self,'partition'):
            warnings.warn('Partitions not implemented')
            for i in range(self.partition.shape[0]):
                ind[i,:] = self.theta>=(self.partition[i,1]) & self.theta<=(self.partition[i,1])
                [i, ind]=max(ind)
        else:
            ind = 0
            
        #a = obs[n].a[:,ind]
        #p = obs[n].p[:,ind]

        # TODO -- add partition index
        if sum(a_temp) == 0:
            a = self.a
        else:
#            import pdb; pdb.set_trace() ## DEBUG ##
            a = a_temp
            
        p = self.p[:]
        x0 = self.x0

        R = np.array(self.rotMatrix)

        x_obs = np.zeros((self.dim,numPoints))
        
        if self.dim == 2:
            x_obs[0,:] = (a[0]*np.cos(theta)).reshape((1,-1))
            x_obs[1,:] = (np.copysign(a[1], theta)*(1 - np.cos(theta)**(2*p[0]))**(1./(2.*p[1]))).reshape((1,-1))
            print('wrong dim')
                
        else:
            # TODO --- next line probs wrong. Power of zero...
            x_obs[0,:] = (a[0]*np.cos(phi)*np.cos(theta)).reshape((1,-1))
            x_obs[1,:] = (a[1]*np.copysign(1., theta)*np.cos(phi)*(1. - np.cos(theta)**(2.*p[0]))**(1./(2.*p[1]))).reshape((1,-1))
            x_obs[2,:] = (a[2]*np.copysign(1,phi)*(1. - (np.copysign(1,theta)*np.cos(phi)*(1. - 0. ** (2*p[2]) - np.cos(theta)**(2.*p[0]))**(1./(2.**p[1])))**(2.*p[1]) - (np.cos(phi)*np.cos(theta)) ** (2.*p[0])) ** (1./(2.*p[2])) ).reshape((1,-1))

        # TODO for outside function - only sf is returned, remove x_obs to speed up
        x_obs_sf = np.zeros((self.dim,numPoints))
        if hasattr(self, 'sf'):
            if type(self.sf) == int or type(self.sf) == float:
                x_obs_sf = R.dot(x_obs*self.sf) + np.tile(x0,(numPoints,1)).T
            else:
                x_obs_sf = R.dot(x_obs*np.tile(self.sf,(1,numPoints))) + np.tile(x0, (numPoints,1)).T 
        else:
            x_obs_sf = R.dot(fx_obs) + np.tile(x0,(1,numPoints))

        # TODO uncomment
        x_obs = R.dot(x_obs) + np.tile(np.array([x0]).T,(1,numPoints))


        if sum(a_temp) == 0:
            self.x_obs = x_obs.T.tolist()
            self.x_obs_sf = x_obs_sf
        else:
             return x_obs_sf
        
        #self.x_obs_sf = R @x_obs_sf.T.tolist()
        
def talker():
    return 07
 
if __name__ == '__main__':
    try:
        a=[0.70,.11,.50]
        #th_r=[-12.,0.,-55.] # In degrees
        x0_hat=[-.28,-0.275,0.0] # In 
         #x0_hat=[-0.0,0,-00] # In degrees
        th_r=[0.,0.,0] # In degrees
        p=[10.,1.,2.]

        th_r=[th_r[i]/180.*pi for i in range(3)]
        
        obstaclePublisher(a=a, th_r=th_r, p=p, init_disp=x0_hat)
    except rospy.ROSInterruptException:
        pass
 

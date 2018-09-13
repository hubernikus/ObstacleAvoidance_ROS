#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''
# Custom libraries
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

class ObstaclesPublisher():
    def __init__(self):
        a=[0.34,0.17,0.17]
        #th_r=[-12.,0.,-55.] # In degrees
        x0_hat=[0,0,0] # In degrees
        th_r=[100.,5.,24] # In degrees
        p=[3.,1.,1.]
        th_r=[th_r[i]/180.*pi for i in range(3)]

        self.createObstacle(a=a,x0=x0_hat, th_r=th_r, p=p)

        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(20) # Frequency

        # Create publishers
        elli_pub = rospy.Publisher('ellipse0_out', PolygonStamped, queue_size=5)
        obs_pub = rospy.Publisher('obstacle0', Obstacle, queue_size=5)
        br_obs_pub = tf.TransformBroadcaster()
        
        # Create listener
        pose_sub = rospy.Subscriber("object_0/pose", PoseStamped, self.callback)

        # Create listener
        pose_sub = rospy.Subscriber("object_0/pose", PoseStamped, self.callback)

        while not rospy.is_shutdown():
            obstacle = Obstacle()
            obstacle.header = ell_poly.header

            # Time varying parameters - constant because staying in body reference frame
            obstacle.x0 = self.x0_rel 
            obstacle.th_r = self.th_r_rel

            # Dynamics properties
            #print('publisher xd', self.dx)
            obstacle.xd = self.dx
            #print('publisher w', self.w)
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
                print("Another {} obstacle2 published".format(N_pubInterval))
                self.itCount=0
        
    def createObstacle(self, a=[1,1,1], p=[1,1,1], x0=[0,0,0], th_r=[0,0,0], sf=1, sigma=1, dx=[0,0,0], w=[0,0,0], init_disp=[0,0,0])

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

        self.x0 = self.rotMatrix.T.dot(np.array(init_disp)) + np.array(x0)
        
        # Relative postition w.r.t. body reference frame (BRF)
        self.x0_rel = self.rotMatrix.T.dot(np.array(init_disp)) + np.array(x0)

        self.itCount = 0
        # Enter main loop

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
        
def talker():
    return 0
 
if __name__ == '__main__':
    try:
        ObstaclesPublisher()
    except rospy.ROSInterruptException:
        pass
 

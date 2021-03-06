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
from obstacle_setUp import * # ipmort 

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Pose, Point
from obstacle_recognition.msg import Obstacle
from matplotlib.patches import Polygon

import numpy as np
from math import pi, floor

import warnings

class obstaclePublisher():
    def __init__(self, setUp='conveyerBelt_basket'):
        
        # Initialize variables -- currently constant obstacle geometry
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(10)    # Frequency

        # Create publishers
        obs_pub = []
        for nn in range(len(obs)):
            obs_pub.append(rospy.Publisher(obs[nn].name + '/base_link', PolygonStamped, queue_size=5) )

        itCount = 0
        #while False:
        while not rospy.is_shutdown():

            for nn in range(len(obs)):
                # draw obstacles
                obs[nn].draw_ellipsoid()

                # Obstacle for algorithm
                obs_poly = PolygonStamped()
                obs_poly.header.stamp = rospy.Time.now()
                obs_poly.header.frame_id = obs[nn].frame_id

                # Publish new frame to new frame
                obs_poly.polygon.points = [Point32(obs[nn].x_obs[i][0], obs[nn].x_obs[i][1], obs[nn].x_obs[i][2]) for i in range(len(obs[nn].x_obs))]
                obs_pub[nn].publish(obs_poly)

            rate.sleep()  # Wait zzzz*
            
            itCount += 1
        # N_pubInterval = 100

if __name__ == '__main__':
    try:
        # obstaclePublisher('conveyerBelt_basket')
        obstaclePublisher('conveyerBelt_basket')
        
    except rospy.ROSInterruptException:
        pass
 

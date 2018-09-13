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

from lib_modulation import *
from lib_ds import *
from obs_common_section import *
from class_obstacle import *

# ROS tools    
import rospy

from obstacle_recognition.msg import Obstacle # Custom message
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Vector3, Twist, PointStamped
from nav_msgs.msg import Path

import tf

# MATH 
import numpy as np
from math import pi, floor

import copy # copying of lists


class TrajectoryPlanner():
    def __init__(self):
        self.n_obs = 2

        print('Start node')
        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(200) # Frequency
        rospy.on_shutdown(self.shutdown)
        
        # Create publishers
        self.pub_vel = rospy.Publisher('lwr/joint_controllers/passive_ds_command_vel', Twist, queue_size=5)
        pub_orient = rospy.Publisher('lwr/joint_controllers/passive_ds_command_orient', Quaternion, queue_size=5)
        pub_cent = [0]*self.n_obs
        pub_cent[0] = rospy.Publisher("obs0_cent", PointStamped, queue_size=5)
        pub_cent[1] = rospy.Publisher("obs1_cent", PointStamped, queue_size=5)
        
        # Create listener
        pose_sub = [0]*self.n_obs
        pose_sub[0] = rospy.Subscriber("obstacle0", Obstacle, self.callback_obs0)
        pose_sub[1] = rospy.Subscriber("obstacle1", Obstacle, self.callback_obs1)

        basket_sub = rospy.Subscriber("basket", Obstacle, self.callback_basket)
        convey_sub = rospy.Subscriber("convey", Obstacle, self.callback_convey)

        attr_sub = rospy.Subscriber("attractor", PoseStamped, self.callback_attr)
        
        self.listener = tf.TransformListener() # TF listener

        self.obs = [0]*self.n_obs
        self.pos_obs=[0]*self.n_obs
        self.quat_obs=[0]*self.n_obs

        # Set watiing variables true
        print('wait obstacle')
        self.awaitObstacle = [True for i in range(self.n_obs)]
        self.awaitAttr = True
        self.obs_basket=[]        
        self.awaitBasket = True

        self.obs_convey=[]        
        self.awaitConvey = True

        while any(self.awaitObstacle):
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        while self.awaitBasket:
            print('Waiting for basket obstacle')
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        while self.awaitConvey:
            print('Waiting for conveyer obstacle')
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        while self.awaitAttr:
            rospy.sleep(0.1)

        print('got it all')

        # Get initial transformation
        rospy.loginfo('Getting Transformationss.')
        awaitingTrafo_ee=True
        awaitingTrafo_obs= [True] * self.n_obs
        awaitingTrafo_attr=True

        for i in range(self.n_obs):
            while awaitingTrafo_obs[i]: # Trafo obs1 position
                try:
                    self.pos_obs[i], self.quat_obs[i] = self.listener.lookupTransform("/world", "/object_{}/base_link".format(i), rospy.Time(0))
                
                    awaitingTrafo_obs[i]=False
                except:
                    rospy.loginfo('Waiting for obstacle {} TRAFO'.format(i))
                    rospy.sleep(0.2)  # Wait zzzz*

        while(awaitingTrafo_ee): # TRAFO robot position

            try: # Get transformation
                self.pos_rob, self.pos_rob = self.listener.lookupTransform("/world", "/lwr_7_link", rospy.Time(0))
                awaitingTrafo_ee=False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Waiting for Robot TF')
                rospy.sleep(0.2)  # Wait zzzz*
            
        #while(awaitingTrafo_attr): # Trafo obs2 position
            # try:
                # self.pos_attr, self.quat_attr = self.listener.lookupTransform("/world", "/attr/base_link", rospy.Time(0))                   
                # awaitingTrafo_attr=False
            # except:
                # rospy.loginfo("Waiting for Attractor TF")
                # rospy.sleep(0.2)  # Wait zzzz*
                
        rospy.loginfo("All TF recieved")

        self.attractor_recieved = False # Set initial value
        # Variables for trajectory prediciton
        while not rospy.is_shutdown():
            if np.sum(np.abs([self.pos_attr.x, self.pos_attr.y, self.pos_attr.z] ) ) == 0:
                # Default pos -- command with angular position in high level controller
                rate.sleep()
                if self.attractor_recieved: # called only first time
                    self.zero_vel()
                    self.attractor_recieved = False
                continue
            
            self.attractor_recieved = True # Nonzero attractor recieved
            
            
            try: # Get transformation
                self.pos_rob, self.quat_rob = self.listener.lookupTransform("/world", "/lwr_7_link", rospy.Time(0))                   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("No <</lwr_7_link>> recieved")
                #continue
            x0_lwr7 = np.array([0,0,0])+np.array(self.pos_rob)

            obs_roboFrame = copy.deepcopy(self.obs)  # TODO -- change, because only reference is created not copy...
            for n in range(len(self.obs)): # for all bostacles
                try: # Get transformation
                    self.pos_obs[n], self.quat_obs[n] = self.listener.lookupTransform("/world", "/object_{}/base_link".format(n), rospy.Time(0))                   
                except:# (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("No <<object{}>> recieved".format(n))
                    #continue

                quat_thr = tf.transformations.quaternion_from_euler(self.obs[n].th_r[0],
                                                                    self.obs[n].th_r[1],
                                                                    self.obs[n].th_r[2])
                quat_thr_mocap = tf.transformations.quaternion_multiply(self.quat_obs[n], quat_thr)

                th_r_mocap = tf.transformations.euler_from_quaternion(quat_thr_mocap)

                # TODO apply transofrm to x0
                obs_roboFrame[n].x0 = self.obs[n].x0 + np.array(self.pos_obs[n]) # Transpose into reference frame
                obs_roboFrame[n].th_r =  [th_r_mocap[0],
                                          th_r_mocap[1],
                                          th_r_mocap[2]]# Rotate into reference frame
                                          
            q0 = Quaternion(1, 0, 0, 0) # Unit quaternion for trajectory

            x = x0_lwr7 # x for trajectory creating
            x_hat =  x0_lwr7 # x fro linear DS

            obs_roboFrame[0].w = [0,0,0]
            obs_roboFrame[0].xd = [0,0,0]

            x_attr = np.array([self.pos_attr.x, self.pos_attr.y, self.pos_attr.z]) # attracotr position

            # Create obstacles class instances
            obs_list = []
            for ii in range(self.n_obs):
                obs_list.append(
                    ObstacleClass(
                        a=obs_roboFrame[ii].a,
                        p=obs_roboFrame[ii].p,
                        x0=obs_roboFrame[ii].x0,
                        th_r=obs_roboFrame[ii].th_r,
                        sf=obs_roboFrame[ii].sf
                        ) )
                obs_list[ii].draw_ellipsoid()
            
            # Get common center
            obs_common_section(obs_list)

            # Add the basket wall as an obstacle -- it it is not considered in common center and therefore added later (HACK for fast simulation..)
            self.obs_basket.center_dyn = copy.deepcopy(self.obs_basket.x0)
            obs_list.append(self.obs_basket)

            self.obs_convey.center_dyn = copy.deepcopy(self.obs_convey.x0)
            obs_list.append(self.obs_convey)

            ds_init = linearAttractor_const(x, x0=x_attr)
            #print('vel_init', np.sqrt(np.sum(ds_init**2)))
            ds_modulated = obs_avoidance_convergence(x, ds_init, obs_list)
            #print('vel', np.sqrt(np.sum(ds_modulated**2)))

            vel = Twist()
            vel.linear = Vector3(ds_modulated[0],ds_modulated[1],ds_modulated[2])
            vel.angular = Vector3(0,0,0)
            self.pub_vel.publish(vel)

            quat = Quaternion(self.quat_attr.x,
                              self.quat_attr.y,
                              self.quat_attr.z,
                              self.quat_attr.w)
            pub_orient.publish(quat)
            
            print("Velcontrol <<world>>:", ds_modulated)

            showCenter = True
            if showCenter:
                point = PointStamped()
                point.header.frame_id = 'world'
                point.header.stamp = rospy.Time.now()

                for ii in range(self.n_obs):
                    if hasattr(obs_list[ii], 'center_dyn'):
                        point.point = Point(obs_list[ii].center_dyn[0],
                                            obs_list[ii].center_dyn[1],
                                            obs_list[ii].center_dyn[2])

                    else:
                        point.point = Point(obs_list[ii].x0[0],
                                            obs_list[ii].x0[1],
                                            obs_list[ii].x0[2])
                    pub_cent[ii].publish(point)


                
            
            rate.sleep()
            
        self.shutdown()


    def shutdown(self):

        self.zero_vel()

        rospy.loginfo('Zero velocity command after shutdown.')            
    def zero_vel(self):
        vel = Twist()
        vel.linear = Vector3(0,0,0)
        vel.angular = Vector3(0,0,0)
        self.pub_vel.publish(vel)
        
    def callback_obs0(self,msg):
        i =0 # TODO add second obstacle
        if self.awaitObstacle[i]:
            rospy.loginfo('Obstacle <<{}>> recieved.'.format(i))
            self.awaitObstacle[i] = False
        self.obs[i] = msg

    def callback_obs1(self,msg):
        i =1 # TODO add second obstacle
        if self.awaitObstacle[i]:
            rospy.loginfo('Obstacle <<{}>> recieved.'.format(i))
            self.awaitObstacle[i] = False
        self.obs[i] = msg

    def callback_basket(self,msg):
        if self.awaitBasket:
            rospy.loginfo('Basket wall recieved.')
            self.awaitBasket = False
        self.obs_basket = msg
    
    def callback_convey(self,msg):
        if self.awaitConvey:
            rospy.loginfo('Basket wall recieved.')
            self.awaitConvey = False
        self.obs_convey = msg
        print(self.obs_convey)


    def callback_attr(self, msg):
        self.awaitAttr = False
        self.pos_attr = msg.pose.position
        self.quat_attr = msg.pose.orientation

        
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: 
    try:
         TrajectoryPlanner()
    except rospy.ROSInterruptException:
        pass
 

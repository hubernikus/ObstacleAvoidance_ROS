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
from class_obstacle import *
from obs_common_section import *

# ROS tools    
import rospy

from obstacle_recognition.msg import Obstacle # Custom message
from geometry_msgs.msg import Point, PointStamped,  Quaternion, Pose, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud

import tf

# MATH 
import numpy as np
from math import pi, floor

import copy # copying of lists

class TrajectoryPredictor():
    def __init__(self):
        print('Start node')
        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(2) # Frequency

        self.n_obs = 2

        # Create publishers
        pub_traj = rospy.Publisher('ds_finalTrajectory', Path, queue_size=5)
        pub_trajInit = rospy.Publisher('ds_initTrajectory', Path, queue_size=5)
        pub_pos = []
        for ii in range(self.n_obs):
            pub_pos0 = rospy.Publisher('pose_obstacle{}'.format(ii), PoseStamped, queue_size=5)
            pub_pos.append(pub_pos0)

        pub_cloud = []
        for ii in range(self.n_obs):
            pub_cloud0 = rospy.Publisher('cloud_obstacle{}'.format(ii), PointCloud , queue_size=5)
            pub_cloud.append(pub_cloud0)

        pub_cent = [0]*self.n_obs
        pub_cent[0] = rospy.Publisher("obs0_cent", PointStamped, queue_size=5)
        pub_cent[1] = rospy.Publisher("obs1_cent", PointStamped, queue_size=5)
            
        pub_attr = rospy.Publisher('pose_attr', PoseStamped, queue_size=5)

        # Collision cloud
        pub_coll = rospy.Publisher('collision_traj', PointCloud , queue_size=5)

        # Create listener
        pose_sub = [0]*self.n_obs
        pose_sub[0] = rospy.Subscriber("obstacle0", Obstacle, self.callback_obs0)
        pose_sub[1] = rospy.Subscriber("obstacle1", Obstacle, self.callback_obs1)

        basket_sub = rospy.Subscriber("basket", Obstacle, self.callback_basket)
        convey_sub = rospy.Subscriber("convey", Obstacle, self.callback_convey)

        
        #attr_sub = rospy.Subscriber("attr_obs", Obstacle, self.callback_attr)
        attr_sub = rospy.Subscriber("attractor", PoseStamped, self.callback_attr)
        
        self.listener = tf.TransformListener() # TF listener

        self.obs = [0]*self.n_obs
        self.pos_obs=[0]*self.n_obs
        self.quat_obs=[0]*self.n_obs

        rospy.loginfo('Wait For Obstacles')
        self.awaitObstacle = [True for i in range(self.n_obs)]
        self.awaitAttr = True
        
        self.obs_basket=[]        
        self.awaitBasket = True

        self.obs_convey=[]        
        self.awaitConvey = True

        while sum(self.awaitObstacle):
            print('Waiting for obstacle')
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        while self.awaitBasket:
            print('Waiting for basket obstacle')
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        while self.awaitConvey:
            print('Waiting for convey obstacle')
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        rospy.loginfo('Wait for attractor Pose')
        while self.awaitAttr:
            rospy.sleep(0.1)

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
            
        rospy.loginfo("All TF recieved")

        self.iSim = 0

        # Variables for trajectory prediciton
        self.n_intSteps = 100
        self.dt = 0.1
        self.dim = 3 #  3D space

        while not rospy.is_shutdown():
            try: # Get transformation
                self.pos_rob, self.quat_rob = self.listener.lookupTransform("/world", "/lwr_7_link", rospy.Time(0))                   
            except:
                rospy.loginfo("No <</lwr_7_link>> recieved")
                #continue
            x0_lwr7 = np.array([0,0,0])+np.array(self.pos_rob)

            x_attr = np.array([self.pos_attr.x, self.pos_attr.y, self.pos_attr.z]) # attracotr position
            
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

                pose_obs = PoseStamped() # Publish pose for verification
                pose_obs.header.stamp = rospy.Time.now()
                pose_obs.header.frame_id = '/world'
                pose_obs.pose.orientation = Quaternion(quat_thr_mocap[0],
                                                      quat_thr_mocap[1],
                                                      quat_thr_mocap[2],
                                                      quat_thr_mocap[3])
                
                pose_obs.pose.position = Point(obs_roboFrame[n].x0[0],
                                              obs_roboFrame[n].x0[1],
                                              obs_roboFrame[n].x0[2])
                obs_roboFrame[n].center_dyn = obs_roboFrame[n].x0


                
            traj = Path()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = '/world'

            trajInit = Path()
            trajInit.header = traj.header

            x = x0_lwr7 # x for trajectory creating
            x_hat =  x0_lwr7 # x fro linear DS

            q0 = Quaternion(1, 0, 0, 0) # Unit quaternion for trajectory

            # Add to trajectory
            pose = PoseStamped()
            pose.header = traj.header
            pose.pose.position = Point(x[0],x[1],x[2])
            pose.pose.orientation = q0
            traj.poses.append(pose)

            # Add initial trajectroy
            poseInit = PoseStamped()
            poseInit.header = pose.header
            poseInit.pose.orientation = q0
            poseInit.pose.position = Point(x_hat[0],x_hat[1],x_hat[2])
            trajInit.poses.append(poseInit)
            
            obs_roboFrame[0].w = [0,0,0]
            obs_roboFrame[0].xd = [0,0,0]

            # Create obstacles-class instances
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
                
            obs_common_section(obs_list)             # Get common center

            # Add the basket wall as an obstacle -- it it is not considered in common center and therefore added later (HACK for fast simulation..)
            self.obs_basket.center_dyn = copy.deepcopy(self.obs_basket.x0)
            obs_list.append(self.obs_basket)

            self.obs_convey.center_dyn = copy.deepcopy(self.obs_convey.x0)
            obs_list.append(self.obs_convey)
                            
            collisionCloud = PointCloud()
            collisionCloud.header.frame_id = 'world'
            collisionCloud.header.stamp = rospy.Time.now()

            # Obstacle avoidance loop for trjectory planing
            for iSim in range(self.n_intSteps):
                ds_init = linearAttractor_const(x, x0=x_attr)

                ds_modulated = obs_avoidance_convergence(x, ds_init, obs_list)
                x = ds_modulated*self.dt + x

                # Initial trajectory
                ds_init = linearAttractor_const(x_hat, x0=x_attr)
                x_hat = ds_init*self.dt + x_hat
                
                # Add initial trajectroy
                poseInit = PoseStamped()
                poseInit.header = pose.header
                poseInit.pose.orientation = q0
                poseInit.pose.position = Point(x_hat[0],x_hat[1],x_hat[2])
                trajInit.poses.append(poseInit)

                # Add to trajectory
                pose = PoseStamped()
                pose.header = traj.header
                pose.pose.position = Point(x[0],x[1],x[2])
                pose.pose.orientation = q0
                traj.poses.append(pose)

                # Check for collision - create collision cloud
                noCollision = obs_check_collision(x, obs_list)
                
                if not noCollision:
                    collisionCloud.points.append( pose.pose.position)
                    print('WARNING --- collision detected at x = ', [round(x[i],2) for i in range(3)])
                
            # Publish trajectories.
            pub_traj.publish(traj)
            pub_trajInit.publish(trajInit)
            pub_coll.publish(collisionCloud)
            
            print("Publishing Trajectory %s" % rospy.get_time())

            poseCheck=True
            if poseCheck:
                # Pubslish pose for testing
                pose_ob = PoseStamped()
                pose_ob.header.frame_id = 'world'
                pose_ob.header.stamp = rospy.Time.now()

                for ii in range(self.n_obs):
                    if hasattr(obs_list[ii], 'center_dyn'):
                        pose_ob.pose.position = Point(obs_list[ii].center_dyn[0],
                                                      obs_list[ii].center_dyn[1],
                                                      obs_list[ii].center_dyn[2])
                    else:
                        pose_ob.pose.position = Point(obs_list[ii].x0[0],
                                                      obs_list[ii].x0[1],
                                                      obs_list[ii].x0[2])
                        
                    quat_thr = tf.transformations.quaternion_from_euler(obs_list[ii].th_r[0],
                                                                        obs_list[ii].th_r[1],
                                                                        obs_list[ii].th_r[2])
                    
                    pose_ob.pose.orientation = Quaternion(quat_thr[0],
                                                          quat_thr[1],
                                                          quat_thr[2],
                                                          quat_thr[3])

                    pub_pos[ii].publish(pose_ob)

                    cloud_ob = PointCloud()
                    cloud_ob.header = pose_ob.header

                    for kk in range(len(obs_list[ii].x_obs) ):
                        anotherPoint = Point(obs_list[ii].x_obs[kk][0],
                                             obs_list[ii].x_obs[kk][1],
                                             obs_list[ii].x_obs[kk][2])
                        cloud_ob.points.append(anotherPoint)
                    pub_cloud[ii].publish(cloud_ob)

            # Create Obstacle Point Cloud to check coherency
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

                
            self.iSim += 1
            
            rate.sleep()
            
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

    def callback_attr(self, msg):
        self.awaitAttr = False
        self.pos_attr = msg.pose.position
        self.quat_attr = msg.pose.orientation
        #self.attractor = msg
 
try:
    TrajectoryPredictor()
except rospy.ROSInterruptException:
    pass
 

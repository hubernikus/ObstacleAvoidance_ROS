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
lib_string = "/home/lukas/catkin_ws/src/obstacle_avoidance/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)

from lib_modulation import *
from dynamicalSystem_lib import linearAttractor
from obs_common_section import *
from class_obstacle import *

# Global vairable 'obs' containing all obstacles is imported
from obstacle_setUp import * # ipmort 

# ROS tools    
import rospy

from obstacle_recognition.msg import Obstacle # Custom message
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Vector3, Twist, PointStamped
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

import tf
from tf.transformations import quaternion_multiply as quat_mult
from tf.transformations import quaternion_inverse as quat_inv
from tf.transformations import quaternion_slerp as quat_slerp
#import tf.transformations.quaternion_multiply as quat_mult

import tf2_ros
#import tf2_py

# MATH 
import numpy as np
import numpy.linalg as LA
import numpy.polynomial.polynomial as poly
from math import pi, floor

#from scipy.signal import savgol_filter # filter
from scipy.signal import *

import copy # copying of lists

q0 = Quaternion(1, 0, 0, 0) # Unit quaternion for trajectory

class TrajectoryPlanner():
    def __init__(self, dsController=False, freq=50, n_intSteps=20, dt_simu=0.1):
        # control mode is either dsController=1 or trajectory_predictor
        dsController = int(dsController)
        
        if dsController:
            controlMode = 'DS_controller'
        else:
            controlMode = 'trajectory_predictor'

        print('Start node TrajectoryPlanner.py with' + controlMode)

          # Initialize node
        rospy.init_node('TrajectoryPlanner' + '_' + controlMode, anonymous=True)
        rate = rospy.Rate(freq) # Frequency
        if dsController:
            rospy.on_shutdown(self.shutdown_vel)

        self.obs = copy.deepcopy(obs)
        # Why two copies? ... there is a second one later one.. chacnge
        self.n_obs = len(obs)
        
        self.n_obs_moving = 1 # TODO - automatically adapt to moving obstacels
        self.it_obs_moving = [0] # the obstacle indes of the moving obstacles
        self.obsRecieved = [0]*self.n_obs

        #self.N_filter = 10
        # self.obs_pose = [0]*self.N_filter

        # self.pos_obs_mov = np.zeros((3, self.n_obs_moving))
        # self.quat_obs_mov = np.zeros((4, self.n_obs_moving))

        # self.vel_obs_mov = np.zeros((3, self.n_obs_moving))
        # self.rot_obs_mov = np.zeros((3, self.n_obs_moving)) # rotational speed

        # TODO -- maybe only for acutally moving obstacles
        
        self.pos_filt = [0]*self.n_obs
        self.vel_filt = [np.zeros(3)]*self.n_obs
        self.q_filt = [0]*self.n_obs
        self.q_diff = [np.array([0,0,0,1])]*self.n_obs
        self.omega = [np.array([0,0,0])]*self.n_obs

        self.pos_filt_old = [0]*self.n_obs
        self.q_filt_old = [0]*self.n_obs

        # Filter timestep 
        self.t = [0]*self.n_obs
        self.t_old = [0]*self.n_obs
        self.dt_filt = [0.01]*self.n_obs        
        
        self.baseFrame = 'world'
        # create listener
        pose_sub = [0]*self.n_obs_moving
        pose_sub[0] = rospy.Subscriber("box/pose", PoseStamped, self.callback_obs)
        
        # Create publishers
        if dsController:
            self.pub_vel = rospy.Publisher('lwr/joint_controllers/passive_ds_command_vel', Twist, queue_size=10)
            pub_orient = rospy.Publisher('lwr/joint_controllers/passive_ds_command_orient', Quaternion, queue_size=10)
        else: # Pubslish trajetory
            pub_traj = rospy.Publisher('ds_modulatedTrajectory', Path, queue_size=2)
            pub_trajInit = rospy.Publisher('ds_initTrajectory', Path, queue_size=2)

        # Visualization publisher
        self.pub_velVis = rospy.Publisher('box/velocity', Path, queue_size=10)
        self.pub_qVis = rospy.Publisher('box/quat', Path, queue_size=10)
            
        # Attractor listener
        attr_sub = rospy.Subscriber("attractor", PoseStamped, self.callback_attr)
        
        # TF listener
        self.listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener2 = tf2_ros.TransformListener(self.tfBuffer)
        #tf2_ros.addTransformsChangedListener("/world", "/lwr_7_link", self.callback_obs0)
        


        # Set watiing variables true
        
        # while any(self.awaitObstacle):
            # rospy.sleep(0.1)  # Wait zzzz* = 1 # wait

        # Get initial transformation
        print('Getting Transformations.')
        awaitingTrafo_obs= [True] * self.n_obs

        # rospy.sleep(1)
        for i in range(self.n_obs):
            while (awaitingTrafo_obs[i]):
                if i in self.it_obs_moving:
                    awaitingTrafo_obs[i] = False
                    continue
                try:
                    pose = self.tfBuffer.lookup_transform(self.baseFrame, self.obs[i].frame_id, rospy.Time())
                    self.pos_filt[i] = np.array([pose.transform.translation.x,
                                                 pose.transform.translation.y,
                                                 pose.transform.translation.z])

                    self.q_filt[i] = np.array([pose.transform.rotation.x,
                                               pose.transform.rotation.y,
                                               pose.transform.rotation.z,
                                               pose.transform.rotation.w])

                    awaitingTrafo_obs[i] = False
                except:
                    print('Waiting2 for obstacle <<{}>> TRAFO'.format(self.obs[i].name))
                    rospy.sleep(0.2)  # Wait zzzz*

        awaitingTrafo_ee = True
        while(awaitingTrafo_ee):            
            try:
                self.pos_rob, self.quat_rob = self.listener.lookupTransform(self.baseFrame, "/lwr_7_link", rospy.Time(0))
                awaitingTrafo_ee= False
            except:
                    print('Waiting for TRAFO ee'.format(i))
                    rospy.sleep(0.2)  # Wait zzzz*
                    

        rospy.loginfo("All TF's recieved")

        # Attractor
        self.awaitAttr = True

        # TODO remove -- faking attractor
        self.awaitAttr = False
        self.pos_attr = Point(-1,+0.5,1) # TODO - attractor callback
        self.quat_attr = Quaternion(0,0,0,1)
        
        print('Wait for attractor Pose')
        while self.awaitAttr:
            rospy.sleep(0.1)
        
        # while np.sum(np.array(self.obsRecieved) < self.N_filter):
            # print(self.obsRecieved)
            # print(self.N_filter)
            # rospy.sleep(0.2)
            # print('Waiting for moving obstacle instances {}'.format(self.obsRecieved) )


        print('Enerting main loop....')
        ####################################### START MAIN LOOOP ##############
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
                self.pos_rob, self.quat_rob = self.listener.lookupTransform(self.baseFrame, "/lwr_7_link", rospy.Time(0))                   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("No <</lwr_7_link>> recieved")
                #continue
            x0_lwr7 = np.array([0,0,0])+np.array(self.pos_rob)

            obs_roboFrame = copy.deepcopy(self.obs)  # TODO -- change, because only reference is created not copy...
            for n in range(len(self.obs)):     # for all bostacles
                #### Evaluate moving obstacles velocities through spline fitting
                if n in self.it_obs_moving:
                    it_mov = np.where(np.array(self.it_obs_moving)==n)
                    

                    # # Pos array
                    # pos = np.zeros((3, self.N_filter))
                    # quat = np.zeros((4, self.N_filter))
                    # t = np.zeros((self.N_filter))

                    # for ii in range(self.N_filter):
                    #     pos[0,ii] = self.obs_pose[ii].transform.translation.x
                    #     pos[1,ii] = self.obs_pose[ii].transform.translation.y
                    #     pos[2,ii] = self.obs_pose[ii].transform.translation.z

                    #     quat[0,ii] = self.obs_pose[ii].transform.rotation.x
                    #     quat[1,ii] = self.obs_pose[ii].transform.rotation.y
                    #     quat[2,ii] = self.obs_pose[ii].transform.rotation.z
                    #     quat[3,ii] = self.obs_pose[ii].transform.rotation.w
                        
                    #     t[ii] = rospy.Time(self.obs_pose[ii].header.stamp.secs, self.obs_pose[ii]+.header.stamp.nsecs).to_sec()

                    #     # assume high update rate, relatively slow changes in motion -> 2nd order polynomial
                    #     # todo - change filter strategy (maybe)
                    #     degpol_p = 1    #
                    #     degPol_q = 2    # 

                    #     polyPos = poly.polyfit(t, pos.T, deg=degPol_p)
                    #     polyQ = poly.polyfit(t, quat.T, deg=degPol_q)

                    #     d_polyPos = np.zeros((degPol_p, 3))    # derivative polynomial
                    #     for dd in range(degPol_p):
                    #         d_polyPos[dd,:] = polyPos[dd+1,:]*(dd+1)

                    #     t_mean = np.mean(t)
                    #     pos_filt = poly.polyval(t_mean, polyPos)
                    #     vel_filt= poly.polyval(t_mean, d_polyPos)

                    #     dt_mean = np.mean(t[1:]-t[:-1])
                    #     q_filt = poly.polyval(t_mean, polyQ)

                    #     q1_filt = q[:,self.N_filter/2-1]
                    #     q2_filt = q[:,self.N_filter/2]
                        
                    #     for ii in range(self.N_filter):
                    #         frac = (t[self.N_filter/2-1]-t[ii])/(t[-2-ii]-t[ii])
                    #         q1_filt = q1_filt + quat_slerp(quat[:,ii], quat[:,-2-ii], frac)

                    #         frac = (t[self.N_filter/2]-t[ii+1])/(t[-1-ii]-t[ii+1])
                    #         q2_filt = q2_filt + quat_slerp(quat[:,ii+1], quat[:,-1-ii], frac)*

                    #     q1_filt = q1_filt/self.N_filter*2
                    #     q1_filt = q1_filt / LA.norm(q1_filt)
                    #     q2_filt = q1_filt/self.N_filter*2
                    #     q2_filt = q2_filt / LA.norm(q2_filt)

                    #     q_filt =  quat_slerp(q1_filt, q2_filt, 0.5)
                    #     # Quaternion just before q mean and after
                        
                    #     d_q_filt = quat_mult(q2_filt, quat_inv(q1_filt))
                                             #/(sefl.N_filter/2]-t[ii])
                        
                    #dq_filt = quat_mult(self.q_filt[ii], quat_inv(self.q_filt_old[ii]))
                    # print('n', n)
                    # print('diff', self.q_diff[n])
                    # print('filt', self.q_filt[n])
                    # print('dt', self.dt_filt[n])

                    print('norm qDIFF', LA.norm(self.q_diff[n]))
                    print('norm qfilt', LA.norm(self.q_filt[n]))
                    omega = (2*quat_exp(quat_log(self.q_diff[n])/self.dt_filt[n])
                                     * quat_inv(self.q_filt[n]))
                    print('omega 4', LA.norm(omega))
                    self.omega[n] = omega[0:3]     # quaternion to vector
                    print('omega 3', LA.norm(self.omega[n]))
                    
                quat_thr = tf.transformations.quaternion_from_euler(self.obs[n].th_r[0],
                                                                    self.obs[n].th_r[1],
                                                                    self.obs[n].th_r[2])

                quat_thr_mocap = tf.transformations.quaternion_multiply(self.q_filt[n], quat_thr)

                th_r_mocap = tf.transformations.euler_from_quaternion(quat_thr_mocap)

                # TODO apply transofrm to x0 ?!
                obs_roboFrame[n].x0 = self.obs[n].x0 + np.array(self.pos_filt[n]) # Transpose into reference frame
                obs_roboFrame[n].th_r =  [th_r_mocap[0],
                                          th_r_mocap[1],
                                          th_r_mocap[2]] # Rotate into reference frame

            x = x0_lwr7 # x for trajectory creating
            x_hat =  x0_lwr7 # x fro linear DS

            obs_roboFrame[0].w = [0,0,0]
            obs_roboFrame[0].xd = [0,0,0]

            x_attr = np.array([self.pos_attr.x, self.pos_attr.y, self.pos_attr.z]) # attracotr position
            # print('x',x)

            # Get common center
            if dsController:
                ds_init = linearAttractor(x, x0=x_attr)

                # ds_modulated = obs_avoidance_convergence(x, ds_init, obs_list)
                ds_modulated = obs_avoidance_interpolation(x, ds_init, obs_roboFrame, attractor=x_attr)

                vel = Twist()
                vel.linear = Vector3(ds_modulated[0],ds_modulated[1],ds_modulated[2])
                vel.angular = Vector3(0,0,0)
                self.pub_vel.publish(vel)

                quat = Quaternion(self.quat_attr.x,
                                  self.quat_attr.y,
                                  self.quat_attr.z,
                                  self.quat_attr.w)

                pub_orient.publish(quat)
            
                print("Velcontrol <<world>> is ", ds_modulated)
            else: # trajectory controller
                traj = Path()
                traj.header.stamp = rospy.Time.now()
                traj.header.frame_id = '/world'

                trajInit = Path()
                trajInit.header = traj.header

                collisionCloud = PointCloud()
                collisionCloud.header.frame_id = 'world'
                collisionCloud.header.stamp = rospy.Time.now()

            #     point.header.stamp = rospy.Time.now()
                for iSim in range(n_intSteps):
                    ds_init = linearAttractor(x, x0=x_attr)

                    # ds_modulated = obs_avoidance_convergence(x, ds_init, obs_roboFrame)
                    ds_modulated = obs_avoidance_interpolation(x, ds_init, obs_roboFrame, attractor=x_attr)                    
                    x = x + ds_modulated*dt_simu

                    # Initial trajectory
                    ds_init = linearAttractor(x_hat, x0=x_attr)
                    x_hat = ds_init*dt_simu + x_hat

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

                    # Check for collision - create collision cloud
                    noCollision = obs_check_collision(x, obs_roboFrame)
                    
                    #if not noCollision:
                    if False:
                        collisionCloud.points.append(pose.pose.position)
                        print('WARNING --- collision detected at x = ', [round(x[i],2) for i in range(3)])
                print("Trajectory Publsihed.")
                # Publish trajectories.
                pub_traj.publish(traj)
                pub_trajInit.publish(trajInit)
                
                #rospy.spin_once()

            nn = 0
            ##### Visulationzation of moving obstacles
            vel0 = PoseStamped()
            vel0.header.stamp = rospy.Time.now()
            vel0.header.frame_id = '/world'

            vel0.pose.position.x = self.pos_filt[nn][0]
            vel0.pose.position.y = self.pos_filt[nn][1]
            vel0.pose.position.z = self.pos_filt[nn][2]

            vel0.pose.orientation = q0

            velTraj = Path()
            velTraj.header = vel0.header
            velTraj.poses.append(vel0)

            mag = 1
            vel1 = PoseStamped()
            vel1.header = vel0.header
            vel1.pose.orientation = vel0.pose.orientation

            vel1.pose.position.x = self.pos_filt[nn][0] + mag*self.vel_filt[nn][0]
            vel1.pose.position.y = self.pos_filt[nn][1] + mag*self.vel_filt[nn][1]
            vel1.pose.position.z = self.pos_filt[nn][2] + mag*self.vel_filt[nn][2]

            velTraj.poses.append(vel1)

            self.pub_velVis.publish(velTraj)

            mag=10
            omegaTraj = velTraj
            omegaTraj.poses[1].pose.position.x = self.pos_filt[nn][0] + mag*self.omega[nn][0]
            omegaTraj.poses[1].pose.position.y = self.pos_filt[nn][1] + mag*self.omega[nn][1]
            omegaTraj.poses[1].pose.position.z = self.pos_filt[nn][2] + mag*self.omega[nn][2]
            
            self.pub_qVis.publish(omegaTraj)
            
            rate.sleep()
            
        self.shutdown_vel()


    ##### Member Functions ####
    def shutdown_vel(self,):
        self.zero_vel()
        rospy.loginfo('Zero velocity command after shutdown.')


    def zero_vel(self):
        vel = Twist()
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, 0)
        self.pub_vel.publish(vel)

   
    def callback_obs(self, msg, ii=0, ii_mov=0): # index out of moving and general obstacles
        try:   # Get transformation
            new_pose = self.tfBuffer.lookup_transform(self.baseFrame, obs[ii].frame_id, rospy.Time(0))

            #if self.obsRecieved[ii_mov]:
                # t_new = rospy.Time(new_tf.header.stamp.secs, self.obs_pose.header.stamp.nsecs)
                # t_old = rospy.Time(self.obs_pose[-1].header.stamp.secs, self.obs_pose[-1].header.stamp.nsecs)
            # if t_new.to_sec() > t_old.to_sec():
            
            # for ii in range(len(self.obs_pose)-1):    
                # self.obs_pose[ii] = self.obs_pose[ii+1]
                # self.obs_pose[-1] = new_

            # if self.obsRecieved[ii_mov] < self.N_filter:
                # self.obsRecieved[ii_mov] += 1

            new_pos = np.array([new_pose.transform.translation.x,
                                new_pose.transform.translation.y,
                                new_pose.transform.translation.z])

            new_quat = np.array([new_pose.transform.rotation.x,
                                 new_pose.transform.rotation.y,
                                 new_pose.transform.rotation.z,
                                 new_pose.transform.rotation.w])

            # Define filter factors
            if self.obsRecieved[ii]==0:
                # Initialize first values
                self.pos_filt[ii] = new_pos
                self.q_filt[ii] = new_quat
                self.t[ii] = rospy.Time(new_pose.header.stamp.secs, new_pose.header.stamp.nsecs).to_sec()
                self.obsRecieved[ii]=0.5
                return
                
            elif self.obsRecieved[ii]==0.5:            
                factor_pos = 1
                factor_time = 1
                factor_time = 1
                self.obsRecieved[ii]=1
            else:
                factor_pos = 0.05
                factor_time = 0.05
                factor_q = 0.05
            
            self.pos_filt_old[ii] = self.pos_filt[ii]
            self.q_filt_old[ii] = self.q_filt[ii]
            self.t_old[ii] = self.t[ii]
                
            # Filter time, too. for the evaluation of velocity etc.
            self.t[ii] = rospy.Time(new_pose.header.stamp.secs, new_pose.header.stamp.nsecs).to_sec()
            self.dt_filt[ii] = (1-factor_time)*self.dt_filt[ii] + (factor_time)*(self.t[ii]-self.t_old[ii])
            # TODO - more ellaborate filter than just low-pass on position; e.g. kalman.
            # Assumption -- const velocity 

            self.pos_filt[ii] = ((1-factor_pos)*(self.pos_filt[ii]+self.dt_filt[ii]*self.vel_filt[ii]) + (factor_pos)*(new_pos))
            self.vel_filt[ii] = (self.pos_filt[ii]-self.pos_filt_old[ii])/self.dt_filt[ii]
            
            # print(LA.norm(self.vel_filt[ii]) )
            
            #self.dq_filt[ii] = np.exp( np.log(self.q_diff)/self.dt[ii])
            self.q_filt[ii] = quat_slerp(self.q_filt[ii]*self.q_diff[ii], new_quat, factor_q)
            self.q_diff[ii] = self.q_filt[ii]*quat_inv(self.q_filt_old[ii])
            #print('Got new obstacle --- TODO remove')
        except:   # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No object <<{}>> recieved".format(self.obs[ii].name))

    def callback_attr(self, msg): # Attractor which is lead by other script
        self.awaitAttr = False
        self.pos_attr = msg.pose.position
        self.quat_attr = msg.pose.orientation

    def kalmanFilter(self, msg, ii_obs=1, ii_mov=0):
        print('Create kalman filter, or similar for future applications!')
        print('Also observe other possible filters.0')
        
def quat_log(q):
    return np.hstack(( np.log(LA.norm(q)),
                       q[:3]/LA.norm(q[:3])*np.arccos(q[0]/LA.norm(q)) ))

def quat_exp(q):
    return np.exp(q[0])*np.hstack(( np.cos(LA.norm(q[:3])),
                                    q[:3]/LA.norm(q[:3])*np.sin(LA.norm(q[:3])) ))

        
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("!REMARK! --- possible usage: trajectory_planner.py arg1")
        TrajectoryPlanner()
    else:
        TrajectoryPlanner(sys.argv[1])

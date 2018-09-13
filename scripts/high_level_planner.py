#!/usr/bin/env python
'''
High level planner

@author lukashuber
@date 2018-06-15

'''

from math import pi
import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from obstacle_recognition.msg import Obstacle # Custom message
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
# Gripper module
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg


import tf

class HighLevelController():
    def __init__(self):

        print('Start node')
        
        # Introduce variables to make sure at least one message is recieved
        #self.awaitJointPos=True # Get first attractor obstacle
        self.awaitEE=True
        self.awaitingTrafo_ee=True

        # Pickup positions
        #self.pos_pickup = [[-0.060, 0.70, 0.34],
                            #[-0.618,-0.267,0.214]]
        self.pos_pickup = [[-0.44, 0.61, 0.34]]
                            

        self.pos_dropoff = [[-.272,-0.420, 0.30],
                           [-0.618,-0.267,0.214]]

        self.quat_dropoff = [0,1,0,0]

        # Resting position & position between drop-off and pick up
        self.restEul = [0,0,0]

        self.restPos = [-0.5,0,0.6]

        # z direction  world Frame
        self.zDir_des = [0,0,-1]

        #self.pickQuat = [0,1,0,0] # Pickup orientation
        
        # Normalize
        zDir_des_norm = np.linalg.norm(self.zDir_des)
        if zDir_des_norm: # normalize if non zerovalue
            self.zDir_des = np.array(self.zDir_des)/zDir_des_norm

        # Resting quaternion
        #self.restQuat = [1,0,0,0]
        
        #print(tf.transformations.quaternion_matrix(self.restQuat))
        #self.joinState_init = [0,0,0,120*pi/180,0,60*pi/180,0]

        self.n_joints = 7
                
        # Initialize node
        rospy.init_node('attractor_publisher', anonymous=True)
        rate = rospy.Rate(10) # Frequency
        rospy.on_shutdown(self.end_mode)

        self.pub_attr = rospy.Publisher('attractor', PoseStamped, queue_size=5)
        self.pub_grip = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10) # Gripper publisher
        # self.pub_joint = rospy.Publisher('lwr/joint_controllers/command_joint_pos', Float64MultiArray, queue_size=10)

        # subscriber
        self.sub_joint = rospy.Subscriber("lwr/ee_pose", Pose, self.callback_joint)

        while self.awaitEE:
            rospy.loginfo('Waiting for EE-Pose')
            rospy.sleep(0.1)

        self.listener = tf.TransformListener() # TF listener
        
        while(self.awaitingTrafo_ee): # TRAFO robot position
            try: # Get transformation
                self.pos_rob, self.pos_rob = self.listener.lookupTransform("/world", "/lwr_7_link", rospy.Time(0))
                self.awaitingTrafo_ee=False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Waiting for Robot-EE Tranformation.')
                rospy.sleep(0.2)  # Wait zzzz*

        
        self.grippingDuration = 1 # Gripping during 0.5 s
        self.t_grippingStart = -1
        self.dist_startGripping = 0.08 # Distance at which gripping starts
        self.deltaJoint_err = 0.003
        
        self.controlMode = 'default' # initial control mode
        #self.controlMode = 'dropoff' # initial control mode
        self.activateGripper = False
        
        self.dist_rangeMax = 1.


        self.it_pick = -1

        self.gripCommand = outputMsg.SModel_robot_output(); # Gripper command

        # set gripper init command -- Initalize gripper
        self.command = outputMsg.SModel_robot_output();
        self.command.rACT = 1 # ?? -- rAct=0 (reset) // rAct=1 (activate)
        self.command.rGTO = 1 # ??
        self.command.rSPA = 255 # gripper speed (0-255)
        self.command.rFRA = 50 # gripper forcee (0-255)
        self.command.rMOD = 1 # resolution of control
        self.command.rPRA = 255 # gripper position - open
        
        rospy.loginfo("Entering High Level Control Loop.")
        while not rospy.is_shutdown():
            try: # Get transformation
                self.pos_rob, self.quat_rob = self.listener.lookupTransform("/world", "/lwr_7_link", rospy.Time(0))                   
            except:
                rospy.loginfo("No <</lwr_7_link>> recieved")

            ##################### DEFAULT MODE ####################
            if self.controlMode=='default': 
                self.mode_default()

                distGoal = np.sqrt(np.sum((np.array(self.pos_rob)-np.array(self.restPos) ) **2))
                #deltaJoint = 0
                if distGoal < self.dist_startGripping:
                    if len(self.pos_pickup) > 1+self.it_pick:
                        #self.it_pick = self.it_pick + 1 # INFINITE LOOP WITH SAME POSITION IF ITERATION OFF
                        self.it_pick = max(self.it_pick, 0)
                        print('Going for obstacle {}'.format(1+self.it_pick) ) 
                        self.controlMode='pickup_xy'
                        print('Preparing pickup')
                        self.send_gripperCommand('open') # check and send gripper command
                        # self.pub_joint.publish(Float64MultiArray()) # zero joint command
                    else:
                        print('Mission complete - shutting down.')
                        rospy.signal_shutdown('Mission complete.')


            ##################### PICKUP MODE -- set only x and y ####################
            if self.controlMode=='pickup_xy': #higher up
                self.mode_default()

                dZ = 0.15
                pos_startPick = [self.pos_pickup[self.it_pick][0],
                                 self.pos_pickup[self.it_pick][1],
                                 self.pos_pickup[self.it_pick][2]+dZ]

                self.mode_pickup(dZ)
                
                distGoal = np.sqrt(np.sum((np.array(self.pos_rob)-np.array(pos_startPick) ) **2))
                #deltaJoint = 0
                if distGoal < self.dist_startGripping:
                    self.controlMode='pickup'
                    print('Starting pickup.')
                        # self.pub_joint.publish(Float64MultiArray()) # zero joint command


            #################### PICKUP MODE -- set x, y AND z ####################
            elif self.controlMode=='pickup':
                #print('now pickup mode')

                self.mode_pickup()

                if not self.activateGripper:
                    #self.it_pick = 0 # could be changed for different positions
                    distGoal = np.sqrt(np.sum((self.pos_rob-np.array(self.pos_pickup[self.it_pick]))**2))
                    self.send_gripperCommand('open') # check and send gripper command\
                    if distGoal < self.dist_startGripping:
                        self.activateGripper=True
                        self.t_grippingStart = rospy.Time.now().to_sec()
                else: # Gripper activated
                    if self.activateGripper:
                        dt_gripping = rospy.Time.now().to_sec() - self.t_grippingStart
                    self.send_gripperCommand('close') # check and send gripper command
                    if dt_gripping > self.grippingDuration:
                        #self.controlMode = 'dropoff_xy'
                        self.controlMode = 'dropoff'
                        print('Preparing dropoff.')
                
                        self.activateGripper = False
                        dt_gripping = 0

            ##################### MODE PRE DROPOFF -- set only x and y ####################
            if self.controlMode=='dropoff_xy': #higher up
                
                dZ = 0.2 # DELTA Z
                pos_goal = np.array(self.pos_dropoff[self.it_pick]) + np.array([0,0,dZ])
                self.mode_dropoff(pos_goal)

                distGoal = np.sqrt(np.sum((self.pos_rob-np.array(pos_goal))**2 ))
                if distGoal < self.dist_startGripping:
                    self.controlMode='dropoff'
                    print('Dropping off now..')   



            #################### DROP OFF ####################
            elif self.controlMode=='dropoff':
                self.attrObs_quat = self.quat_dropoff

                # Check if attractor in in range
                # if np.sqrt(np.sum(np.array(self.pos_attr)**2)) > self.dist_rangeMax:
                    # print('WARNING -- Drop-off location too far way. I go home.')
                    # self.mode_dropoff(self.restPos) # TODO uncomment for test
                    #self.mode_dropoff(pos_goal)
                # else:
                    # self.mode_dropoff(pos_goal)

                pos_goal = np.array(self.pos_dropoff[self.it_pick])
                self.mode_dropoff(pos_goal)
                     

                if not self.activateGripper:
                    distGoal = np.sqrt(np.sum((self.pos_rob-np.array(pos_goal))**2 ))
                    self.send_gripperCommand('close') # check and send gripper command

                    if distGoal < self.dist_startGripping:
                        self.activateGripper=True
                        self.t_grippingStart = rospy.Time.now().to_sec()
                else: # Gripper activated
                    if self.activateGripper:
                        dt_gripping = rospy.Time.now().to_sec() - self.t_grippingStart

                    self.send_gripperCommand('open') # check and send gripper command

                    if dt_gripping > self.grippingDuration:
                        #self.controlMode = 'dropoff_post'  # TODO uncomment for test
                        #self.controlMode = 'default'  # TODO uncomment for test
                        self.controlMode = 'pickup_xy'  # TODO uncomment for test
                        self.activateGripper = False
                        print('Entering default mode!')
                        dt_gripping = 0
                        
            ##################### MODE POST-DROPOFF -- set only x and y ####################
            if self.controlMode=='dropoff_post': #higher up
                
                self.attrObs_quat = self.quat_dropoff
                
                dZ = 0.2 # DELTA Z
                pos_goal = np.array(self.pos_dropoff[self.it_pick]) + np.array([0,0,dZ])
                self.mode_dropoff(pos_goal)

                distGoal = np.sqrt(np.sum((self.pos_rob-np.array(pos_goal))**2 ))
                if distGoal < self.dist_startGripping:
                    self.controlMode='default'
                    print('Dropping off finished..')   



            rate.sleep()

    def end_mode(self):
        self.mode_default()
        rospy.loginfo('Node is killed.')
        
    def mode_default(self):

        attr = PoseStamped()
        attr.header.stamp = rospy.Time.now()
        attr.header.frame_id = 'world'

        attr.pose.position = Point(self.restPos[0], self.restPos[1], self.restPos[2])
        attr.pose.orientation = Quaternion(self.restQuat[0], self.restQuat[1], self.restQuat[2], self.restQuat[3])
        # attr.pose.position = Point(0,0,0)
        # attr.pose.orientation = Quaternion(0,0,0,1)
        self.pub_attr.publish(attr)

        #joint_desired = Float64MultiArray()
        # joint_desired.layout.dim = []
        # joint_desired.data = self.joinState_init
        # self.pub_joint.publish(joint_desired)


    def mode_pickup(self, deltaZ=0):
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        attr = PoseStamped()
        attr.header.stamp = rospy.Time.now()
        attr.header.frame_id = 'world'

        attr.pose.position = Point(self.pos_pickup[self.it_pick][0],
                                   self.pos_pickup[self.it_pick][1],
                                   self.pos_pickup[self.it_pick][2]+deltaZ)
        
        self.pickQuat = self.restQuat # TODO remove if desired
        attr.pose.orientation = Quaternion(self.pickQuat[0], self.pickQuat[1],self.pickQuat[2],self.pickQuat[3])
        self.pub_attr.publish(attr)

        
        #self.pub_joint.publish(Float64MultiArray()) # zero jiont command
        
    def mode_dropoff(self, pos_goal):
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        attr = PoseStamped()
        attr.header.stamp = rospy.Time.now()
        attr.header.frame_id = 'world'

        #attr.pose.position = Point(self.pos_attr[0],self.pos_attr[1],self.pos_attr[2])
        attr.pose.position = Point(pos_goal[0], pos_goal[1], pos_goal[2])
            #self.pos_attr[0],self.pos_attr[1],self.pos_attr[2])
        attr.pose.orientation = Quaternion(self.pickQuat[0], self.pickQuat[1],self.pickQuat[2],self.pickQuat[3])
        self.pub_attr.publish(attr)
        
        #self.pub_joint.publish(Float64MultiArray()) # zero jiont command
        

    def send_gripperCommand(self, pos):
        if pos=='open':
            self.command.rPRA = 0
        elif pos=='close':
            self.command.rPRA = 170

        # Check position range
        #self.command.rPRA = int(char) 
        if self.command.rPRA > 255:
            self.command.rPRA = 255
        elif self.command.rPRA < 0:
            self.command.rPRA = 0

        self.pub_grip.publish(self.command)
            
        # TODO also check other commands if being changed... e.g. force

    def callback_jointStates(self, msg):
        self.awaitJointPos = False
        self.jointState = msg.position[:self.n_joints]

    def callback_joint(self, msg):
        ee_quat = [msg.orientation.x,
                   msg.orientation.y,
                   msg.orientation.z,
                   msg.orientation.w]
        
        ee_posMatrix = tf.transformations.quaternion_matrix(ee_quat)

        # Rodirgues' rotation formula to align z-ee with z-des
        z_ee = ee_posMatrix[:3,2]

        k = np.cross(z_ee, self.zDir_des )
        normK = np.linalg.norm(k)
        if normK: # Nonzero value
            k = k/normK

        sinTheta = normK
        cosTheta = np.dot(z_ee, self.zDir_des)
        theta = np.copysign(np.arccos(cosTheta), sinTheta)

        # Quaternion -- axis, angle representation
        delta_quat = tf.transformations.quaternion_about_axis(theta, k)

        #z_new = v*cosTheta + np.cross(k, )*sinTheta +  k*np.dot(k, )*(1-cosTheta)
        #self.restQuat = tf.transformations.quaternion_multiply(ee_quat, delta_quat)

        # TODO - remove
        #self.restQuat = tf.transformations.quaternion_from_euler(0,-pi,0) # REMOVE..? Maybe?
        self.restQuat = [0,1,0,0]
        

        self.awaitEE = False # Continue initialization

def qv_mult(q1, v1):
    # Quaternion Vector Mulitplication
    # Numpy arrays...
    #v1 = tf.transformations.unit_vector(v1)
    #nq2 = list(v1)

    v1 = np.hstack((v1,0))

    return tf.transformations.quaternion_multpily(
        tf.transformations.quaternion_multiply(q1, v1), 
        tf.transformations.quaternion_conjugate(q1)
        )[:3]

                
if __name__ == '__main__':
    try:
        HighLevelController()
    except rospy.ROSInterruptException:
        pass

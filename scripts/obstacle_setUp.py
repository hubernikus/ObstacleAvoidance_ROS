#!/usr/bin/env python

from class_obstacle import * # rotation matrix

#setUp = 'conveyerBelt_basket'
setUp = 'box_only'


# def defineObstacles(setUp = 'conveyerBelt_basket'):
if setUp == 'box_only':
    xd_0=[0,0,0] # Initial velocity
    w_0=[0,0,0]  # Initial angular velocity
    
    # BIG BOX
    obs=[]
    # name ='stacked_boxes'
    # a=[0.25,0.25,0.25]
    # p=[2.,2.,2.]
    
    # x0=[0,0,-0.2] # relative to coordinate frame
    # th_r=[0.,0.,0] # In degrees

    # th_r=[th_r[i]/180.*pi for i in range(3)]

    # frame_id ='box/base_link'
    # #frame_id = 'world'
    # obs.append(
    #     Obstacle(a=a, th_r=th_r, p=p, x0=x0, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )
 
    # stacked_boxes
    name ='box'
    a=[0.36,0.21,0.21]
    #th_r=[-12.,0.,-55.] # In degrees
    x0_hat=[0,0,0] # In degrees
    th_r=[100.,5.,24] # In degrees
    #th_r=[0.,10,90] # In degrees
    p=[3.,1.,1.]
    th_r=[th_r[i]/180.*pi for i in range(3)]

    frame_id = 'world'
    obs.append(
        Obstacle(a=a, th_r=th_r, p=p, x0=x0_hat, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )

    print('Obstacles  \n')
    
if setUp == 'conveyerBelt_basket':
    xd_0=[0,0,0]
    w_0=[0,0,0]
    
    # First arm
    obs=[]
    a=[0.36,0.21,0.21]
    #th_r=[-12.,0.,-55.] # In degrees
    x0_hat=[0,0,0] # In degrees
    th_r=[100.,5.,24] # In degrees
    #th_r=[0.,10,90] # In degrees
    p=[3.,1.,1.]
    th_r=[th_r[i]/180.*pi for i in range(3)]

    name ='arm1'
    frame_id = 'world'

    obs.append(
        Obstacle(a=a, th_r=th_r, p=p, x0=x0_hat, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )

    # Second arm
    a=[0.35,0.21,0.21]
    #a=[0.3,0.10,0.10]
    #th_r=[-12.,0.,-55.] # In degrees
    x0_hat=[0,0,0] # In degrees
    th_r=[0.,-12.,95] # In degrees
    p=[3.,1.,1.]

    name ='arm2'
    frame_id = 'world'

    th_r=[th_r[i]/180.*pi for i in range(3)]
    obs.append(
        Obstacle(a=a, th_r=th_r, p=p, x0=x0_hat, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )

    # Basket 
    a=[0.70,.11,.50]
    #th_r=[-12.,0.,-55.] # In degrees
    x0_hat=[-.28,-0.275,0.0] # In 
     #x0_hat=[-0.0,0,-00] # In degrees
    th_r=[0.,0.,0] # In degrees
    p=[10.,1.,2.]

    name ='basket'
    frame_id = 'world'

    th_r=[th_r[i]/180.*pi for i in range(3)]
    obs.append( Obstacle(a=a, th_r=th_r, p=p, x0=x0_hat, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )


    # Conveyer belt
    a=[1.0,.09,.30]
    #th_r=[-12.,0.,-55.] # In degrees
    x0_hat=[-0,0.5,0.0] # In 
    #x0_hat=[-0.0,0,-00] # In degrees
    th_r=[0.,0.,0] # In degrees
    p=[10.,1.,2.]

    name ='conveyer'
    frame_id = 'world'
    
    th_r=[th_r[i]/180.*pi for i in range(3)]

    obs.append( Obstacle(a=a, th_r=th_r, p=p, x0=x0_hat, name=name, frame_id=frame_id, w=w_0, xd=xd_0) )

elif setUp == 'noObstacle':
    obs=[]


print('Obstacles imported \n')

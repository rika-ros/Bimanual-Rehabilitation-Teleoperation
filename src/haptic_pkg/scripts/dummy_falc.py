#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Wrench,WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates,ModelState
import numpy as np
import scipy
from scipy import linalg

rospy.init_node("Box_control", anonymous=True) # Initialize ros node

pub_left_force_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)
pub_right_force_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

pos_falcon_left = np.array([0.0,0.0,0.0])
pos_falcon_right = np.array([0.0,0.0,0.0])


#variables

msg_falcon_force_left = falconForces()
msg_falcon_force_right = falconForces()

k_joy = 200

def pub_falcon_force_left(force_x,force_y,force_z):
    msg_falcon_force_left.X=force_x
    msg_falcon_force_left.Y=force_y
    msg_falcon_force_left.Z=force_z
    pub_left_force_falcon.publish(msg_falcon_force_left)

def pub_falcon_force_right(force_x,force_y,force_z):
    msg_falcon_force_right.X=force_x
    msg_falcon_force_right.Y=force_y
    msg_falcon_force_right.Z=force_z
    pub_right_force_falcon.publish(msg_falcon_force_right)

#functions that read the data. Therefore named as calllback fns in pgm_call
def callback_pos_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0],data.axes[1],data.axes[2]])  

def callback_pos_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0],data.axes[1],data.axes[2]]) 

rospy.Subscriber("/falcon/joystick", Joy, callback_pos_falcon_left,queue_size=1)
rospy.Subscriber("/falcon/joystick1", Joy, callback_pos_falcon_right,queue_size=1)
#rospy.Subscriber("/gazebo/model_states",ModelStates,box_pos_cb,queue_size=1)
#rospy.Subscriber("/gazebo/link_states",LinkStates,box_pos_cb,queue_size=1)

if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time=0
        prev_pos_left_falcon=np.array([0,0,0])
        prev_pos_right_falcon=np.array([0,0,0])
        global start_time,time_elapsed
        start_time=time.time()
        time_elapsed=0
        while not rospy.is_shutdown():
            if(time.time() - prev_time >= 0.01):
                prev_time=time.time()
                dt=time.time()-start_time-time_elapsed
                time_elapsed=time.time()-start_time
                #print(time_elapsed)

                vel_falcon_left=100*k_joy*(pos_falcon_left-prev_pos_left_falcon)#100 since dt=0.01
                prev_pos_left_falcon=pos_falcon_left
                vel_falcon_right=100*k_joy*(pos_falcon_right-prev_pos_right_falcon)#100 since dt=0.01
                prev_pos_right_falcon=pos_falcon_right


                ############ States ########################################
                
                #pos_box_left  # give the positon of left box
                #pos_box_right  # gives the position of right box
                #vel_box ## gives the velocity of the box
                #pos_box ## gives the position of the box

                #pos_falcon_left # gives the position of left falcon
                #pos_falcon_right # gives the position of right falocon

                #force_box_left # gives the force acting on left box
                #force_box_right # gives the force acting on right box
              
                ############################## 
                if(time_elapsed<5):
                    kpf=3
                    kvf=3
                else:
                    kpf=1
                    kvf=17


                ## Write your code here ######
                if(time_elapsed<5):
                    fx_F_l=0/k_joy  #-kpf*(k_joy*(pos_falcon_left[0])-0)+kvf*(-vel_falcon_left[0])
                    fy_F_l=0/k_joy  #-kpf*(k_joy*(pos_falcon_left[1]-0.1)-0)+kvf*(-vel_falcon_left[1])
                    fz_F_l=0/k_joy  #-10*(k_joy*(pos_falcon_left[2]-0.126)-0)+1*(-vel_falcon_left[2])#kept kp as 10 and kv as 1
                    fx_F_r=0/k_joy  #-kpf*(k_joy*(pos_falcon_right[0])-pos_box_right[0])+kvf*(-vel_falcon_right[0])
                    fy_F_r=0/k_joy  #-kpf*(k_joy*(pos_falcon_right[1]-0.1)-pos_box_right[1])+kvf*(-vel_falcon_right[1])
                    fz_F_r=0/k_joy  #-10*(k_joy*(pos_falcon_right[2]-0.126)-0)+1*(-vel_falcon_right[2])#kept kp as 10 and kv as 1
                else:
                    fx_F_l = 5#-kpf*((pos_falcon_left[0]*k_joy)-5)#force_box_left[0]/(5*kpb)#-0.3*vel_falcon_left[0]#forcefalcon left x
                    fy_F_l = -2#-kpf*((pos_falcon_left[1]*k_joy)-2)#0.5*force_box_left[1]
                    fz_F_l = 3.4#-kpf*((pos_falcon_left[2]*k_joy)-3.4)#.5*force_box_left[2]
                    fx_F_r = 0/k_joy#force_box_right[0]/(5*kpb)#-0.3*vel_falcon_right[0]
                    fy_F_r = 0/k_joy#.5*force_box_right[1]
                    fz_F_r = 0/k_joy#0.5*force_box_right[2]
                    
                    #print([fx_F_l, fx_F_r])#print(pos_box_left)
                    print([pos_falcon_left[0]*k_joy,pos_falcon_left[1]*k_joy,pos_falcon_left[2]*k_joy])


                pub_falcon_force_left(fx_F_l,fy_F_l,fz_F_l)
                pub_falcon_force_right(fx_F_r,fy_F_r,fz_F_r)

                
    except KeyboardInterrupt:
        pass
    # rospy.spin()

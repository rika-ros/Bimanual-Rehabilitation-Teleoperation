#!/usr/bin/env python3

import rospy,scipy,statistics,time
from geometry_msgs.msg import Wrench,WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates,ModelState
import numpy as np
from numpy import matmul
from scipy import linalg
from scipy.io import savemat

rospy.init_node("Box_control", anonymous=True) # Initialize ros node

force_Box = rospy.Publisher("/force_B_fromsides", Wrench, queue_size=1)
Orient_states = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
force_to_left_box = rospy.Publisher("/force_left", Wrench, queue_size=1)   #publisher
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

msg_force_to_left_box =  Wrench()  # msgs
msg_force_to_right_box =  Wrench()
msg_force_to_box =  Wrench()
msg_force_to_left_falcon = falconForces()
msg_force_to_right_falcon = falconForces()

pos_falcon_left = np.array([0.0,0.0,0.0])  #state variables
pos_falcon_right = np.array([0.0,0.0,0.0])
pos_box_left = np.array([0.0,0.0,0.0])
pos_box_right = np.array([0.0,0.0,0.0])
vel_box = np.array([0.0,0.0,0.0])
pos_box = np.array([0.0,0.0,0.0])
orientation_box = np.array([0.0,0.0,0.0,0.0])
force_fb_left_box = np.array([0.0,0.0,0.0]) 
force_fb_right_box = np.array([0.0,0.0,0.0])

#functions for publishing forces or giving inputs
def left_box(force_x,force_y,force_z):  
    msg_force_to_left_box.force.x = force_x
    msg_force_to_left_box.force.y = force_y
    msg_force_to_left_box.force.z = force_z
    force_to_left_box.publish(msg_force_to_left_box)  #calling publisher
def right_box(force_x,force_y,force_z): 
    msg_force_to_right_box.force.x = force_x
    msg_force_to_right_box.force.y = force_y
    msg_force_to_right_box.force.z = force_z
    force_to_right_box.publish(msg_force_to_right_box)    #calling publisher
def box(force_x,force_y,force_z):  
    msg_force_to_box.force.x = force_x
    msg_force_to_box.force.y = force_y
    msg_force_to_box.force.z = force_z
    force_Box.publish(msg_force_to_box)  #calling publisher
def left_falcon(force_x,force_y,force_z):
    msg_force_to_left_falcon.X=force_x
    msg_force_to_left_falcon.Y=force_y
    msg_force_to_left_falcon.Z=force_z
    force_to_left_falcon.publish(msg_force_to_left_falcon)    #calling publisher
def right_falcon(force_x,force_y,force_z):
    msg_force_to_right_falcon.X=force_x
    msg_force_to_right_falcon.Y=force_y
    msg_force_to_right_falcon.Z=force_z
    force_to_right_falcon.publish(msg_force_to_right_falcon)    #calling publisher


#functions for accessing state data
def callback_left_falcon(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0],data.axes[1],data.axes[2]])  
def callback_right_falcon(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0],data.axes[1],data.axes[2]])
def callback_left_box(data): 
    global force_fb_left_box
    force_fb_left_box = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z]
def callback_right_box(data):
    global force_fb_right_box
    force_fb_right_box = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z]  
def box_pos_cb(data):
    global pos_box_left,pos_box_right,vel_box,pos_box, orientation_box
    pos_box = [data.pose[1].position.x,data.pose[1].position.y,data.pose[1].position.z-0.35]
    pos_box_left = [pos_box[0]-0.25-0.05,pos_box[1],pos_box[2]]
    pos_box_right = [pos_box[0]+0.25+0.05,pos_box[1],pos_box[2]]
    vel_box = [data.twist[1].linear.x,data.twist[1].linear.y,data.twist[1].linear.z]
    orientation_box = [data.pose[1].orientation.x,data.pose[1].orientation.y,data.pose[1].orientation.z,data.pose[1].orientation.w]

rospy.Subscriber("/falcon/joystick", Joy, callback_right_falcon,queue_size=1)  #subscribing to joystick data
rospy.Subscriber("/falcon/joystick1", Joy, callback_left_falcon,queue_size=1)
rospy.Subscriber("/left/force_feedback",WrenchStamped,callback_left_box,queue_size=1)    #subscribing to side box force f/b data
rospy.Subscriber("/right/force_feedback",WrenchStamped,callback_right_box,queue_size=1)
rospy.Subscriber("/gazebo/link_states",LinkStates,box_pos_cb,queue_size=1)   #subscribing to box data- position and orientaton..

k_joy = 100
kp = 2

#storage matrices...
pos_box_store = np.empty((3,0))
vel_box_store = np.empty((3,0))
orientation_box_store = np.empty((4,0))
dt=0.01
tf= 50
if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time=0
        prev_pos_fal_left= np.array([0,0,0])
        prev_pos_fal_right= np.array([0,0,0])
        n_count=0
        while not rospy.is_shutdown():
            if(time.time() - prev_time >= dt):
                prev_time=time.time()
                curr_time=time.time()-program_starts 
                vel_falcon_left=(pos_falcon_left*50-prev_pos_fal_left*50)/dt
                if(np.abs(vel_falcon_left[0])<1e-05):
                    vel_falcon_left[0]= 0
                prev_pos_fal_left = pos_falcon_left
                #print(curr_time)
                ######### Publish _force ##########
                right_box(0,0,0)
                box(0.0,0,0) #2cm/s
                print(pos_falcon_left[0]*50,pos_falcon_left[1]*50)
                exl=pos_box[0]-pos_falcon_left[0]*50
                eyl=pos_box[1]-pos_falcon_left[1]*50
                flbx=-7*(exl)-25*vel_box[0]
                flby=-7*(eyl)-25*vel_box[1]
                flbz=0#-20*(pos_box[2]-(pos_falcon_left[2]-0.07)*20)-10*vel_box[2]
                flb=np.array([flbx,flby,flbz])
                exr=pos_box[0]-pos_falcon_right[0]*50
                eyr=pos_box[1]-pos_falcon_right[1]*50
                frbx=-7*(exr)-25*vel_box[0]
                frby=-7*(eyr)-25*vel_box[1]
                frbz=0#-20*(pos_box[2]-(pos_falcon_right[2]-0.07)*20)-10*vel_box[2]
                frb=np.array([frbx,frby,frbz])
                #print(exl,eyl,exr,eyr)
                #print(pos_falcon_left)
                #fb=np.array([-20*(pos_box_left[0]-pos_falcon_left[0]*10)-10*vel_box[0],0.0,0.0])
                left_box(flb[0],flb[1],flb[2])#
                right_box(frb[0],frb[1],frb[2]) #
                left_falcon(-flb[0]+frb[0],-flb[1]+frb[1],-flb[2]+frb[2])
                right_falcon(-frb[0]+flb[0],-frb[1]+flb[1],-frb[2]+flb[2])
                #print(pos_falcon_left)
                #print(flb)
                #print(force_fb_left_box)

                #print(exl,eyl)
                #print(pos_box_left)
                pos_box_store= np.append(pos_box_store, np.matrix([[pos_box[0]],[pos_box[1]],[pos_box[2]]]), axis=1)
                vel_box_store= np.append(vel_box_store, np.matrix([[vel_box[0]],[vel_box[1]],[vel_box[2]]]), axis=1)
                orientation_box_store= np.append(orientation_box_store, np.matrix([[orientation_box[0]],[orientation_box[1]],[orientation_box[2]],[orientation_box[3]]]), axis=1)

                if(curr_time>tf):
                    scipy.io.savemat('pos_.mat', {'pos_': pos_box_store})
                    scipy.io.savemat('vel_.mat', {'vel_': vel_box_store})
                    scipy.io.savemat('orient_.mat', {'orient_': orientation_box_store})
                    left_falcon(0,0,0)
                    right_falcon(0,0,0)
                    break

                
    except KeyboardInterrupt:
        pass
    # rospy.spin()
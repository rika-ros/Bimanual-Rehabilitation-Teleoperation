#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Wrench,WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates,ModelStates
import numpy as np

rospy.init_node("Box_control", anonymous=True) # Initialize ros node

pub_left_force_box = rospy.Publisher("/force_left", Wrench, queue_size=1)
pub_left_force_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)
pub_right_force_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
pub_right_force_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

pos_falcon_left = np.array([0.0,0.0,0.0])
pos_falcon_right = np.array([0.0,0.0,0.0])

pos_box_left = np.array([0.0,0.0,0.0])
pos_box_right = np.array([0.0,0.0,0.0])

force_box_left = np.array([0.0,0.0,0.0])
force_box_right = np.array([0.0,0.0,0.0])

vel_box = np.array([0.0,0.0,0.0])
pos_box = np.array([0.0,0.0,0.0])

msg_box_force_left =  Wrench()
msg_falcon_force_left = falconForces()

msg_box_force_right =  Wrench()
msg_falcon_force_right = falconForces()

joy_stick_sensitivity = 100
kp=10
kv=8

def pub_box_force_left(force_x,force_y,force_z=0):
    msg_box_force_left.force.x = force_x
    msg_box_force_left.force.y = force_y
    msg_box_force_left.force.z = force_z
    pub_left_force_box.publish(msg_box_force_left)


def pub_box_force_right(force_x,force_y,force_z=0):
    msg_box_force_right.force.x = force_x
    msg_box_force_right.force.y = force_y
    msg_box_force_right.force.z = force_z
    pub_right_force_box.publish(msg_box_force_right)

def pub_falcon_force_left(force_x,force_y,force_z=0):
    msg_falcon_force_left.X=force_x
    msg_falcon_force_left.Y=force_y
    msg_falcon_force_left.Z=force_z
    pub_left_force_falcon.publish(msg_falcon_force_left)

def pub_falcon_force_right(force_x,force_y,force_z=0):
    msg_falcon_force_right.X=force_x
    msg_falcon_force_right.Y=force_y
    msg_falcon_force_right.Z=force_z
    pub_right_force_falcon.publish(msg_falcon_force_right)

def callback_pos_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0],data.axes[1],data.axes[2]])
    #print(data.axes)
    #print(data.axes[0])
    

def callback_pos_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0],data.axes[1],data.axes[2]])
    

def callback_force_box_left(data):
    global force_box_left
    force_box_left = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z]
    

def callback_force_box_right(data):
    global force_box_right
    force_box_right = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z]
    
def box_pos_cb(data):
    global pos_box_left,pos_box_right,vel_box,pos_box
    pos_box = [data.pose[1].position.x,data.pose[1].position.y,data.pose[1].position.z]
    pos_box_left = [pos_box[0]-0.25-0.025,pos_box[1],pos_box[2]]
    pos_box_right = [pos_box[0]+0.25+0.025,pos_box[1],pos_box[2]]
    vel_box = [data.twist[1].linear.x,data.twist[1].linear.y,data.twist[1].linear.z]


rospy.Subscriber("/falcon/joystick", Joy, callback_pos_falcon_left,queue_size=1)
rospy.Subscriber("/left/force_feedback",WrenchStamped,callback_force_box_left,queue_size=1)
rospy.Subscriber("/falcon/joystick1", Joy, callback_pos_falcon_right,queue_size=1)
rospy.Subscriber("/right/force_feedback",WrenchStamped,callback_force_box_right,queue_size=1)
rospy.Subscriber("/gazebo/link_states",LinkStates,box_pos_cb,queue_size=1)
prev_error_x = 0
prev_error_y = 0

if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time=0
        global curr_pos_left_falcon
        prev_pos_left_falcon=np.array([0,0,0])
        prev_pos_right_falcon=np.array([0,0,0])
        while not rospy.is_shutdown():
            if(time.time() - prev_time >= 0.01):
                prev_time=time.time()
                vel_left_falcon=100*joy_stick_sensitivity*(pos_falcon_left-prev_pos_left_falcon)#100 since dt=0.01
                prev_pos_left_falcon=pos_falcon_left
                vel_right_falcon=100*joy_stick_sensitivity*(pos_falcon_right-prev_pos_right_falcon)#100 since dt=0.01
                prev_pos_right_falcon=pos_falcon_right

                ############ States ########################################
                """
                pos_box_left  # give the positon of left box
                pos_box_right  # gives the position of right box
                vel_box $ gives the velocity of the box
                pos_falcon_left # gives the position of left falcon
                pos_falcon_right # gives the position of right falocon
                force_box_left # gives the force acting on left box
                force_box_right # gives the force acting on right box
                
                """
                ############################## 
                #print(pos_falcon_left)
                #print(pos_falcon_right)
                #print(pos_box_right)
                #print(pos_box)
                #print(force_box_left)
                #print(force_box_right)

                ## Write your code here ######
                force_left_box_x = 0#kp*(joy_stick_sensitivity*(pos_falcon_left[0])-pos_box[0])#+kv*(vel_left_falcon[0]-vel_box[0])
                force_left_box_y = 5#kp*joy_stick_sensitivity*(pos_falcon_left[1])#kp*(joy_stick_sensitivity*(pos_falcon_left[1])-pos_box[1])+kv*(vel_left_falcon[0]-vel_box[0])
                #force_left_box_z = 0#joy_stick_sensitivity*(pos_falcon_left[2]

                force_right_box_x = 0#force_box_left[0]#0#joy_stick_sensitivity*pos_falcon_right[0]
                force_right_box_y = -15#force_box_left[1]#0#joy_stick_sensitivity*pos_falcon_right[1]
                #force_right_box_z = 0#joy_stick_sensitivity*pos_falcon_right[2]

                force_left_falcon_x = 0#force_box_left[0]
                force_left_falcon_y = 0#force_box_left[1]
                #force_left_falcon_z = force_box_left[2]

                force_right_falcon_x = 0#force_box_right[0]
                force_right_falcon_y = 0#force_box_right[1]
                #force_right_falcon_z = force_box_right[2]
                #print(force_box_right[0])
                #print(force_box_right[1])


                ######### Publish _force ##########

                pub_box_force_left(force_left_box_x,force_left_box_y) 
                pub_box_force_right(force_right_box_x,force_right_box_y)
                #pub_falcon_force_left(force_left_falcon_x,force_left_falcon_y)
                #pub_falcon_force_right(force_right_falcon_x,force_right_falcon_y)

                ######## Publish force to box and falcon ########
                """
                pub_box_force_left(force_left_box_x,force_left_box_y) 
                pub_box_force_right(force_right_box_x,force_right_box_y)
                pub_falcon_force_left(force_left_falcon_x,force_left_falcon_y)
                pub_falcon_force_right(force_right_falcon_x,force_right_falcon_y)
                
                """


                
    except KeyboardInterrupt:
        pass
    # rospy.spin()
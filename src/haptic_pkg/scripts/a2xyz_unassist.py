#!/usr/bin/env python3
#unassist xy

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
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)

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
omeg=2*np.math.pi/60

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
    #orientation_box = [data.pose[1].orientation.x,data.pose[1].orientation.y,data.pose[1].orientation.z,data.pose[1].orientation.w]

rospy.Subscriber("/falcon/joystick", Joy, callback_right_falcon,queue_size=1)  #subscribing to joystick data
rospy.Subscriber("/falcon/joystick1", Joy, callback_left_falcon,queue_size=1)
rospy.Subscriber("/left/force_feedback",WrenchStamped,callback_left_box,queue_size=1)    #subscribing to side box force f/b data
rospy.Subscriber("/right/force_feedback",WrenchStamped,callback_right_box,queue_size=1)
rospy.Subscriber("/gazebo/link_states",LinkStates,box_pos_cb,queue_size=1)   #subscribing to box data- position and orientaton..

D=0
M=1.02
Ax=np.matrix([[0, 1], [0, -D/M]])
Bx=np.matrix([[0], [1/M]])
Ay=np.matrix([[0, 1], [0, -D/M]])
By=np.matrix([[0], [1/M]])
dt=0.01
tf= 80

k_joy = 100
kp = 2
#storage matrices...
pos_box_store = np.empty((3,0))
vel_box_store = np.empty((3,0))
#orientation_box_store = np.empty((4,0))
pos_box_des_storeX = np.empty((0,1))
pos_box_des_storeY = np.empty((0,1))
pos_box_des_storeZ = np.empty((0,1))
time_store = np.empty((0,1))
#intialize
X = np.matrix([[pos_box[0]],[vel_box[0]]])
Y = np.matrix([[pos_box[1]],[vel_box[1]]])
Z = np.matrix([[pos_box[2]],[vel_box[2]]])
TH= np.matrix([[0],[0]])
THd=np.matrix([[0],[0]])
Xd = np.matrix([[0.0],[0]])
PHI= np.matrix([[0],[0]])
PHId=np.matrix([[0],[0]])

if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time = 0
        prev_pos_falcon_left = np.array([0, 0, 0])
        prev_pos_falcon_right = np.array([0, 0, 0])
        n_count = 0

        while not rospy.is_shutdown():
            if (time.time() - prev_time >= dt):
                dta = time.time() - program_starts - prev_time
                prev_time = time.time() - program_starts

                # des_pos(curr_time)
                # print(prev_time)
                if (prev_time < 10):
                    Xd = Xd + np.matrix([[dt / 40], [0]])
                    Yd = np.matrix([[0], [0]])
                    Zd = np.matrix([[0.0], [0]])
                elif (prev_time < 20):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd + np.matrix([[dt / 90], [0]])
                elif (prev_time < 30):
                    Xd = Xd
                    Yd = Yd + np.matrix([[dt / 40], [0]])
                    Zd = Zd
                elif (prev_time < 40):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd - np.matrix([[dt / 90], [0]])
                elif (prev_time < 50):
                    Xd = Xd - np.matrix([[dt / 40], [0]])
                    Yd = Yd
                    Zd = Zd
                elif (prev_time < 60):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd + np.matrix([[dt / 90], [0]])
                elif (prev_time < 70):
                    Xd = Xd
                    Yd = Yd - np.matrix([[dt / 40], [0]])
                    Zd = Zd
                else:
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd - np.matrix([[dt / 90], [0]])

                des_pos = np.array([Xd[0], Yd[0], Zd[0]])

                if int(prev_time) % 10 == 0 and abs(prev_time - int(prev_time)) < 0.01:
                    sec = int(prev_time)
                    if sec < 10:
                        print("0-10 sec: Move in X from 0 to 0.75 | Y: 0 | Z: 0")
                        print(des_pos)
                    elif sec < 20:
                        print("10-20 sec: Hold X = 0.75, Y = 0 | Move Z from 0 to 0.7")
                        print(des_pos)
                    elif sec < 30:
                        print("20-30 sec: Hold X = 0.75, Z = 0.7 | Move Y from 0 to 0.8")
                        print(des_pos)
                    elif sec < 40:
                        print("30-40 sec: Hold X = 0.75, Y = 0.8 | Move Z from 0.7 to 0.1")
                        print(des_pos)
                    elif sec < 50:
                        print("40-50 sec: Hold Y = 0.8, Z = 0.1 | Move X from 0.75 to 0.001")
                        print(des_pos)
                    elif sec < 60:
                        print("50-60 sec: Hold X = 0.001, Y = 0.8 | Move Z from 0.1 to 0.7")
                        print(des_pos)
                    elif sec < 70:
                        print("60-70 sec: Hold X = 0.001, Z = 0.7 | Move Y from 0.8 to 0.1")
                        print(des_pos)
                    elif sec < 80:
                        print("70-80 sec: Hold X = 0.001, Y = 0.1 | Move Z from 0.7 to 0")
                        print(des_pos)
                    else:
                        print("80+ sec: Trajectory complete or idle.")
                        print(des_pos)

                #print(pos_box)
                xix = X-Xd
                xiyp = Y-Yd
                xizp = Z-Zd
                xith = TH-THd
                xiphi = PHI-PHId
                exl=pos_box[0]-pos_falcon_left[0]*50
                uxlH=-8*(exl)-10*vel_box[0]
                eyl=pos_box[1]-(pos_falcon_left[2]-0.12)*(50) #eyl=pos_box[1]-(-pos_falcon_left[2]+0.12)*(50)
                uylH=-5*(eyl)-10*vel_box[1]
                ezl=pos_box[2]-pos_falcon_left[1]*50
                uzlH=-5*(ezl)-10*vel_box[2]
                flH=np.matrix([[uxlH],[uylH],[uzlH]])
                exr=pos_box[0]-pos_falcon_right[0]*50
                uxrH=-8*(exr)-10*vel_box[0]
                eyr=pos_box[1]-(pos_falcon_right[2]-0.12)*(50)
                uyrH=-5*(eyr)-10*vel_box[1]
                ezr=pos_box[2]-pos_falcon_right[1]*50
                uzrH=-5*(ezr)-10*vel_box[2]
                frH=np.matrix([[uxrH],[uyrH],[uzrH]])
                uxH=uxlH+uxrH
                flb=flH#+flR
                frb=frH#+frR
                #frot=frb-flb

                left_box(flb[0],flb[1],flb[2])#
                right_box(frb[0],frb[1],frb[2]) #
                left_falcon(-flH[0],-flH[2],-flH[1])#(-0,0,0)#(-flH[0]+flR[0]+frb[0],-flH[1]+flR[1]+frb[1],-flH[2]+flR[2]+frb[2])
                right_falcon(-frH[0],-frH[2],-frH[1])#(0,0,0)#(-frH[0]+frR[0]+flb[0],-frH[1]+frR[1]+flb[1],-frH[2]+frR[2]+flb[2])
                #print(np.matmul(np.matmul(By,uyrH-uylH)))
                
                X = np.matrix([[pos_box[0]],[vel_box[0]]])
                Y = np.matrix([[pos_box[1]],[vel_box[1]]])
                Z = np.matrix([[pos_box[2]],[vel_box[2]]])
                xix=X-Xd
                xiy=Y-Yd
                xiz=Z-Zd
                
                pos_box_store= np.append(pos_box_store, np.matrix([[pos_box[0]],[pos_box[1]],[pos_box[2]]]), axis=1)
                vel_box_store= np.append(vel_box_store, np.matrix([[vel_box[0]],[vel_box[1]],[vel_box[2]]]), axis=1)
                pos_box_des_storeX= np.append(pos_box_des_storeX, Xd[0], axis=0) 
                pos_box_des_storeY= np.append(pos_box_des_storeY, Yd[0], axis=0) 
                pos_box_des_storeZ= np.append(pos_box_des_storeZ, Zd[0], axis=0) 
                time_store= np.append(time_store, np.matrix(prev_time), axis=0)

                ######### Publish _force ##########
                if(prev_time>tf):
                    scipy.io.savemat('tim_.mat', {'tim_': time_store})
                    scipy.io.savemat('pos_.mat', {'pos_': pos_box_store})
                    scipy.io.savemat('vel_.mat', {'vel_': vel_box_store})
                    #scipy.io.savemat('orient_.mat', {'orient_': orientation_box_store})
                    #scipy.io.savemat('pos_des_.mat', {'pos_des_': pos_box_des_store})
                    scipy.io.savemat('pos_des_X.mat', {'pos_des_X': pos_box_des_storeX})  
                    scipy.io.savemat('pos_des_Y.mat', {'pos_des_Y': pos_box_des_storeY})
                    scipy.io.savemat('pos_des_Z.mat', {'pos_des_Z': pos_box_des_storeZ}) 
                    left_falcon(0,0,0)
                    right_falcon(0,0,0)
                    break

                
    except KeyboardInterrupt:
        pass
    # rospy.spin()

#!/usr/bin/env python3
#z

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
    msg_force_to_left_falcon.Y=force_z
    msg_force_to_left_falcon.Z=force_y
    force_to_left_falcon.publish(msg_force_to_left_falcon)    #calling publisher
def right_falcon(force_x,force_y,force_z):
    msg_force_to_right_falcon.X=force_x
    msg_force_to_right_falcon.Y=force_z
    msg_force_to_right_falcon.Z=force_y
    force_to_right_falcon.publish(msg_force_to_right_falcon)    #calling publisher


#functions for accessing state data
def callback_left_falcon(data):
    global pos_falcon_left,button_l
    pos_falcon_left = np.array([data.axes[0],data.axes[2]-0.12,data.axes[1]+0.06]) 
    button_l=data.buttons[0] 
def callback_right_falcon(data):
    global pos_falcon_right,button_r
    pos_falcon_right = np.array([data.axes[0],data.axes[2]-0.12,data.axes[1]+0.06])
    button_r=data.buttons[0]
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

QzR=np.diag([1,0])       #Robot parameters
LzrHh=np.matrix([[0,0]])
#LzlHh=np.matrix([[0,0]])
LzsHh=np.matrix([[0,0]])
LzdHh=np.matrix([[0,0]])
PzsHh=np.matrix([[0, 0],[0, 0]])
PzdHh=np.matrix([[0, 0],[0, 0]])
kzrHh=np.matrix([[0],[0]])
kzlHh=np.matrix([[0],[0]])
kzsHh=np.matrix([[0],[0]])
kzdHh=np.matrix([[0],[0]])
xizph=np.matrix([[0],[0]])
xizthh=np.matrix([[0],[0]])
xizptilde=np.matrix([[0],[0]])
xizthtilde=np.matrix([[0],[0]])
# uzrR=np.matrix([[0]]) #not
# uzlR=np.matrix([[0]])
D=0
M=1.02
g=9.81
Az=np.matrix([[0, 1], [0, 0]])
Bz=np.matrix([[0], [1/M]])
cz=np.matrix([[0], [-g]])
Gammazp=np.diag([100, 2]) 
Gammazth=np.diag([100, 2]) 
alphazp=np.diag([10,10])
alphazth=np.diag([10,10])
dt=0.01
tf= 40

k_joy = 100
kp = 2
#storage matrices...
pos_box_store = np.empty((3,0))
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

if __name__ == "__main__":
    try:
        program_starts = time.time()
        curr_time=0

        while not rospy.is_shutdown():
            if(time.time() - curr_time >= dt):
                dta=time.time() - program_starts - curr_time
                curr_time=time.time()-program_starts
                # print(curr_time)
                # if (curr_time<5):
                #     Zd = np.matrix([[0],[0]])
                # elif (curr_time<10):
                #     Zd = np.matrix([[1],[0]])
                # elif (curr_time<15):
                #     Zd = np.matrix([[2],[0]])
                # elif (curr_time<20):
                #     Zd = np.matrix([[1],[0]])
                # else:
                #     Zd = np.matrix([[0],[0]])
                # print("Desired state: {}Actual states: {}".format(Zd[0],pos_box[1]))
                
                # xizp = Z-Zd
                # xith = TH-THd
                exl=pos_box[0]-(pos_falcon_left[0]*50-0)
                #print(pos_falcon_left[0])
                uxlH=-2*(exl)-1*vel_box[0]
                eyl=pos_box[1]-(pos_falcon_left[1]*50-0)
                uylH=-2*(eyl)-1*vel_box[1]
                ezl=pos_box[2]-pos_falcon_left[2]*50
                uzlH=-5*(ezl)-1*vel_box[2]

                exr=pos_box[0]-pos_falcon_right[0]*50
                uxrH=-2*(exr)-10*vel_box[0]
                eyr=pos_box[1]-pos_falcon_right[1]*50
                uyrH=-2*(eyr)-10*vel_box[1]
                ezr=pos_box[2]-pos_falcon_right[2]*50
                uzrH=-5*(ezr)-10*vel_box[2] 

                # uzsH=uzlH+uzrH
                # uzdH=uzrH-uzlH

                # a = Az-(np.matmul(Bz,LzsHh))
                # q = QzR+np.matmul(LzsHh.T,LzsHh)   #took transpose
                # b = Bz
                # r = 1
                # PzR = linalg.solve_continuous_are(a, b, q, r) #linalg.solve_continuous_are(a, b, q, r)  #np.matrix([[5, 1],[1, 1]])
                # LzR=np.matmul(Bz.T,PzR)
                # LzrR=np.multiply(0.5,LzR+LzsHh)-LzrHh
                # LzlR=LzR-LzrR
                # #-----
                # kzR=np.matrix([[0],-kzsHh[1]-(M*M*g)])
                # kzrR=np.multiply(0.5,kzR+kzsHh)-kzrHh
                # kzlR=kzR-kzrR
                # print(np.matmul(np.matmul(-Bz,Bz.T),kzR+kzsHh)+cz)
                # uzrR = -np.matmul(LzrR,xizp)#-np.matmul(Bz.T,kzrR)
                # uzlR = -np.matmul(LzlR,xizp)#-np.matmul(Bz.T,kzlR)
                #------------
                # uxlR=0
                # uxrR=0
                # uylR=0
                # uyrR=0

                # uzsHh= -np.matmul(LzsHh,xizp)#-np.matmul(Bz.T,kzsHh)
                # uzdHh= -np.matmul(LzdHh,xizp)#-np.matmul(Bz.T,kzdHh)
                # xizph= xizph+np.multiply(dt,np.matmul(Az,xizph)+np.matmul(Bz,(uzrR+uzlR))+np.matmul(Bz,uzsHh)-np.matmul(Gammazp,xizptilde))#+cz
                # xizthh=xizthh+np.multiply(dt,np.matmul(Az,xizthh)+np.matmul(Bz,(uzrR-uylR))+np.matmul(Bz,uzdHh)-np.matmul(Gammazth,xizthtilde))#+cz
                # PzsHh= PzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,xizptilde),xizp.T))
                # PzdHh= PzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,xizthtilde),xizp.T))
                # #-===============
                # kzsHh= kzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,np.matmul(Bz,Bz.T)),xizptilde))
                # kzdHh= kzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,np.matmul(Bz,Bz.T)),xizthtilde))
                # PzsHh[0,1]=PzsHh[1,0]
                # PzdHh[0,1]=PzdHh[1,0]
                # kzsHh[0] = 0
                # kzdHh[0] = 0
                # LzsHh=np.matmul(Bz.T,PzsHh)
                # LzdHh=np.matmul(Bz.T,PzdHh)
                # LzrHh=np.multiply(0.5,LzsHh+LzdHh)
                # LzlHh=LzsHh-LzrHh
                # kzrHh=np.multiply(0.5,kzsHh+kzdHh)
                # kzlHh=kzsHh-kzrHh
                # uzrHh= -np.matmul(LzrHh,xizp)-np.matmul(Bz.T,kzrHh)
                # uzlHh= -np.matmul(LzlHh,xizp)-np.matmul(Bz.T,kzlHh)
                #============-

                left_box(uxlH,uylH,uzlH)#
                right_box(uxrH,uyrH,uzrH) #
                left_falcon(-uxlH+uxrH,-uylH+uyrH,-uzlH+uzrH)
                right_falcon(-uxrH+uxlH,-uyrH+uylH,-uzrH+uzlH)
                # left_falcon(-uxlH+uxlR+uxrH,-uylH+uylR+uyrH,-uzlH+uzlR+uzrH)
                # right_falcon(-uxrH+uxrR+uxlH,-uyrH+uyrR+uylH,-uzrH+uzrR+uzlH)
                # left_falcon(0,0,0)
                # right_falcon(0,0,0)
                #print(np.matmul(np.matmul(By,uyrH-uylH)))
                # xith=xith+np.multiply(dt,np.matmul(Az,xith)+np.matmul(Bz,uzrR-uzlR)+np.matmul(Bz,np.matrix(uzrH-uzlH)))#+ cz
                # X = np.matrix([[pos_box[0]],[vel_box[0]]])
                # Y = np.matrix([[pos_box[1]],[vel_box[1]]])
                # Z = np.matrix([[pos_box[2]],[vel_box[2]]])
                # xiz=Z-Zd
                # TH= xith+THd
                # xizptilde=xizph-xizp
                # xizthtilde=xizthh-xith
                
                # pos_box_store= np.append(pos_box_store, np.matrix([[pos_box[0]],[pos_box[1]],[pos_box[2]]]), axis=1)
                # #orientation_box_store= np.append(orientation_box_store, np.matrix([[orientation_box[0]],[orientation_box[1]],[orientation_box[2]],[orientation_box[3]]]), axis=1)
                # #pos_box_des_store= np.append(pos_box_des_store, np.array([Xd[0],Yd[0],Zd[0]]), axis=1) 
                # pos_box_des_storeZ= np.append(pos_box_des_storeZ, Zd[0], axis=0) 
                # time_store= np.append(time_store, np.matrix(curr_time), axis=0)

                ######### Publish _force ##########
                if(curr_time>tf or button_r==4 or button_l==4):
                    # scipy.io.savemat('tim_.mat', {'tim_': time_store})
                    # scipy.io.savemat('pos_.mat', {'pos_': pos_box_store})
                    # scipy.io.savemat('pos_des_Z.mat', {'pos_des_Z': pos_box_des_storeZ}) 
                    left_falcon(0,0,0)
                    right_falcon(0,0,0)
                    break

                
    except KeyboardInterrupt:
        pass
    # rospy.spin()
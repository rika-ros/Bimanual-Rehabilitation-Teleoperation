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
omeg=2*np.math.pi/60

button_r = 0
button_l = 0
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
    pos_falcon_left = np.array([data.axes[0],data.axes[2]-0.12,data.axes[1]]) 
    button_l=data.buttons[0] 
def callback_right_falcon(data):
    global pos_falcon_right,button_r
    pos_falcon_right = np.array([data.axes[0],data.axes[2]-0.12,data.axes[1]])
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

QxR=np.diag([0.5,0])
QyR=np.diag([0.5,0])
QzR=np.diag([2,0])       #Robot parameters
LyrHh=np.matrix([[0,0]])
LzrHh=np.matrix([[0,0]])
#LzlHh=np.matrix([[0,0]])
LxHh=np.matrix([[0,0]])
LysHh=np.matrix([[0,0]])
LydHh=np.matrix([[0,0]])
LzsHh=np.matrix([[0,0]])
LzdHh=np.matrix([[0,0]])
PxHh=np.matrix([[0, 0],[0, 0]])
PysHh=np.matrix([[0, 0],[0, 0]])
PydHh=np.matrix([[0, 0],[0, 0]])
PzsHh=np.matrix([[0, 0],[0, 0]])
PzdHh=np.matrix([[0, 0],[0, 0]])
kzrHh=np.matrix([[0],[0]])
kzlHh=np.matrix([[0],[0]])
kzsHh=np.matrix([[0],[0]])
kzdHh=np.matrix([[0],[0]])
xixh=np.matrix([[0],[0]])
xiyph=np.matrix([[0],[0]])
xiythh=np.matrix([[0],[0]])
xizph=np.matrix([[0],[0]])
xizthh=np.matrix([[0],[0]])
xixtilde=np.matrix([[0],[0]])
xiyptilde=np.matrix([[0],[0]])
xiythtilde=np.matrix([[0],[0]])
xizptilde=np.matrix([[0],[0]])
xizthtilde=np.matrix([[0],[0]])
# uzrR=np.matrix([[0]]) #not
# uzlR=np.matrix([[0]])
D=0
M=1.02
g=9.81
A=np.matrix([[0, 1], [0, 0]])
B=np.matrix([[0], [1/M]])
Az=np.matrix([[0, 1], [0, 0]])
Bz=np.matrix([[0], [1/M]])
cz=np.matrix([[0], [-g]])
Gammax=np.diag([100, 2])
Gammayp=np.diag([100, 2]) 
Gammayth=np.diag([100, 2]) 
Gammazp=np.diag([100, 2]) 
Gammazth=np.diag([100, 2]) 
alphax=np.diag([10,10])
alphayp=np.diag([10,10])
alphayth=np.diag([10,10])
alphazp=np.diag([10,10])
alphazth=np.diag([10,10])
dt=0.01
tf= 80

k_joy = 100
kp = 2
#storage matrices...
pos_box_store = np.empty((3,0))
pos_box_des_storeX = np.empty((0,1))
pos_box_des_storeY = np.empty((0,1))
pos_box_des_storeZ = np.empty((0,1))
LxHh_store = np.empty((0,2))
LyrHh_store = np.empty((0,2))
LylHh_store = np.empty((0,2))
LzrHh_store = np.empty((0,2))
LzlHh_store = np.empty((0,2))
kzrHh_store = np.empty((2,0))
kzlHh_store = np.empty((2,0))
LxrR_store = np.empty((0,2))
LxlR_store = np.empty((0,2))
LyrR_store = np.empty((0,2))
LylR_store = np.empty((0,2))
LzrR_store = np.empty((0,2))
LzlR_store = np.empty((0,2))
kzrR_store = np.empty((2,0))
kzlR_store = np.empty((2,0))
uxH_store = np.empty((0,1))        #for empty array kind put   np.empty(0) alone
uyrH_store = np.empty((0,1)) 
uylH_store = np.empty((0,1))
uzrH_store = np.empty((0,1)) 
uzlH_store = np.empty((0,1))
uxHh_store = np.empty((0,1))
uyrHh_store = np.empty((0,1))
uylHh_store = np.empty((0,1))
uzrHh_store = np.empty((0,1))
uzlHh_store = np.empty((0,1))
uxrR_store = np.empty((0,1))
uxlR_store = np.empty((0,1))
uyrR_store = np.empty((0,1))
uylR_store = np.empty((0,1))
uzrR_store = np.empty((0,1))
uzlR_store = np.empty((0,1))
time_store = np.empty((0,1))
#intialize
X = np.matrix([[pos_box[0]],[vel_box[0]]])
Y = np.matrix([[pos_box[1]],[vel_box[1]]])
Z = np.matrix([[pos_box[2]],[vel_box[2]]])
Xd = np.matrix([[0.0],[0]])
THy= np.matrix([[0],[0]])
THdy=np.matrix([[0],[0]])
THz= np.matrix([[0],[0]])
THdz=np.matrix([[0],[0]])

if __name__ == "__main__":
    try:
        program_starts = time.time()
        curr_time=0

        while not rospy.is_shutdown():
            if(time.time() - curr_time >= dt):
                dta=time.time() - program_starts - curr_time
                curr_time=time.time()-program_starts
                # print(curr_time)
                if (curr_time<10):
                    Xd = Xd+np.matrix([[dt/10],[0]])
                    Yd = np.matrix([[0],[0]])
                    Zd = np.matrix([[0.0],[0]])
                    print("X: {}Xd: {}".format(pos_box[0],Xd[0]))
                    test=1
                elif (curr_time<20):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd+np.matrix([[dt/10],[0]])
                    print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                    test=3
                elif (curr_time<30):
                    Xd = Xd
                    Yd = Yd+np.matrix([[dt/10],[0]])
                    Zd = Zd
                    print("Y: {}Yd: {}".format(pos_box[1],Yd[0]))
                    test=2
                elif (curr_time<40):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd-np.matrix([[dt/10],[0]])
                    print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                    test=3
                elif (curr_time<50):
                    Xd = Xd-np.matrix([[dt/10],[0]])
                    Yd = Yd
                    Zd = Zd
                    print("X: {}Xd: {}".format(pos_box[0],Xd[0]))
                    test=1
                elif (curr_time<60):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd+np.matrix([[dt/10],[0]])
                    print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                    test=3
                elif (curr_time<70):
                    Xd = Xd
                    Yd = Yd-np.matrix([[dt/10],[0]])
                    Zd = Zd
                    print("Y: {}Yd: {}".format(pos_box[1],Yd[0]))
                    test=2
                else:
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd-np.matrix([[dt/10],[0]])
                    print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                    test=3

                # if (curr_time<15):
                #     Xd = np.matrix([[1],[0]])
                #     Yd = np.matrix([[0],[0]])
                #     Zd = np.matrix([[0],[0]])
                #     print("X: {}Xd: {}".format(pos_box[0],Xd[0]))
                #     test=1
                # elif (curr_time<30):
                #     Xd = np.matrix([[1],[0]])
                #     Yd = np.matrix([[0],[0]])
                #     Zd = np.matrix([[1],[0]])
                #     print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                #     test=3
                # elif (curr_time<45):
                #     Xd = np.matrix([[1],[0]])
                #     Yd = np.matrix([[1],[0]])
                #     Zd = np.matrix([[1],[0]])
                #     print("Y: {}Yd: {}".format(pos_box[1],Yd[0]))
                #     test=2
                # elif (curr_time<60):
                #     Xd = np.matrix([[0],[0]])
                #     Yd = np.matrix([[1],[0]])
                #     Zd = np.matrix([[1],[0]])
                #     print("X: {}Xd: {}".format(pos_box[0],Xd[0]))
                #     test=1
                # else:
                #     Xd = np.matrix([[0],[0]])
                #     Yd = np.matrix([[1],[0]])
                #     Zd = np.matrix([[0],[0]])
                #     print("Z: {}Zd: {}".format(pos_box[2],Zd[0]))
                #     test=3

                # Xd = np.matrix([[0.5*np.math.cos(omeg*curr_time)],[0]])
                # Yd = np.matrix([[0.5*np.math.cos(omeg*curr_time)],[0]])
                # Zd = np.matrix([[0.5*np.math.sin(omeg*curr_time)+0.55],[0]])
                # print("Desired state: {}Actual states: {}".format(Zd[0],pos_box[2]))
                
                xix = X-Xd
                xiyp = Y-Yd
                xithy = THy-THdy
                xizp = Z-Zd
                xithz = THz-THdz
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

                uxH=uxlH+uxrH
                uysH=uylH+uyrH
                uydH=uyrH-uylH
                uzsH=uzlH+uzrH
                uzdH=uzrH-uzlH

                a = A-(np.matmul(B,LxHh))
                q = QxR+np.matmul(LxHh.T,LxHh)   #took transpose
                b = B
                r = 1
                PxR = linalg.solve_continuous_are(a, b, q, r) 
                LxR=np.matmul(B.T,PxR)
                uxR=np.matmul(-LxR,xix)
                if (uxR>0):
                    LxlR=LxR
                    uxlR=uxR
                    LxrR=np.matrix([[0,0]])
                    uxrR=np.matrix([[0]])
                else:
                    LxrR=LxR
                    uxrR=uxR
                    LxlR=np.matrix([[0,0]])
                    uxlR=np.matrix([[0]])

                a = A-(np.matmul(B,LysHh))
                q = QyR+np.matmul(LysHh.T,LysHh)   #took transpose
                b = B
                r = 1
                PyR = linalg.solve_continuous_are(a, b, q, r) 
                LyR=np.matmul(B.T,PyR)
                LyrR=np.multiply(0.5,LyR+LysHh)-LyrHh
                LylR=LyR-LyrR
                uyrR = -np.matmul(LyrR,xiyp)
                uylR = -np.matmul(LylR,xiyp)

                a = Az-(np.matmul(Bz,LzsHh))
                q = QzR+np.matmul(LzsHh.T,LzsHh)   #took transpose
                b = Bz
                r = 1
                PzR = linalg.solve_continuous_are(a, b, q, r) #linalg.solve_continuous_are(a, b, q, r)  #np.matrix([[5, 1],[1, 1]])
                LzR=np.matmul(Bz.T,PzR)
                LzrR=np.multiply(0.5,LzR+LzsHh)-LzrHh
                LzlR=LzR-LzrR
                # #-----
                kzR=np.matrix([[0],-kzsHh[1]-(M*M*g)])
                kzrR=np.multiply(0.5,kzR+kzsHh)-kzrHh
                # print(kzrHh)
                # print(kzrR)
                kzlR=kzR-kzrR
                # print(np.matmul(np.matmul(-Bz,Bz.T),kzR+kzsHh)+cz)
                uzrR = -np.matmul(LzrR,xizp)#-np.matmul(Bz.T,kzrR)
                uzlR = -np.matmul(LzlR,xizp)#-np.matmul(Bz.T,kzlR)

                # if (test==1):
                    # uyrH=np.matrix([[0]])
                    # uyrR=np.matrix([[0]])
                     # uzrH=np.matrix([[0]])
                    # uzrR=np.matrix([[0]])

                    # uylH=np.matrix([[0]])
                    # uylR=np.matrix([[0]])

                    # uzlH=np.matrix([[0]])
                    # uzlR=np.matrix([[0]])
                # elif (test==2):
                    # uxrH=np.matrix([[0]])
                    # uxlH=np.matrix([[0]])
                    # uxrR=np.matrix([[0]])
                    # uxlR=np.matrix([[0]])
                    
                    # uzrH=np.matrix([[0]])
                    # uzlH=np.matrix([[0]])
                    # uzrR=np.matrix([[0]])
                    # uzlR=np.matrix([[0]])
                # elif (test==3):
                    # uxrH=np.matrix([[0]])
                    # uxlH=np.matrix([[0]])
                    # uxrR=np.matrix([[0]])
                    # uxlR=np.matrix([[0]])
                    # uyrH=np.matrix([[0]])
                    # uyrR=np.matrix([[0]])
                    # uylH=np.matrix([[0]])
                    # uylR=np.matrix([[0]])
                #------------
                # uxlR=0
                # uxrR=0
                # uylR=0
                # uyrR=0

                uxHh= -np.matmul(LxHh,xix)
                uysHh= -np.matmul(LysHh,xiyp)
                uydHh= -np.matmul(LydHh,xiyp)
                uzsHh= -np.matmul(LzsHh,xizp)#-np.matmul(Bz.T,kzsHh)
                uzdHh= -np.matmul(LzdHh,xizp)#-np.matmul(Bz.T,kzdHh)
                xixh= xixh+np.multiply(dt,np.matmul(A,xixh)+np.matmul(B,(uxrR+uxlR))+np.matmul(B,uxHh)-np.matmul(Gammax,xixtilde))
                xiyph= xiyph+np.multiply(dt,np.matmul(A,xiyph)+np.matmul(B,(uyrR+uylR))+np.matmul(B,uysHh)-np.matmul(Gammayp,xiyptilde))
                xiythh=xiythh+np.multiply(dt,np.matmul(A,xiythh)+np.matmul(B,(uyrR-uylR))+np.matmul(B,uydHh)-np.matmul(Gammayth,xiythtilde))
                xizph= xizph+np.multiply(dt,np.matmul(Az,xizph)+np.matmul(Bz,(uzrR+uzlR))+np.matmul(Bz,uzsHh)-np.matmul(Gammazp,xizptilde))#+cz
                xizthh=xizthh+np.multiply(dt,np.matmul(Az,xizthh)+np.matmul(Bz,(uzrR-uzlR))+np.matmul(Bz,uzdHh)-np.matmul(Gammazth,xizthtilde))#+cz
                PxHh= PxHh+np.multiply(dt,np.matmul(np.matmul(alphax,xixtilde),xix.T))
                PysHh= PysHh+np.multiply(dt,np.matmul(np.matmul(alphayp,xiyptilde),xiyp.T))
                PydHh= PydHh+np.multiply(dt,np.matmul(np.matmul(alphayth,xiythtilde),xiyp.T))
                PzsHh= PzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,xizptilde),xizp.T))
                PzdHh= PzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,xizthtilde),xizp.T))
                # #-===============
                kzsHh= kzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,np.matmul(Bz,Bz.T)),xizptilde))
                kzdHh= kzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,np.matmul(Bz,Bz.T)),xizthtilde))
                PxHh[0,1]=PxHh[1,0]
                PysHh[0,1]=PysHh[1,0]
                PydHh[0,1]=PydHh[1,0]
                PzsHh[0,1]=PzsHh[1,0]
                PzdHh[0,1]=PzdHh[1,0]
                kzsHh[0] = 0
                kzdHh[0] = 0
                LxHh=np.matmul(B.T,PxHh) 
                LysHh=np.matmul(B.T,PysHh)
                LydHh=np.matmul(B.T,PydHh)
                LzsHh=np.matmul(Bz.T,PzsHh)
                LzdHh=np.matmul(Bz.T,PzdHh)
                LyrHh=np.multiply(0.5,LysHh+LydHh)
                LylHh=LysHh-LyrHh
                LzrHh=np.multiply(0.5,LzsHh+LzdHh)
                LzlHh=LzsHh-LzrHh
                kzrHh=np.multiply(0.5,kzsHh+kzdHh)
                kzlHh=kzsHh-kzrHh
                uxHh= -np.matmul(LxHh,xix)
                uyrHh= -np.matmul(LyrHh,xiyp)
                uylHh= -np.matmul(LylHh,xiyp)
                uzrHh= -np.matmul(LzrHh,xizp)-np.matmul(Bz.T,kzrHh)
                uzlHh= -np.matmul(LzlHh,xizp)-np.matmul(Bz.T,kzlHh)
                
                #============-

                left_box(uxlH,uylH,uzlH)#
                right_box(uxrH,uyrH,uzrH) #
                left_falcon(-uxlH+uxrH+uxlR,-uylH+uyrH+uylR,-uzlH+uzrH+uzlR)
                right_falcon(-uxrH+uxlH+uxrR,-uyrH+uylH+uyrR,-uzrH+uzlH+uzrR)
                # left_falcon(-uxlH+uxlR+uxrH,-uylH+uylR+uyrH,-uzlH+uzlR+uzrH)
                # right_falcon(-uxrH+uxrR+uxlH,-uyrH+uyrR+uylH,-uzrH+uzrR+uzlH)
                # left_falcon(0,0,0)
                # right_falcon(0,0,0)
                #print(np.matmul(np.matmul(By,uyrH-uylH)))
                xithy=xithy+np.multiply(dt,np.matmul(A,xithy)+np.matmul(B,uyrR-uylR)+np.matmul(B,np.matrix(uyrH-uylH)))
                xithz=xithz+np.multiply(dt,np.matmul(Az,xithz)+np.matmul(Bz,uzrR-uzlR)+np.matmul(Bz,np.matrix(uzrH-uzlH)))#+ cz
                X = np.matrix([[pos_box[0]],[vel_box[0]]])
                Y = np.matrix([[pos_box[1]],[vel_box[1]]])
                Z = np.matrix([[pos_box[2]],[vel_box[2]]])
                xix=X-Xd
                xiy=Y-Yd
                THy= xithy+THdy
                xiz=Z-Zd
                THz= xithz+THdz
                xixtilde=xixh-xix
                xiyptilde=xiyph-xiyp
                xiythtilde=xiythh-xithy
                xizptilde=xizph-xizp
                xizthtilde=xizthh-xithz
                
                pos_box_store= np.append(pos_box_store, np.matrix([[pos_box[0]],[pos_box[1]],[pos_box[2]]]), axis=1)
                pos_box_des_storeX= np.append(pos_box_des_storeX, Xd[0], axis=0)
                pos_box_des_storeY= np.append(pos_box_des_storeY, Yd[0], axis=0)
                pos_box_des_storeZ= np.append(pos_box_des_storeZ, Zd[0], axis=0) 
                LxHh_store= np.append(LxHh_store, LxHh, axis=0)
                LyrHh_store= np.append(LyrHh_store, LyrHh, axis=0)
                LylHh_store= np.append(LylHh_store, LylHh, axis=0)
                LzrHh_store= np.append(LzrHh_store, LzrHh, axis=0)
                LzlHh_store= np.append(LzlHh_store, LzlHh, axis=0)
                kzrHh_store= np.append(kzrHh_store, kzrHh, axis=1)
                kzlHh_store= np.append(kzlHh_store, kzlHh, axis=1)
                LxrR_store= np.append(LxrR_store, LxrR, axis=0)
                LxlR_store= np.append(LxlR_store, LxlR, axis=0)
                LyrR_store= np.append(LyrR_store, LyrR, axis=0)
                LylR_store= np.append(LylR_store, LylR, axis=0)
                LzrR_store= np.append(LzrR_store, LzrR, axis=0)
                LzlR_store= np.append(LzlR_store, LzlR, axis=0)
                kzrR_store= np.append(kzrR_store, kzrR, axis=1)
                kzlR_store= np.append(kzlR_store, kzlR, axis=1)
                # a=np.asarray(a)
                kzrR_store=np.asarray(kzrR_store)
                kzlR_store=np.asarray(kzlR_store)
                uxH_store= np.append(uxH_store, np.matrix(uxH), axis=0)
                uyrH_store= np.append(uyrH_store, np.matrix(uyrH), axis=0)
                uylH_store= np.append(uylH_store, np.matrix(uylH), axis=0)
                uzrH_store= np.append(uzrH_store, np.matrix(uzrH), axis=0)
                uzlH_store= np.append(uzlH_store, np.matrix(uzlH), axis=0)
                uxHh_store= np.append(uxHh_store, uxHh, axis=0)
                uyrHh_store= np.append(uyrHh_store, uyrHh, axis=0)
                uylHh_store= np.append(uylHh_store, uylHh, axis=0)
                uzrHh_store= np.append(uzrHh_store, uzrHh, axis=0)
                uzlHh_store= np.append(uzlHh_store, uzlHh, axis=0)
                uxrR_store= np.append(uxrR_store, uxrR, axis=0)
                uxlR_store= np.append(uxlR_store, uxlR, axis=0)
                uyrR_store= np.append(uyrR_store, uyrR, axis=0)
                uylR_store= np.append(uylR_store, uylR, axis=0)
                uzrR_store= np.append(uzrR_store, uzrR, axis=0)
                uzlR_store= np.append(uzlR_store, uzlR, axis=0)
                time_store= np.append(time_store, np.matrix(curr_time), axis=0)
                # print(kzrHh_store)
                # print(kzrR_store)

                ######### Publish _force ##########
                if(curr_time>tf or button_r==4 or button_l==4):
                    # print(kzrR_store.shape)
                    # print(kzrHh_store.shape)
                    scipy.io.savemat('tim_.mat', {'tim_': time_store})
                    scipy.io.savemat('pos_.mat', {'pos_': pos_box_store})
                    scipy.io.savemat('pos_des_X.mat', {'pos_des_X': pos_box_des_storeX})
                    scipy.io.savemat('pos_des_Y.mat', {'pos_des_Y': pos_box_des_storeY})
                    scipy.io.savemat('pos_des_Z.mat', {'pos_des_Z': pos_box_des_storeZ})
                    scipy.io.savemat('LxHh_.mat', {'LxHh_': LxHh_store})
                    scipy.io.savemat('LyrHh_.mat', {'LyrHh_': LyrHh_store})
                    scipy.io.savemat('LylHh_.mat', {'LylHh_': LylHh_store})
                    scipy.io.savemat('LzrHh_.mat', {'LzrHh_': LzrHh_store}) #***********
                    scipy.io.savemat('LzlHh_.mat', {'LzlHh_': LzlHh_store})
                    scipy.io.savemat('kzrR_.mat', {'kzrR_': kzrR_store})
                    scipy.io.savemat('kzlR_.mat', {'kzlR_': kzlR_store})
                    # np.save('kzrRs.npy',kzrR_store,allow_pickle=True)
                    # np.save('kzlRs.npy',kzlR_store,allow_pickle=True)
                    scipy.io.savemat('kzrHh_.mat', {'kzrHh_': kzrHh_store})
                    scipy.io.savemat('kzlHh_.mat', {'kzlHh_': kzlHh_store})
                    scipy.io.savemat('LxrR_.mat', {'LxrR_': LxrR_store})
                    scipy.io.savemat('LxlR_.mat', {'LxlR_': LxlR_store})
                    scipy.io.savemat('LyrR_.mat', {'LyrR_': LyrR_store})
                    scipy.io.savemat('LylR_.mat', {'LylR_': LylR_store})
                    scipy.io.savemat('LzrR_.mat', {'LzrR_': LzrR_store})
                    scipy.io.savemat('LzlR_.mat', {'LzlR_': LzlR_store})
                    
                    scipy.io.savemat('uxH_.mat', {'uxH_': uxH_store})
                    scipy.io.savemat('uyrH_.mat', {'uyrH_': uyrH_store})
                    scipy.io.savemat('uylH_.mat', {'uylH_': uylH_store})
                    scipy.io.savemat('uzrH_.mat', {'uzrH_': uzrH_store})
                    scipy.io.savemat('uzlH_.mat', {'uzlH_': uzlH_store})
                    scipy.io.savemat('uxHh_.mat', {'uxHh_': uxHh_store})
                    scipy.io.savemat('uyrHh_.mat', {'uyrHh_': uyrHh_store})
                    scipy.io.savemat('uylHh_.mat', {'uylHh_': uylHh_store})
                    scipy.io.savemat('uzrHh_.mat', {'uzrHh_': uzrHh_store})
                    scipy.io.savemat('uzlHh_.mat', {'uzlHh_': uzlHh_store})
                    scipy.io.savemat('uxrR_.mat', {'uxrR_': uxrR_store})
                    scipy.io.savemat('uxlR_.mat', {'uxlR_': uxlR_store})
                    scipy.io.savemat('uyrR_.mat', {'uyrR_': uyrR_store})
                    scipy.io.savemat('uylR_.mat', {'uylR_': uylR_store})
                    scipy.io.savemat('uzrR_.mat', {'uzrR_': uzrR_store})
                    scipy.io.savemat('uzlR_.mat', {'uzlR_': uzlR_store}) 
                    left_falcon(0,0,0)
                    right_falcon(0,0,0)
                    break

                
    except KeyboardInterrupt:
        pass
    # rospy.spin()

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
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)

msg_force_to_left_box =  Wrench()  # msgs
msg_force_to_right_box =  Wrench()
msg_force_to_box =  Wrench()
msg_force_to_left_falcon = falconForces()
msg_force_to_right_falcon = falconForces()

pos_falcon_left = np.array([0,0,0])  #state variables
pos_falcon_right = np.array([0,0,0])
pos_box_left = np.array([0,0,0])
pos_box_right = np.array([0,0,0])
vel_box = np.array([0,0,0])
pos_box = np.array([0,0,0])
force_fb_left_box = np.array([0,0,0]) 
force_fb_right_box = np.array([0,0,0])
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

QzR=np.diag([10,2])       #Robot parameters
QyR=np.diag([2,0])
QxR=np.diag([2,0])
PxHh=np.matrix([[0, 0],[0, 0]])
PyrHh=np.matrix([[0, 0],[0, 0]])
PylHh=np.matrix([[0, 0],[0, 0]])
PysHh=np.matrix([[0, 0],[0, 0]])
PydHh=np.matrix([[0, 0],[0, 0]])
PzrHh=np.matrix([[0, 0],[0, 0]])
PzlHh=np.matrix([[0, 0],[0, 0]])
PzsHh=np.matrix([[0, 0],[0, 0]])
PzdHh=np.matrix([[0, 0],[0, 0]])
LxHh=np.matrix([[0,0]])
LyrHh=np.matrix([[0,0]])
LylHh=np.matrix([[0,0]])
LysHh=np.matrix([[0,0]])
LydHh=np.matrix([[0,0]])
LzrHh=np.matrix([[0,0]])
LzlHh=np.matrix([[0,0]])
LzsHh=np.matrix([[2,1]])
LzdHh=np.matrix([[0,0]])
kzrHh=np.matrix([[0],[0]])
kzlHh=np.matrix([[0],[0]])
kzsHh=np.matrix([[0],[0]])
kzdHh=np.matrix([[0],[0]])
#xi_x
xixh=np.matrix([[0],[0]])
xiyph=np.matrix([[0],[0]])
xiythh=np.matrix([[0],[0]])
xizph=np.matrix([[0],[0]])
xizphih=np.matrix([[0],[0]])
xixtilde=np.matrix([[0],[0]])
xiyptilde=np.matrix([[0],[0]])
xiythtilde=np.matrix([[0],[0]])
xizptilde=np.matrix([[0],[0]])
xizphitilde=np.matrix([[0],[0]])
uxR=np.matrix([[0]]) ### not
uyrR=np.matrix([[0]]) #not
uylR=np.matrix([[0]])
uzrR=np.matrix([[0]]) #not
uzlR=np.matrix([[0]])
D=0
M=1.02
g=9.81
Ax=np.matrix([[0, 1], [0, -D/M]])
Bx=np.matrix([[0], [1/M]])
Ay=np.matrix([[0, 1], [0, -D/M]])
By=np.matrix([[0], [1/M]])
Az=np.matrix([[0, 1], [0, 0]])
Bz=np.matrix([[0], [1/M]])
cz=np.matrix([[0], [-g]])
Gammax=np.diag([100, 2])
Gammayp=np.diag([100, 2]) 
Gammayth=np.diag([100, 2]) 
Gammazp=np.diag([100, 2]) 
Gammazphi=np.diag([100, 2]) 
alphax=np.diag([10,10])
alphayp=np.diag([10,10])
alphayth=np.diag([10,10])
alphazp=np.diag([10,10])
alphazphi=np.diag([10,10])
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
TH= np.matrix([[0],[0]])#xiyth
Xd = np.matrix([[0.0],[0]])
THd=np.matrix([[0],[0]])
PHI= np.matrix([[0],[0]])#xiyth
PHId=np.matrix([[0],[0]])

if __name__ == "__main__":
    try:
        program_starts = time.time()
        prev_time=0
        prev_pos_falcon_left= np.array([0,0,0])
        prev_pos_falcon_right= np.array([0,0,0])
        #n_count=0
        while not rospy.is_shutdown():
            if(time.time()-program_starts - prev_time >= dt):
                dta=time.time() - program_starts - prev_time
                prev_time=time.time()-program_starts
                print(prev_time)
                if (prev_time<10):
                    Xd = Xd+np.matrix([[dt/40],[0]])
                    Yd = np.matrix([[0],[0]])
                    Zd = np.matrix([[0.0],[0]])
                elif (prev_time<20):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd+np.matrix([[dt/90],[0]])
                elif (prev_time<30):
                    Xd = Xd
                    Yd = Yd+np.matrix([[dt/40],[0]])
                    Zd = Zd
                elif (prev_time<40):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd-np.matrix([[dt/90],[0]])
                elif (prev_time<50):
                    Xd = Xd-np.matrix([[dt/40],[0]])
                    Yd = Yd
                    Zd = Zd
                elif (prev_time<60):
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd+np.matrix([[dt/90],[0]])
                elif (prev_time<70):
                    Xd = Xd
                    Yd = Yd-np.matrix([[dt/40],[0]])
                    Zd = Zd
                else:
                    Xd = Xd
                    Yd = Yd
                    Zd = Zd-np.matrix([[dt/90],[0]])
                #Xd = np.matrix([[1.7*np.math.cos(omeg*prev_time)],[0]])
                #Yd = np.matrix([[1.7*np.math.sin(omeg*prev_time)],[0]])
                xix = X-Xd
                xiyp = Y-Yd
                xizp = Z-Zd
                xith = TH-THd
                xiphi = PHI-PHId
                exl=pos_box[0]-pos_falcon_left[0]*50
                uxlH=-8*(exl)-10*vel_box[0]
                eyl=pos_box[1]-(pos_falcon_left[2]-0.12)*(50)
                uylH=-5*(eyl)-10*vel_box[1]
                ezl=pos_box[2]-pos_falcon_left[1]*50
                uzlH=-10*(ezl)-2*vel_box[2]
                flH=np.matrix([[uxlH],[uylH],[uzlH]])
                exr=pos_box[0]-pos_falcon_right[0]*50
                uxrH=-8*(exr)-10*vel_box[0]
                eyr=pos_box[1]-(pos_falcon_right[2]-0.12)*(50)
                uyrH=-5*(eyr)-10*vel_box[1]
                ezr=pos_box[2]-pos_falcon_right[1]*50
                uzrH=-10*(ezr)-2*vel_box[2]
                frH=np.matrix([[uxrH],[uyrH],[uzrH]])
                uxH=uxlH+uxrH
                uysH=uylH+uyrH#...
                uydH=uyrH-uylH#.....
                uzsH=uzlH+uzrH#...
                uzdH=uzrH-uzlH#.....
                a = Ax-(np.matmul(Bx,LxHh))
                q = QxR+np.matmul(LxHh.T,LxHh)   #took transpose
                b = Bx
                r = 1
                PxR = linalg.solve_continuous_are(a, b, q, r, e=None, s=None, balanced=True)
                LxR=np.matmul(Bx.T,PxR)
                uxR = -np.matmul(LxR,xix)
                if(uxR>0):
                    LxlR=LxR
                    LxrR=np.matrix([[0,0]])
                    uxlR=uxR
                    uxrR=np.matrix([[0]])
                else:
                    LxrR=LxR
                    LxlR=np.matrix([[0, 0]])
                    uxrR=uxR 
                    uxlR=np.matrix([[0]])
                a = Ay-(np.matmul(By,LysHh))
                q = QyR+np.matmul(LysHh.T,LysHh)   #took transpose
                b = By
                r = 1
                #print(a)
                #print(q)
                PyR = linalg.solve_continuous_are(a, b, q, r)
                LyR=np.matmul(By.T,PyR)
                LyrR=np.multiply(0.5,LyR+LysHh)-LyrHh
                LylR=LyR-LyrR
                uyrR = -np.matmul(LyrR,xiyp)
                uylR = -np.matmul(LylR,xiyp)
                a = Az-(np.matmul(Bz,LzsHh))
                q = QzR+np.matmul(LzsHh.T,LzsHh)   #took transpose**********************
                b = Bz
                r = 1
                #print(a)
                #print(q)
                PzR = np.matrix([[3, 1],[1, 5]])#np.matrix([[3, 1],[1, 5]]) #linalg.solve_continuous_are(a, b, q, r) #*********
                #PzR = linalg.solve_discrete_are(a, b, q, r, e=None, s=None, balanced=True) 
                
                #print(PzR)
                LzR=np.matmul(Bz.T,PzR)
                LzrR=np.multiply(0.5,LzR+LzsHh)-LzrHh
                LzlR=LzR-LzrR
                #print(LzlR)
                kzR=np.matrix([[0],-kzsHh[1]-(M*M*g)])
                #print(kzR)
                kzrR=np.multiply(0.5,kzR+kzsHh)-kzrHh
                kzlR=kzR-kzrR
                uzrR = -np.matmul(LzrR,xizp)-np.matmul(Bz.T,kzrR)#..
                uzlR = -np.matmul(LzlR,xizp)-np.matmul(Bz.T,kzlR)
                #print(uxlR)
                uxHh= -np.matmul(LxHh,xix)
                uysHh= -np.matmul(LysHh,xiyp)
                uydHh= -np.matmul(LydHh,xiyp)
                uzsHh= -np.matmul(LzsHh,xizp)-np.matmul(Bz.T,kzsHh)
                uzdHh= -np.matmul(LzdHh,xizp)-np.matmul(Bz.T,kzdHh)
                xixh= xixh+np.multiply(dt,np.matmul(Ax,xixh)+np.multiply(Bx,(uxrR+uxlR))+np.multiply(Bx,uxHh)-np.matmul(Gammax,xixtilde))
                xiyph= xiyph+np.multiply(dt,np.matmul(Ay,xiyph)+np.multiply(By,(uyrR+uylR))+np.multiply(By,uysHh)-np.matmul(Gammayp,xiyptilde))
                xiythh=xiythh+np.multiply(dt,np.matmul(Ay,xiythh)+np.matmul(By,uyrR-uylR)+np.matmul(By,uydHh)-np.matmul(Gammayth,xiythtilde))
                xizph= xizph+np.multiply(dt,np.matmul(Az,xizph)+np.multiply(Bz,(uzrR+uzlR))+np.multiply(Bz,uzsHh)+cz-np.matmul(Gammazp,xizptilde))
                xizphih=xizphih+np.multiply(dt,np.matmul(Az,xizphih)+np.matmul(Bz,uzrR-uzlR)+np.matmul(Bz,uzdHh)+cz-np.matmul(Gammazphi,xizphitilde))
                # if (prev_time>3.4 and prev_time<3.55):
                #     print(xiythh)
                #     print(xizphih)
                PxHh= PxHh+np.multiply(dt,np.matmul(np.matmul(alphax,xixtilde),xix.T))
                PysHh= PysHh+np.multiply(dt,np.matmul(np.matmul(alphayp,xiyptilde),xiyp.T))
                PydHh= PydHh+np.multiply(dt,np.matmul(np.matmul(alphayth,xiythtilde),xiyp.T))
                PzsHh= PzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,xizptilde),xizp.T))
                PzdHh= PzdHh+np.multiply(dt,np.matmul(np.matmul(alphazphi,xizphitilde),xizp.T))
                kzsHh= kzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,np.matmul(Bz,Bz.T)),xizptilde))
                kzdHh= kzdHh+np.multiply(dt,np.matmul(np.matmul(alphazphi,np.matmul(Bz,Bz.T)),xizphitilde))
                PxHh[0,1]=PxHh[1,0]
                PysHh[0,1]=PysHh[1,0]
                PydHh[0,1]=PydHh[1,0]
                PzsHh[0,1]=PzsHh[1,0]
                PzdHh[0,1]=PzdHh[1,0]
                kzsHh[0] = 0
                kzdHh[0] = 0
                LxHh=np.matmul(Bx.T,PxHh)
                LysHh=np.matmul(By.T,PysHh)
                LydHh=np.matmul(By.T,PydHh)
                LzsHh=np.matmul(Bz.T,PzsHh)
                LzdHh=np.matmul(Bz.T,PzdHh)
                LyrHh=np.multiply(0.5,LysHh+LydHh)
                LylHh=LysHh-LyrHh
                LzrHh=np.multiply(0.5,LzsHh+LzdHh)
                LzlHh=LzsHh-LzrHh
                kzrHh=np.multiply(0.5,kzsHh+kzdHh)
                kzlHh=kzsHh-kzrHh
                uyrHh= -np.matmul(LyrHh,xiyp)
                uylHh= -np.matmul(LylHh,xiyp)
                uzrHh= -np.matmul(LzrHh,xizp)-np.matmul(Bz.T,kzrHh)
                uzlHh= -np.matmul(LzlHh,xizp)-np.matmul(Bz.T,kzlHh)
                #print(uzrHh+uzrR-(uzlHh+uzlR))
                #a=Ax-np.matmul(Bx,LxR)
                # QxHh=-(np.matmul(a.T,PxHh)+np.matmul(PxHh,a)-np.matmul(np.matmul(PxHh,Bx),np.matmul(Bx.T,PxHh)))
                # QxHh[0,1]=0
                # QxHh[1,0]=0
                # #QxR=Qxsum-QxHh###################
                # #if(abs(flH[0]+frH[0])<20):
                #     #flR=np.matrix([[0],[0],[0]])
                #     #frR=np.matrix([[0],[0],[0]])
                # a=Ay-np.matmul(By,LyR)
                # QysHh=-(np.matmul(a.T,PysHh)+np.matmul(PysHh,a)-np.matmul(np.matmul(PysHh,By),np.matmul(By.T,PysHh)))
                #QysHh[0,1]=0
                #QysHh[1,0]=0
                #QyR=Qysum-QysHh
                flb=flH#+flR
                frb=frH#+frR
                #frot=frb-flb

                left_box(flb[0],flb[1],flb[2])#
                right_box(frb[0],frb[1],frb[2]) #
                #print("Desired state: {}, {}    Actual states: {}, {}".format(Xd[0],Yd[0],pos_box[0],pos_box[1]))
                left_falcon(-flH[0]+uxlR,-flH[2]+uzlR,-flH[1]+uylR)#(-0,0,0)#(-flH[0]+flR[0]+frb[0],-flH[1]+flR[1]+frb[1],-flH[2]+flR[2]+frb[2])
                right_falcon(-frH[0]+uxrR,-frH[2]+uzrR,-frH[1]+uyrR)#(0,0,0)#(-frH[0]+frR[0]+flb[0],-frH[1]+frR[1]+flb[1],-frH[2]+frR[2]+flb[2])
                #print(np.matmul(np.matmul(By,uyrH-uylH)))
                xith=xith+np.multiply(dt,np.matmul(Ay,xith)+np.matmul(By,uyrR-uylR)+np.matmul(By,np.matrix(uydH)))
                xiphi=xiphi+np.multiply(dt,np.matmul(Az,xiphi)+np.matmul(Bz,uzrR-uzlR)+cz+np.matmul(Bz,np.matrix(uzdH))) #....
                X = np.matrix([[pos_box[0]],[vel_box[0]]])
                Y = np.matrix([[pos_box[1]],[vel_box[1]]])
                Z = np.matrix([[pos_box[2]],[vel_box[2]]])
                xix=X-Xd
                xiyp=Y-Yd
                xizp=Z-Zd
                TH= xith+THd
                PHI= xiphi+PHId
                xixtilde=xixh-xix
                xiyptilde=xiyph-xiyp
                xizptilde=xizph-xizp
                xiythtilde=xiythh-xith
                xizphitilde=xizphih-xiphi
                
                pos_box_store= np.append(pos_box_store, np.matrix([[pos_box[0]],[pos_box[1]],[pos_box[2]]]), axis=1)
                vel_box_store= np.append(vel_box_store, np.matrix([[vel_box[0]],[vel_box[1]],[vel_box[2]]]), axis=1)
                #orientation_box_store= np.append(orientation_box_store, np.matrix([[orientation_box[0]],[orientation_box[1]],[orientation_box[2]],[orientation_box[3]]]), axis=1)
                #pos_box_des_store= np.append(pos_box_des_store, np.array([Xd[0],Yd[0],Zd[0]]), axis=1) 
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
                #----
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
                time_store= np.append(time_store, np.matrix(prev_time), axis=0)
		print("======== DEBUG ========")
		print("xix (X error):", xix.T)
		print("LxHh (gain):", LxHh)
		print("uxHh (assist force X):", uxHh)
		print("Right Falcon force sent (X):", -frH[0]+uxrR)
		print("Right Falcon raw error force (frH[0]):", frH[0])
		print("=======================")

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
                    scipy.io.savemat('LxHh_.mat', {'LxHh_': LxHh_store})
                    scipy.io.savemat('LyrHh_.mat', {'LyrHh_': LyrHh_store})
                    scipy.io.savemat('LylHh_.mat', {'LylHh_': LylHh_store})
                    scipy.io.savemat('LzrHh_.mat', {'LzrHh_': LzrHh_store}) #***********
                    scipy.io.savemat('LzlHh_.mat', {'LzlHh_': LzlHh_store})
                    scipy.io.savemat('kzrHh_.mat', {'kzrHh_': kzrHh_store})
                    scipy.io.savemat('kzlHh_.mat', {'kzlHh_': kzlHh_store})

                    scipy.io.savemat('LxrR_.mat', {'LxrR_': LxrR_store})
                    scipy.io.savemat('LxlR_.mat', {'LxlR_': LxlR_store})
                    scipy.io.savemat('LyrR_.mat', {'LyrR_': LyrR_store})
                    scipy.io.savemat('LylR_.mat', {'LylR_': LylR_store})
                    scipy.io.savemat('LzrR_.mat', {'LzrR_': LzrR_store})
                    scipy.io.savemat('LzlR_.mat', {'LzlR_': LzlR_store})
                    scipy.io.savemat('kzrR_.mat', {'kzrR_': kzrR_store})
                    scipy.io.savemat('kzlR_.mat', {'kzlR_': kzlR_store})
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

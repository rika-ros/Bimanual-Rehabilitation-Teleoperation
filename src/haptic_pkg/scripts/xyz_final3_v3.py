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


time_list = []
pos_box_list = []
vel_box_list = []
zd_list = []

button_r=0
button_l=0
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
tf= 20

k_joy = 100
kp = 2

#intialize
X = np.matrix([[pos_box[0]],[vel_box[0]]])
Y = np.matrix([[pos_box[1]],[vel_box[1]]])
Z = np.matrix([[pos_box[2]],[vel_box[2]]])
Zd = np.matrix([[0.0],[0]])
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
                if (curr_time<10):
                    Zd = Zd+np.matrix([[dt/10],[0]])
                    test =0
                    print("Lift Up")
                    #print(curr_time)
                elif (curr_time<20):
                    Zd = Zd-np.matrix([[dt/10],[0]])
                    test=1
                    print("Bring down")
                
                xizp = Z-Zd
                xithz = THz-THdz
                uxlH=np.matrix([[0]])
                uylH=np.matrix([[0]])
                ezl=pos_box[2]-pos_falcon_left[2]*50
                uzlH=-5*(ezl)-1*vel_box[2]

                
                uxrH=np.matrix([[0]])
                uyrH=np.matrix([[0]])
                ezr=pos_box[2]-pos_falcon_right[2]*50
                uzrH=-5*(ezr)-10*vel_box[2] 

                uxH=uxlH+uxrH
                uysH=uylH+uyrH
                uydH=uyrH-uylH
                uzsH=uzlH+uzrH
                uzdH=uzrH-uzlH

               
                b = Bz
                
# Use stable matrix 'a' directly
		a = Az
		q = QzR  # simple diagonal matrix, already defined as np.diag([2, 0])
		r = 1

# Ensure q is well-conditioned (add small value to diagonal if needed)
		if np.linalg.matrix_rank(q) < 2:
			q = q.astype(float) + np.diag([1e-4, 1e-4])


# Solve Riccati equation safely
		try:
    			PzR = linalg.solve_continuous_are(a, Bz, q, r)
		except Exception as e:
    			rospy.logwarn("Riccati equation solve failed: %s", e)
    		PzR = np.eye(2)  # fallback to identity if it fails

		LzR = np.matmul(Bz.T, PzR)
		LzrR = np.multiply(0.5, LzR + LzsHh) - LzrHh
		LzlR = LzR - LzrR

 		
                LzR=np.matmul(Bz.T,PzR)
                LzrR=np.multiply(0.5,LzR+LzsHh)-LzrHh
                LzlR=LzR-LzrR
                # #-----
                kzR=np.matrix([[0],-kzsHh[1]-(M*M*g)])
                kzrR=np.multiply(0.5,kzR+kzsHh)-kzrHh
                # print(kzrHh)
                # print(kzrR)
                kzlR=kzR-kzrR 
                uzrR = -np.matmul(LzrR,xizp)#-np.matmul(Bz.T,kzrR)
                uzlR = -np.matmul(LzlR,xizp)#-np.matmul(Bz.T,kzlR)

                uxrR=np.matrix([[0]])
                uxlR=np.matrix([[0]])
                uyrR=np.matrix([[0]])
                uylR=np.matrix([[0]])

                uzsHh= -np.matmul(LzsHh,xizp)#-np.matmul(Bz.T,kzsHh)
                uzdHh= -np.matmul(LzdHh,xizp)#-np.matmul(Bz.T,kzdHh)
                xizph= xizph+np.multiply(dt,np.matmul(Az,xizph)+np.matmul(Bz,(uzrR+uzlR))+np.matmul(Bz,uzsHh)-np.matmul(Gammazp,xizptilde))#+cz
                xizthh=xizthh+np.multiply(dt,np.matmul(Az,xizthh)+np.matmul(Bz,(uzrR-uzlR))+np.matmul(Bz,uzdHh)-np.matmul(Gammazth,xizthtilde))#+cz
                PzsHh= PzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,xizptilde),xizp.T))
                PzdHh= PzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,xizthtilde),xizp.T))
                # #-===============
                kzsHh= kzsHh+np.multiply(dt,np.matmul(np.matmul(alphazp,np.matmul(Bz,Bz.T)),xizptilde))
                kzdHh= kzdHh+np.multiply(dt,np.matmul(np.matmul(alphazth,np.matmul(Bz,Bz.T)),xizthtilde))
                PzsHh[0,1]=PzsHh[1,0]
                PzdHh[0,1]=PzdHh[1,0]
                kzsHh[0] = 0
                kzdHh[0] = 0
                LzsHh=np.matmul(Bz.T,PzsHh)
                LzdHh=np.matmul(Bz.T,PzdHh)
                LzrHh=np.multiply(0.5,LzsHh+LzdHh)
                LzlHh=LzsHh-LzrHh
                kzrHh=np.multiply(0.5,kzsHh+kzdHh)
                kzlHh=kzsHh-kzrHh
                uzrHh= -np.matmul(LzrHh,xizp)-np.matmul(Bz.T,kzrHh)
                uzlHh= -np.matmul(LzlHh,xizp)-np.matmul(Bz.T,kzlHh)
                
                #============-

                left_box(uxlH,uylH,uzlH)
                right_box(uxrH,uyrH,uzrH) 
                if test == 1:  # Lowering phase (10-20s)
            # Falcon should resist the box falling down - apply upward force
                        z_force_left  = -uzlH + uzrH + uzlR
                        z_force_right = -uzrH + uzlH + uzrR

                else:  # Lifting phase (0-10s)
    # Falcon should resist your upward motion - apply downward force
                        z_force_left  = -uzlH + uzrH - uzlR
                        z_force_right = -uzrH + uzlH - uzrR

# Compute full 3D force for both falcons
                        x_force_left  = -uxlH + uxrH + uxlR
                        y_force_left  = -uylH + uyrH + uylR

                        x_force_right = -uxrH + uxlH + uxrR
                        y_force_right = -uyrH + uylH + uyrR

# Send forces to Falcons
                left_falcon(x_force_left, y_force_left, z_force_left)
                right_falcon(x_force_right, y_force_right, z_force_right)

    			
                # left_falcon(-uxlH+uxlR+uxrH,-uylH+uylR+uyrH,-uzlH+uzlR+uzrH)
                # right_falcon(-uxrH+uxrR+uxlH,-uyrH+uyrR+uylH,-uzrH+uzrR+uzlH)
                # left_falcon(0,0,0)
                # right_falcon(0,0,0)
                #print(np.matmul(np.matmul(By,uyrH-uylH)))
                xithz=xithz+np.multiply(dt,np.matmul(Az,xithz)+np.matmul(Bz,uzrR-uzlR)+np.matmul(Bz,np.matrix(uzrH-uzlH)))#+ cz
                X = np.matrix([[pos_box[0]],[vel_box[0]]])
                Y = np.matrix([[pos_box[1]],[vel_box[1]]])
                Z = np.matrix([[pos_box[2]],[vel_box[2]]])
                xiz=Z-Zd
                THz= xithz+THdz
                xizptilde=xizph-xizp
                xizthtilde=xizthh-xithz
                time_list.append(curr_time)
		pos_box_list.append(Z[0, 0])  # Current Z position of box
		vel_box_list.append(Z[1, 0])  # Current Z velocity of box
		zd_list.append(Zd[0, 0])      # Desired Z trajectory

                ######### Publish _force ##########
                if(curr_time>tf or button_r==4 or button_l==4):
                    left_falcon(0,0,0)
                    right_falcon(0,0,0)
                    break
		data_dict = {
		    'time': np.array(time_list),
		    'pos_box': np.array(pos_box_list),
		    'vel_box': np.array(vel_box_list),
		    'zd_traj': np.array(zd_list)
		}
		savemat('xyz_final3_saved.mat', data_dict)
		rospy.loginfo("Saved data to xyz_final3_saved.mat")

                
    except KeyboardInterrupt:
        pass
    # rospy.spin(), explain what we are supposed to feel from the falcons, when we run this code and roslaunch a box it just goes up and the falcons vibrate

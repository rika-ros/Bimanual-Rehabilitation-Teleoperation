#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
from geometry_msgs.msg import Wrench
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates
from scipy.linalg import solve_continuous_are

# ——— PARAMETERS ———
M    = 1.02
Kh, Dh = 5.0, 1.0
sens   = np.array([20.,20.,50.])
Tseg   = 10.0
Lx, Ly, Lz = 0.5, 0.5, 0.5

# ——— LQR GAINS ———
A = np.array([[0,1],[0,0]])
B = np.array([[0],[1.0/M]])
QxR = np.diag([0.5,0.])
QyR = np.diag([0.5,0.])
QzR = np.diag([2.0,0.])
Rmat = np.array([[1.0]])

Px = solve_continuous_are(A,B,QxR,Rmat)
Py = solve_continuous_are(A,B,QyR,Rmat)
Pz = solve_continuous_are(A,B,QzR,Rmat)

LxR = B.T.dot(Px)
LyR = B.T.dot(Py)
LzR = B.T.dot(Pz)

# ——— STATE ———
posFalL = np.zeros(3); posFalR = np.zeros(3)
pos_box  = np.zeros(3); vel_box  = np.zeros(3)
phase = 0

# ——— ROS SETUP ———
rospy.init_node('xyz_trajectory_aan_full')
pub_box_L = rospy.Publisher('/force_left', Wrench,       queue_size=1)
pub_box_R = rospy.Publisher('/force_right',Wrench,       queue_size=1)
pubFalL   = rospy.Publisher('/falconForce1', falconForces,queue_size=1)
pubFalR   = rospy.Publisher('/falconForce', falconForces, queue_size=1)

def box_cb(msg):
    global pos_box,vel_box
    try:
        i = msg.name.index('floating_box::base_link')
        p=msg.pose[i].position; t=msg.twist[i].linear
        pos_box[:] = [p.x,p.y,p.z]; vel_box[:] = [t.x,t.y,t.z]
    except: pass

def joyL_cb(m): posFalL[:] = m.axes
def joyR_cb(m): posFalR[:] = m.axes

rospy.Subscriber('/gazebo/link_states',LinkStates,box_cb)
rospy.Subscriber('/falcon/joystick', Joy, joyL_cb)
rospy.Subscriber('/falcon/joystick1',Joy, joyR_cb)

rate = rospy.Rate(100)
t0 = time.time()
rospy.loginfo("Starting full XYZ AAN trajectory...")

while not rospy.is_shutdown():
    t = time.time() - t0

    # phase detection & logging
    if t< Tseg and phase!=1:
        rospy.loginfo("Phase 1: +X"); phase=1
    elif Tseg<=t<2*Tseg and phase!=2:
        rospy.loginfo("Phase 2: +Y"); phase=2
    elif 2*Tseg<=t<3*Tseg and phase!=3:
        rospy.loginfo("Phase 3: +Z"); phase=3
    elif t>=3*Tseg and phase!=4:
        rospy.loginfo_once("Trajectory complete"); phase=4

    # desired pos & vel
    def des(t,L,phase_id):
        tt = t - (phase_id-1)*Tseg
        if 0<=tt<Tseg: return L*(tt/Tseg), L/Tseg
        else:           return (L,0.)
    xd, xdot = des(t,Lx,1)
    yd, ydot = des(t,Ly,2)
    zd, zdot = des(t,Lz,3)

    # human spring–damper
    err_h = pos_box - sens*(posFalL+posFalR)/2
    uH    = -Kh*err_h - Dh*vel_box

    # LQR assists
    ex = np.array([pos_box[0]-xd, vel_box[0]])
    ey = np.array([pos_box[1]-yd, vel_box[1]])
    ez = np.array([pos_box[2]-zd, vel_box[2]])

    uAx = -float(LxR.dot(ex))
    uAy = -float(LyR.dot(ey))
    uAz = -float(LzR.dot(ez))

    # total forces
    f_tot = uH + np.array([uAx,uAy,uAz])

    # split & publish
    fL = f_tot*0.5; fR = f_tot*0.5
    wL, wR = Wrench(), Wrench()
    wL.force.x,wL.force.y,wL.force.z = fL
    wR.force.x,wR.force.y,wR.force.z = fR
    pub_box_L.publish(wL); pub_box_R.publish(wR)

    mL,mR = falconForces(), falconForces()
    mL.X,mL.Y,mL.Z = fL; mR.X,mR.Y,mR.Z = fR
    pubFalL.publish(mL); pubFalR.publish(mR)

    rate.sleep()


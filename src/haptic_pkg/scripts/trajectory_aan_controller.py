#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from geometry_msgs.msg import Wrench
from gazebo_msgs.msg import LinkStates

# Node init
rospy.init_node('trajectory_aan_controller', anonymous=True)

# Publishers
pub_box_force_left  = rospy.Publisher('/force_left',  Wrench, queue_size=1)
pub_box_force_right = rospy.Publisher('/force_right', Wrench, queue_size=1)

# State variables
pos_box = np.zeros(3)
vel_box = np.zeros(3)
start_time = time.time()

# AAN gains
Kp = 50.0    # spring stiffness
Kd = 5.0     # damping coefficient

def get_desired(t):
    # 0-10s: move 0->1 m along X
    if t < 10.0:
        return np.array([1.0 * (t / 10.0), 0.0, 0.0])
    # 10-20s: move 1 m along Y
    elif t < 20.0:
        if abs(t - 10.0) < 0.01:
            rospy.loginfo("Segment 2: Move along Y direction")
        return np.array([1.0, 1.0 * ((t - 10.0) / 10.0), 0.0])
    # 20-30s: move 0.5 m along Z
    elif t < 30.0:
        if abs(t - 20.0) < 0.01:
            rospy.loginfo("Segment 3: Move along Z direction")
        return np.array([1.0, 1.0, 0.5 * ((t - 20.0) / 10.0)])
    else:
        rospy.loginfo_once("Trajectory complete")
        return np.array([1.0, 1.0, 0.5])

def box_state_cb(msg):
    global pos_box, vel_box
    try:
        idx = msg.name.index('floating_box::base_link')
        pose  = msg.pose[idx]
        twist = msg.twist[idx]
        pos_box = np.array([pose.position.x,
                            pose.position.y,
                            pose.position.z])
        vel_box = np.array([twist.linear.x,
                            twist.linear.y,
                            twist.linear.z])
    except ValueError:
        pass

def publish_wrench(pub, force):
    msg = Wrench()
    msg.force.x = force[0]
    msg.force.y = force[1]
    msg.force.z = force[2]
    pub.publish(msg)

# Subscribe to box state
rospy.Subscriber('/gazebo/link_states', LinkStates, box_state_cb)

rate = rospy.Rate(100)  # 100 Hz
rospy.loginfo("Starting 3D trajectory AAN controller")

while not rospy.is_shutdown():
    t = time.time() - start_time
    desired     = get_desired(t)
    error       = desired - pos_box
    assist_force = Kp * error - Kd * vel_box

    force_left  = assist_force * 0.5
    force_right = assist_force * 0.5

    publish_wrench(pub_box_force_left,  force_left)
    publish_wrench(pub_box_force_right, force_right)

    rate.sleep()


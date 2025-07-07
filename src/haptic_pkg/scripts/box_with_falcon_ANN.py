#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates
import numpy as np

# Initialize ROS node
rospy.init_node("Box_control_AAN", anonymous=True)

# Publishers
pub_left_force_box = rospy.Publisher("/force_left", Wrench, queue_size=1)
pub_right_force_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
pub_left_force_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)
pub_right_force_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)

# Global variables
pos_falcon_left = np.array([0.0, 0.0, 0.0])
pos_falcon_right = np.array([0.0, 0.0, 0.0])
pos_box_left = np.array([0.0, 0.0, 0.0])
pos_box_right = np.array([0.0, 0.0, 0.0])
force_box_left = np.array([0.0, 0.0, 0.0])
force_box_right = np.array([0.0, 0.0, 0.0])
vel_box = np.array([0.0, 0.0, 0.0])

# Message containers
msg_box_force_left = Wrench()
msg_box_force_right = Wrench()
msg_falcon_force_left = falconForces()
msg_falcon_force_right = falconForces()

# Sensitivity tuning
sensi_xy = 20
sensi_z = 50

# AAN gain
K_gain = 50  # tune this

def apply_deadzone(val, threshold=0.05):
    return val if abs(val) > threshold else 0.0

# Publishers
def pub_box_force_left(fx, fy, fz=0):
    msg_box_force_left.force.x = fx
    msg_box_force_left.force.y = fy
    msg_box_force_left.force.z = fz
    pub_left_force_box.publish(msg_box_force_left)

def pub_box_force_right(fx, fy, fz=0):
    msg_box_force_right.force.x = fx
    msg_box_force_right.force.y = fy
    msg_box_force_right.force.z = fz
    pub_right_force_box.publish(msg_box_force_right)

def pub_falcon_force_left(fx, fy, fz=0):
    msg_falcon_force_left.X = fx
    msg_falcon_force_left.Y = fy
    msg_falcon_force_left.Z = fz
    pub_left_force_falcon.publish(msg_falcon_force_left)

def pub_falcon_force_right(fx, fy, fz=0):
    msg_falcon_force_right.X = fx
    msg_falcon_force_right.Y = fy
    msg_falcon_force_right.Z = fz
    pub_right_force_falcon.publish(msg_falcon_force_right)

# Subscribers
def callback_pos_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = data.axes

def callback_pos_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = data.axes

def callback_force_box_left(data):
    global force_box_left
    force_box_left = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]

def callback_force_box_right(data):
    global force_box_right
    force_box_right = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]

def box_pos_cb(data):
    global pos_box_left, pos_box_right, vel_box
    try:
        index = data.name.index("floating_box::base_link")
        box_pose = data.pose[index]
        box_twist = data.twist[index]
        pos_box_left = [box_pose.position.x - 0.25, box_pose.position.y, box_pose.position.z]
        pos_box_right = [box_pose.position.x + 0.25, box_pose.position.y, box_pose.position.z]
        vel_box = [box_twist.linear.x, box_twist.linear.y, box_twist.linear.z]
    except ValueError:
        rospy.logwarn_throttle(5, "Model 'floating_box::base_link' not found in /gazebo/link_states.")

# ROS Topic Subscriptions
rospy.Subscriber("/falcon/joystick", Joy, callback_pos_falcon_left, queue_size=1)
rospy.Subscriber("/falcon/joystick1", Joy, callback_pos_falcon_right, queue_size=1)
rospy.Subscriber("/left/force_feedback", WrenchStamped, callback_force_box_left, queue_size=1)
rospy.Subscriber("/right/force_feedback", WrenchStamped, callback_force_box_right, queue_size=1)
rospy.Subscriber("/gazebo/link_states", LinkStates, box_pos_cb, queue_size=1)

# Control Loop
if __name__ == "__main__":
    try:
        prev_time = 0
        while not rospy.is_shutdown():
            if time.time() - prev_time >= 0.01:
                prev_time = time.time()

                ### Human inputs from Falcon ###
                uh_lx = sensi_xy * apply_deadzone(pos_falcon_left[0])
                uh_ly = sensi_xy * apply_deadzone(pos_falcon_left[1])
                uh_lz = sensi_z * apply_deadzone(pos_falcon_left[2])

                uh_rx = sensi_xy * apply_deadzone(pos_falcon_right[0])
                uh_ry = sensi_xy * apply_deadzone(pos_falcon_right[1])
                uh_rz = sensi_z * apply_deadzone(pos_falcon_right[2])

                ### Current vs Desired Position ###
                px_l, py_l, pz_l = pos_box_left
                px_r, py_r, pz_r = pos_box_right

                dx_l = pos_falcon_left[0]
                dy_l = pos_falcon_left[1]
                dz_l = pos_falcon_left[2]

                dx_r = pos_falcon_right[0]
                dy_r = pos_falcon_right[1]
                dz_r = pos_falcon_right[2]

                ### AAN Forces: Robot assistance + Human estimate ###
                fx_l = -K_gain * (px_l - dx_l) + uh_lx
                fy_l = -K_gain * (py_l - dy_l) + uh_ly
                fz_l = -K_gain * (pz_l - dz_l) + uh_lz

                fx_r = -K_gain * (px_r - dx_r) + uh_rx
                fy_r = -K_gain * (py_r - dy_r) + uh_ry
                fz_r = -K_gain * (pz_r - dz_r) + uh_rz

                # Send force to box
                pub_box_force_left(fx_l, fy_l, fz_l)
                pub_box_force_right(fx_r, fy_r, fz_r)

                # Send force feedback to Falcon
                pub_falcon_force_left(*force_box_left)
                pub_falcon_force_right(*force_box_right)

    except KeyboardInterrupt:
        pass


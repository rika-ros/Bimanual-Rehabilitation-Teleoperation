#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import LinkStates
from scipy.io import savemat

rospy.init_node("Box_control", anonymous=True)

# Publishers
force_to_left_box = rospy.Publisher("/force_left", Wrench, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)

# Messages
msg_force_left = Wrench()
msg_force_right = Wrench()
msg_force_left_falcon = falconForces()
msg_force_right_falcon = falconForces()

# Globals
pos_falcon_left = np.array([0.0, 0.0, 0.0])
pos_falcon_right = np.array([0.0, 0.0, 0.0])
pos_box = np.array([0.0, 0.0, 0.0])
vel_box = np.array([0.0, 0.0, 0.0])
omeg = 2 * np.pi / 60

# Subscribers

def cb_falcon_left(data):
    global pos_falcon_left
    pos_falcon_left = np.array([data.axes[0], data.axes[1], data.axes[2]])

def cb_falcon_right(data):
    global pos_falcon_right
    pos_falcon_right = np.array([data.axes[0], data.axes[1], data.axes[2]])

def cb_box(data):
    global pos_box, vel_box
    pos_box = np.array([
        data.pose[1].position.x,
        data.pose[1].position.y,
        data.pose[1].position.z - 0.35
    ])
    vel_box = np.array([
        data.twist[1].linear.x,
        data.twist[1].linear.y,
        data.twist[1].linear.z
    ])

rospy.Subscriber("/falcon/joystick", Joy, cb_falcon_right)
rospy.Subscriber("/falcon/joystick1", Joy, cb_falcon_left)
rospy.Subscriber("/gazebo/link_states", LinkStates, cb_box)

# Force application helpers

def left_box(fx, fy, fz):
    msg_force_left.force.x = fx
    msg_force_left.force.y = fy
    msg_force_left.force.z = fz
    force_to_left_box.publish(msg_force_left)

def right_box(fx, fy, fz):
    msg_force_right.force.x = fx
    msg_force_right.force.y = fy
    msg_force_right.force.z = fz
    force_to_right_box.publish(msg_force_right)

def left_falcon(fx, fy, fz):
    msg_force_left_falcon.X = fx
    msg_force_left_falcon.Y = fy
    msg_force_left_falcon.Z = fz
    force_to_left_falcon.publish(msg_force_left_falcon)

def right_falcon(fx, fy, fz):
    msg_force_right_falcon.X = fx
    msg_force_right_falcon.Y = fy
    msg_force_right_falcon.Z = fz
    force_to_right_falcon.publish(msg_force_right_falcon)

# Logging
pos_box_store = np.empty((3, 0))
vel_box_store = np.empty((3, 0))
pos_des_store_x = np.empty((0, 1))
pos_des_store_y = np.empty((0, 1))

dt = 0.01
tf = 70

if __name__ == "__main__":
    try:
        program_start = time.time()
        prev_time = 0
        while not rospy.is_shutdown():
            now = time.time()
            if now - prev_time >= dt:
                prev_time = now
                t = now - program_start

                # Desired trajectory (circle)
                Xd = np.matrix([[1.7 * np.cos(omeg * t)], [0]])
                Yd = np.matrix([[1.7 * np.sin(omeg * t)], [0]])

                # Compute force from falcon user input (no assist)
                exl = pos_box[0] - pos_falcon_left[0] * 50
                eyl = pos_box[1] - pos_falcon_left[1] * 50
                uxlH = -8 * exl - 10 * vel_box[0]
                uylH = -5 * eyl - 10 * vel_box[1]

                exr = pos_box[0] - pos_falcon_right[0] * 50
                eyr = pos_box[1] - pos_falcon_right[1] * 50
                uxrH = -8 * exr - 10 * vel_box[0]
                uyrH = -5 * eyr - 10 * vel_box[1]

                # Apply forces
                left_box(uxlH, uylH, 0)
                right_box(uxrH, uyrH, 0)
                left_falcon(-uxlH, -uylH, 0)
                right_falcon(-uxrH, -uyrH, 0)

                # Store data
                pos_box_store = np.append(pos_box_store, np.matrix([[pos_box[0]], [pos_box[1]], [pos_box[2]]]), axis=1)
                vel_box_store = np.append(vel_box_store, np.matrix([[vel_box[0]], [vel_box[1]], [vel_box[2]]]), axis=1)
                pos_des_store_x = np.append(pos_des_store_x, Xd[0], axis=0)
                pos_des_store_y = np.append(pos_des_store_y, Yd[0], axis=0)

                if t > tf:
                    savemat('pos_.mat', {'pos_': pos_box_store})
                    savemat('vel_.mat', {'vel_': vel_box_store})
                    savemat('pos_des_X.mat', {'pos_des_X': pos_des_store_x})
                    savemat('pos_des_Y.mat', {'pos_des_Y': pos_des_store_y})
                    left_falcon(0, 0, 0)
                    right_falcon(0, 0, 0)
                    break

    except KeyboardInterrupt:
        pass


#!/usr/bin/env python3

import rospy, time, scipy
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Joy
from ros_falcon.msg import falconForces 
from gazebo_msgs.msg import LinkStates
import numpy as np
from scipy.io import savemat

# Global vars
button_r = 0
button_l = 0

rospy.init_node("Box_control", anonymous=True)

# Publishers
force_Box = rospy.Publisher("/force_B_fromsides", Wrench, queue_size=1)
force_to_left_box = rospy.Publisher("/force_left", Wrench, queue_size=1)
force_to_right_box = rospy.Publisher("/force_right", Wrench, queue_size=1)
force_to_left_falcon = rospy.Publisher("/falconForce1", falconForces, queue_size=1)
force_to_right_falcon = rospy.Publisher("/falconForce", falconForces, queue_size=1)

# Messages
msg_force_to_left_box = Wrench()
msg_force_to_right_box = Wrench()
msg_force_to_box = Wrench()
msg_force_to_left_falcon = falconForces()
msg_force_to_right_falcon = falconForces()

# States
pos_falcon_left = np.zeros(3)
pos_falcon_right = np.zeros(3)
vel_box = np.zeros(3)
pos_box = np.zeros(3)

# Trajectory trackers
Zd = np.matrix([[0.0], [0]])

# Data logging
time_store = []
pos_box_store = []
vel_box_store = []
pos_box_des_storeX = []
pos_box_des_storeY = []
pos_box_des_storeZ = []

# Force publishing functions
def left_box(x, y, z):
    msg_force_to_left_box.force.x = x
    msg_force_to_left_box.force.y = y
    msg_force_to_left_box.force.z = z
    force_to_left_box.publish(msg_force_to_left_box)

def right_box(x, y, z):
    msg_force_to_right_box.force.x = x
    msg_force_to_right_box.force.y = y
    msg_force_to_right_box.force.z = z
    force_to_right_box.publish(msg_force_to_right_box)

def left_falcon(x, y, z):
    msg_force_to_left_falcon.X = x
    msg_force_to_left_falcon.Y = z
    msg_force_to_left_falcon.Z = y
    force_to_left_falcon.publish(msg_force_to_left_falcon)

def right_falcon(x, y, z):
    msg_force_to_right_falcon.X = x
    msg_force_to_right_falcon.Y = z
    msg_force_to_right_falcon.Z = y
    force_to_right_falcon.publish(msg_force_to_right_falcon)

# Callbacks
def callback_left_falcon(data):
    global pos_falcon_left, button_l
    pos_falcon_left = np.array([data.axes[0], data.axes[2]-0.12, data.axes[1]])
    button_l = data.buttons[0]

def callback_right_falcon(data):
    global pos_falcon_right, button_r
    pos_falcon_right = np.array([data.axes[0], data.axes[2]-0.12, data.axes[1]])
    button_r = data.buttons[0]

def callback_box(data):
    global pos_box, vel_box
    pos = data.pose[1].position
    vel = data.twist[1].linear
    pos_box[:] = [pos.x, pos.y, pos.z - 0.35]
    vel_box[:] = [vel.x, vel.y, vel.z]

# Subscriptions
rospy.Subscriber("/falcon/joystick", Joy, callback_right_falcon, queue_size=1)
rospy.Subscriber("/falcon/joystick1", Joy, callback_left_falcon, queue_size=1)
rospy.Subscriber("/gazebo/link_states", LinkStates, callback_box, queue_size=1)

# Dynamics
dt = 0.01
tf = 20
M = 1.02

if __name__ == "__main__":
    try:
        program_starts = time.time()
        curr_time = 0

        while not rospy.is_shutdown():
            if time.time() - program_starts - curr_time >= dt:
                prev_time = curr_time
                curr_time = time.time() - program_starts
                t = curr_time

                # Trajectory
                if t < 10:
                    Zd += np.matrix([[dt/10], [0]])
                    print("Lift up")
                elif t < 20:
                    Zd -= np.matrix([[dt/10], [0]])
                    print("Bring down")

                # Simple impedance control (no AAN)
                uzlH = -5 * (pos_box[2] - pos_falcon_left[2]*50) - 1 * vel_box[2]
                uzrH = -5 * (pos_box[2] - pos_falcon_right[2]*50) - 10 * vel_box[2]

                # Apply forces to side boxes
                left_box(0, 0, uzlH)
                right_box(0, 0, uzrH)

                # Falcon feedback force
                if t < 10:
                    z_force_left  = -uzlH + uzrH
                    z_force_right = -uzrH + uzlH
                else:
                    z_force_left  = -uzlH + uzrH
                    z_force_right = -uzrH + uzlH

                left_falcon(0, 0, z_force_left)
                right_falcon(0, 0, z_force_right)

                # Store data
                time_store.append(t)
                pos_box_store.append(pos_box.copy())
                vel_box_store.append(vel_box.copy())
                pos_box_des_storeX.append(Zd[0,0])
                pos_box_des_storeY.append(0.0)
                pos_box_des_storeZ.append(Zd[0,0])

                # Stop condition
                if t > tf or button_r == 4 or button_l == 4:
                    left_falcon(0, 0, 0)
                    right_falcon(0, 0, 0)

                    # Save .mat files
                    scipy.io.savemat('tim_.mat', {'tim_': time_store})
                    scipy.io.savemat('pos_.mat', {'pos_': pos_box_store})
                    scipy.io.savemat('vel_.mat', {'vel_': vel_box_store})
                    scipy.io.savemat('pos_des_X.mat', {'pos_des_X': pos_box_des_storeX})
                    scipy.io.savemat('pos_des_Y.mat', {'pos_des_Y': pos_box_des_storeY})
                    scipy.io.savemat('pos_des_Z.mat', {'pos_des_Z': pos_box_des_storeZ})
                    break

    except KeyboardInterrupt:
        pass

